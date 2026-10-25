#!/usr/bin/env python3
"""record_aeb_scenario.py - AEB scenario recorder.

  python3 tools/avkit/record_aeb_scenario.py --label roadblock_3mps \
      --duration 40 --world urban_course_roadblock --target-speed 3.0 \
      [--obstacle aeb_test_box] --ros-args -p use_sim_time:=true

Session dir: data/aeb_scenarios/<YYYYmmdd_HHMMSS>_<label>/
  aeb_status.jsonl   every /safety/aeb_status field (state name, distances, TTC, ...)
  aeb_cmd.jsonl      /control/aeb_cmd   {"t","v","wz"}  (arbiter input; shadow: unused)
  mpc_cmd.jsonl      /control/mpc_cmd   {"t","v","wz"}  (arbiter input; empty with aeb:=false)
  cmd_vel.jsonl      /cmd_vel           {"t","v","wz"}  (what the car is told to do)
  active_source.jsonl /control/active_source {"t","source"}  (control_arbiter's pick)
  odom_truth.jsonl   /odom_truth        {"t","x","y","yaw","v"}  absolute world frame
  odom_ekf.jsonl     /odom_ekf          {"t","x","y","yaw","v"}  spawn-zeroed frame
  lane_path.jsonl    /lane/path_odom    {"t","n"}  arrival time + pose count
  plan_path.jsonl    /plan/path_odom    {"t","n"}
  target_speed.jsonl /maneuver/target_speed {"t","v"}
  clock.jsonl        {"t","wall"}  sim time vs wall clock, every ~0.5 s wall
  meta.json          label, world, map road_block pose, --obstacle records,
                     target speed, row counts

With aeb:=true (arbiter in the loop) active_source/mpc_cmd fill up; in the
aeb:=false shadow setup they stay empty and /cmd_vel is the MPC's own command.

--obstacle NAME (repeatable) copies data/aeb_obstacles/NAME.json, written by
scripts/spawn_aeb_obstacle.sh, into meta.json at the END of the run, so a box
spawned after the recorder started is still captured; clock.jsonl lets the
analyzer map its wall-clock spawn/remove times onto sim time.

Same constraints as record_av_lap.py: no pandas; spin_once + a
wall-clock deadline, never rclpy.shutdown() from a callback.
"""
import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from av_feature_msgs.msg import AebStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, String

STATE_NAMES = ['INACTIVE', 'MONITORING', 'WARNING', 'BRAKING', 'STOP_HOLD', 'FAULT']
STREAMS = ('aeb_status', 'aeb_cmd', 'mpc_cmd', 'cmd_vel', 'active_source', 'odom_truth',
           'odom_ekf', 'lane_path', 'plan_path', 'target_speed', 'clock')
OBSTACLE_DIR = os.path.expanduser('~/vlm_av_ws/data/aeb_obstacles')


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class AebScenarioRecorder(Node):
    def __init__(self, out_dir):
        super().__init__('aeb_scenario_recorder')
        self.files = {n: open(os.path.join(out_dir, n + '.jsonl'), 'w') for n in STREAMS}
        self.n_rows = {n: 0 for n in STREAMS}

        self.create_subscription(AebStatus, '/safety/aeb_status', self._aeb_cb, 10)
        self.create_subscription(Twist, '/control/aeb_cmd', self._twist_cb('aeb_cmd'), 10)
        self.create_subscription(Twist, '/control/mpc_cmd', self._twist_cb('mpc_cmd'), 10)
        self.create_subscription(String, '/control/active_source', self._source_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._twist_cb('cmd_vel'), 10)
        self.create_subscription(Odometry, '/odom_truth', self._odom_cb('odom_truth'), 20)
        self.create_subscription(Odometry, '/odom_ekf', self._odom_cb('odom_ekf'), 20)
        self.create_subscription(Path, '/lane/path_odom', self._path_cb('lane_path'), 10)
        self.create_subscription(Path, '/plan/path_odom', self._path_cb('plan_path'), 10)
        self.create_subscription(Float64, '/maneuver/target_speed', self._target_cb,
                                 qos_profile_sensor_data)

    def _t(self):
        return round(self.get_clock().now().nanoseconds * 1e-9, 3)

    def _emit(self, name, row):
        if row['t'] <= 0.0:
            return  # /clock not received yet: sim-time stamp would be 0
        self.files[name].write(json.dumps(row) + '\n')
        self.n_rows[name] += 1

    def _aeb_cb(self, m):
        self._emit('aeb_status', {
            't': self._t(),
            'stamp': round(_stamp(m.header), 3),
            'state': STATE_NAMES[m.state] if m.state < len(STATE_NAMES) else 'UNKNOWN',
            'emergency_active': bool(m.emergency_active),
            'obstacle_distance_m': round(m.obstacle_distance_m, 3),
            'relative_speed_mps': round(m.relative_speed_mps, 3),
            'ttc_sec': round(min(m.time_to_collision_sec, 1e9), 3),
            'd_safe_m': round(m.safe_braking_distance_m, 3),
            'requested_decel_mps2': round(m.requested_deceleration_mps2, 3),
            'obstacle_id': m.obstacle_id,
            'reason': m.reason})

    def _twist_cb(self, name):
        def cb(m):
            self._emit(name, {'t': self._t(), 'v': round(m.linear.x, 4),
                              'wz': round(m.angular.z, 4)})
        return cb

    def _odom_cb(self, name):
        def cb(m):
            p = m.pose.pose.position
            self._emit(name, {'t': self._t(), 'x': round(p.x, 4), 'y': round(p.y, 4),
                              'yaw': round(_yaw(m.pose.pose.orientation), 4),
                              'v': round(math.hypot(m.twist.twist.linear.x,
                                                    m.twist.twist.linear.y), 4)})
        return cb

    def _path_cb(self, name):
        def cb(m):
            self._emit(name, {'t': self._t(), 'n': len(m.poses)})
        return cb

    def _source_cb(self, m):
        self._emit('active_source', {'t': self._t(), 'source': m.data})

    def emit_clock(self):
        self._emit('clock', {'t': self._t(), 'wall': round(time.time(), 3)})

    def _target_cb(self, m):
        self._emit('target_speed', {'t': self._t(), 'v': round(m.data, 3)})

    def close(self):
        for f in self.files.values():
            f.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', default='scenario')
    ap.add_argument('--duration', type=float, default=40.0)
    ap.add_argument('--world', default='urban_course_roadblock',
                    help='world name; its maps/<world>.json road_block is copied into meta.json')
    ap.add_argument('--target-speed', type=float, default=None,
                    help='the /maneuver/target_speed being published for this run (meta only)')
    ap.add_argument('--obstacle', action='append', default=[], metavar='NAME',
                    help='spawn_aeb_obstacle.sh box to record as ground truth (repeatable)')
    ap.add_argument('--out', default=os.path.expanduser('~/vlm_av_ws/data/aeb_scenarios'))
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    out_dir = os.path.join(args.out, datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + args.label)
    os.makedirs(out_dir, exist_ok=True)

    road_block = None
    map_path = os.path.expanduser(f'~/vlm_av_ws/maps/{args.world}.json')
    try:
        with open(map_path) as f:
            road_block = json.load(f).get('road_block')
    except OSError:
        pass

    rclpy.init(args=sys.argv)
    node = AebScenarioRecorder(out_dir)
    print(f'recording -> {out_dir}  ({args.duration:.0f} s wall, Ctrl-C stops early)', flush=True)
    deadline = time.monotonic() + args.duration
    next_clock = 0.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() >= next_clock:
                node.emit_clock()
                next_clock = time.monotonic() + 0.5
    except KeyboardInterrupt:
        pass
    node.emit_clock()
    node.close()
    obstacles = []
    for name in args.obstacle:
        try:
            with open(os.path.join(OBSTACLE_DIR, name + '.json')) as f:
                obstacles.append(json.load(f))
        except OSError:
            print(f'warn: no spawn record for obstacle {name!r} in {OBSTACLE_DIR}', flush=True)
    with open(os.path.join(out_dir, 'meta.json'), 'w') as f:
        json.dump({'label': args.label, 'world': args.world, 'road_block': road_block,
                   'obstacles': obstacles,
                   'target_speed_mps': args.target_speed, 'duration_s': args.duration,
                   'rows': node.n_rows}, f, indent=1)
    print('rows:', node.n_rows, flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
