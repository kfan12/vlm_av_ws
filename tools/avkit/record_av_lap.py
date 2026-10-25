#!/usr/bin/env python3
"""record_av_lap.py - minimal lap recorder for the AV stack.

Subscribes to the stack's key topics and writes one JSONL file per signal
into a session dir.

  python3 tools/avkit/record_av_lap.py --label baseline --duration 240 \
      --world urban_course --ros-args -p use_sim_time:=true

Session dir: data/av_laps/<YYYYmmdd_HHMMSS>_<label>/
  odom_truth.jsonl   /odom_truth  {"t","x","y","yaw","v"}  absolute world frame
  odom_ekf.jsonl     /odom_ekf    {"t","x","y","yaw","v"}  spawn-zeroed odom frame
  cmd_vel.jsonl      {"t","v","wz"}
  v_ref.jsonl        {"t","v"}      (/mpc/v_ref_ramped)
  target_speed.jsonl {"t","v"}      (/maneuver/target_speed)
  state.jsonl        {"t","state"}  (/maneuver/state)
  aeb_state.jsonl    {"t","state","obstacle_distance_m","obstacle_id","reason"}
                     (/safety/aeb_status, only if aeb_node runs)
  plan_path.jsonl    {"t","pts":[[x,y],...]}  (1 Hz snapshots, odom frame)
  lane_path.jsonl    same
  meta.json          label, world, map seed, duration, per-signal row counts

Every row is stamped with the node's clock at arrival (sim time with
use_sim_time), so all streams share one time base; rows that arrive before
/clock (t = 0) are dropped.

/odom_truth is recorded in the absolute world frame, not spawn-zeroed
(the analyzer relies on this); aeb_state added so laps with the arbiter can count
FAULT ticks; meta carries world + seed so baselines are only compared
within one world.

Constraints inherited from the debugkit: no pandas; never rclpy.shutdown()
inside a callback (spin_once + wall-clock deadline instead).
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
from av_feature_msgs.msg import AebStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, String

AEB_STATE_NAMES = ['INACTIVE', 'MONITORING', 'WARNING', 'BRAKING', 'STOP_HOLD', 'FAULT']
STREAMS = ('odom_truth', 'odom_ekf', 'cmd_vel', 'v_ref', 'target_speed', 'state',
           'aeb_state', 'plan_path', 'lane_path')


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LapRecorder(Node):
    def __init__(self, out_dir):
        super().__init__('lap_recorder')
        self.files = {n: open(os.path.join(out_dir, n + '.jsonl'), 'w') for n in STREAMS}
        self.n_rows = {n: 0 for n in STREAMS}
        self.last_path_t = {}

        self.create_subscription(Odometry, '/odom_truth', self._odom_cb('odom_truth'), 20)
        self.create_subscription(Odometry, '/odom_ekf', self._odom_cb('odom_ekf'), 20)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Float64, '/mpc/v_ref_ramped', self._float_cb('v_ref'), 10)
        self.create_subscription(Float64, '/maneuver/target_speed',
                                 self._float_cb('target_speed'), 10)
        self.create_subscription(String, '/maneuver/state', self._state_cb, 10)
        self.create_subscription(AebStatus, '/safety/aeb_status', self._aeb_cb, 10)
        self.create_subscription(Path, '/plan/path_odom', self._path_cb('plan_path'), 10)
        self.create_subscription(Path, '/lane/path_odom', self._path_cb('lane_path'), 10)

    def _t(self):
        return round(self.get_clock().now().nanoseconds * 1e-9, 3)

    def _emit(self, name, row):
        if row['t'] <= 0.0:
            return  # /clock not received yet: sim-time stamp would be 0
        self.files[name].write(json.dumps(row) + '\n')
        self.n_rows[name] += 1

    def _odom_cb(self, name):
        def cb(m):
            p = m.pose.pose.position
            self._emit(name, {'t': self._t(), 'x': round(p.x, 4), 'y': round(p.y, 4),
                              'yaw': round(_yaw(m.pose.pose.orientation), 4),
                              'v': round(math.hypot(m.twist.twist.linear.x,
                                                    m.twist.twist.linear.y), 4)})
        return cb

    def _cmd_cb(self, m):
        self._emit('cmd_vel', {'t': self._t(), 'v': round(m.linear.x, 4),
                               'wz': round(m.angular.z, 4)})

    def _float_cb(self, name):
        def cb(m):
            self._emit(name, {'t': self._t(), 'v': round(m.data, 4)})
        return cb

    def _state_cb(self, m):
        self._emit('state', {'t': self._t(), 'state': m.data})

    def _aeb_cb(self, m):
        self._emit('aeb_state', {
            't': self._t(),
            'state': AEB_STATE_NAMES[m.state] if m.state < len(AEB_STATE_NAMES) else 'UNKNOWN',
            'obstacle_distance_m': round(m.obstacle_distance_m, 3),
            'obstacle_id': m.obstacle_id,
            'reason': m.reason})

    def _path_cb(self, name):
        def cb(m):
            t = self._t()
            # 1 Hz snapshots are plenty for offline analysis
            if t - self.last_path_t.get(name, -1e18) < 1.0:
                return
            self.last_path_t[name] = t
            self._emit(name, {'t': t, 'pts': [[round(p.pose.position.x, 3),
                                               round(p.pose.position.y, 3)]
                                              for p in m.poses]})
        return cb

    def close(self):
        for f in self.files.values():
            f.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', default='lap')
    ap.add_argument('--duration', type=float, default=240.0)
    ap.add_argument('--world', default='urban_course',
                    help='world name; its maps/<world>.json seed is copied into meta.json')
    ap.add_argument('--out', default=os.path.expanduser('~/vlm_av_ws/data/av_laps'))
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    out_dir = os.path.join(args.out, datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + args.label)
    os.makedirs(out_dir, exist_ok=True)

    seed = None
    try:
        with open(os.path.expanduser(f'~/vlm_av_ws/maps/{args.world}.json')) as f:
            seed = json.load(f).get('seed')
    except OSError:
        pass

    rclpy.init(args=sys.argv)
    node = LapRecorder(out_dir)
    print(f'recording -> {out_dir}  ({args.duration:.0f} s wall, Ctrl-C stops early)', flush=True)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.close()
    with open(os.path.join(out_dir, 'meta.json'), 'w') as f:
        json.dump({'label': args.label, 'world': args.world, 'seed': seed,
                   'duration_s': args.duration, 'rows': node.n_rows}, f, indent=1)
    print('rows:', node.n_rows, flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
