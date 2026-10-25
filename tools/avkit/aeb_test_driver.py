#!/usr/bin/env python3
"""aeb_test_driver.py - /maneuver/target_speed driver for AEB shadow-mode scenarios.

  python3 tools/avkit/aeb_test_driver.py --speed 1.0 --stop-on-brake --remove-after 3 --resume \\
      --ros-args -p use_sim_time:=true

Publishes /maneuver/target_speed at 10 Hz and logs events to stdout.
--stop-on-brake stands in for control_arbiter in shadow mode: the MPC
brakes more gently than AEB's 6 m/s^2 would, so it is a smoke test of the
STOP_HOLD logic, not of stopping distance.
  --stop-at-x X       set target 0 once /odom_truth x >= X  (stop-short repro)
  --stop-on-brake     set target 0 once AEB reports BRAKING (poor-man's arbiter)
  --remove-after S    after S s of continuous STOP_HOLD, delete the road_block model
  --resume            once AEB is back in MONITORING after the removal, resume V
"""
import argparse
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from av_feature_msgs.msg import AebStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

NAMES = ['INACTIVE', 'MONITORING', 'WARNING', 'BRAKING', 'STOP_HOLD', 'FAULT']


class Driver(Node):
    def __init__(self, a):
        super().__init__('aeb_test_driver')
        self.a = a
        self.target = a.speed
        self.state = None
        self.hold_since = None
        self.removed = False
        self.resumed = False
        self.x = None
        self.pub = self.create_publisher(Float64, '/maneuver/target_speed', 10)
        self.create_subscription(AebStatus, '/safety/aeb_status', self.on_aeb, 10)
        self.create_subscription(Odometry, '/odom_truth', self.on_odom, 20)
        self.create_timer(0.1, self.tick)

    def t(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def log(self, msg):
        print(f'[{self.t():8.2f}] {msg}', flush=True)

    def on_odom(self, m):
        self.x = m.pose.pose.position.x
        if self.a.stop_at_x is not None and self.target > 0 and not self.resumed \
                and self.x >= self.a.stop_at_x:
            self.target = 0.0
            self.log(f'stop_at_x: target -> 0 at truth x={self.x:.2f}')

    def on_aeb(self, m):
        s = NAMES[m.state] if m.state < len(NAMES) else str(m.state)
        if s != self.state:
            self.log(f'AEB {self.state} -> {s}  dist={m.obstacle_distance_m:.2f} '
                     f'ttc={min(m.time_to_collision_sec, 1e9):.2f} x={self.x}')
            self.state = s
            self.hold_since = self.t() if s == 'STOP_HOLD' else None
        if self.a.stop_on_brake and s == 'BRAKING' and self.target > 0 and not self.resumed:
            self.target = 0.0
            self.log('stop_on_brake: target -> 0')
        if self.a.remove_after is not None and not self.removed and s == 'STOP_HOLD' \
                and self.hold_since is not None and self.t() - self.hold_since >= self.a.remove_after:
            self.removed = True
            r = subprocess.run(
                ['ign', 'service', '-s', f'/world/{self.a.world}/remove',
                 '--reqtype', 'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean',
                 '--timeout', '3000', '--req', 'name: "road_block" type: MODEL'],
                capture_output=True, text=True)
            self.log(f'removed road_block: {r.stdout.strip()} {r.stderr.strip()}')
        if self.a.resume and self.removed and not self.resumed and s == 'MONITORING':
            self.resumed = True
            self.target = self.a.speed
            self.log(f'resume: target -> {self.a.speed}')

    def tick(self):
        self.pub.publish(Float64(data=self.target))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, required=True)
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--world', default='urban_course_roadblock')
    ap.add_argument('--stop-at-x', type=float, default=None)
    ap.add_argument('--stop-on-brake', action='store_true')
    ap.add_argument('--remove-after', type=float, default=None)
    ap.add_argument('--resume', action='store_true')
    a, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init(args=sys.argv)
    n = Driver(a)
    deadline = time.monotonic() + a.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
