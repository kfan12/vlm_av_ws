#!/usr/bin/env python3
"""_course_demo_watch.py aeb|ctrl - event-driven watchers for the aeb demo panes.

AEB transitions (WARNING -> BRAKING -> STOP_HOLD) can last well under a
second, so a 1 Hz `ros2 topic echo --once` poll misses them. These print a
'>>' line on every change plus a heartbeat line once a second.

  aeb   /safety/aeb_status: state, emergency flag, distance, TTC, d_safe, decel
  ctrl  /control/active_source (who owns /cmd_vel) + /cmd_vel + target_speed

System python (rclpy + the workspace's av_feature_msgs), started by
_course_demo_pane.sh panes 3 and 4 in aeb mode.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String

from av_feature_msgs.msg import AebStatus

STATE_NAMES = {
    AebStatus.INACTIVE: 'INACTIVE', AebStatus.MONITORING: 'MONITORING',
    AebStatus.WARNING: 'WARNING', AebStatus.BRAKING: 'BRAKING',
    AebStatus.STOP_HOLD: 'STOP_HOLD', AebStatus.FAULT: 'FAULT',
}
HEARTBEAT_SEC = 1.0
NO_TTC = 1e6  # aeb_node reports "no closing obstacle" as a huge TTC (1e9), not inf


def _num(x, width, prec):
    return f'{x:{width}.{prec}f}' if math.isfinite(x) and abs(x) < NO_TTC else f'{"inf":>{width}}'


class Watch(Node):
    def __init__(self, mode):
        super().__init__(f'course_demo_watch_{mode}')
        self.last_key = None
        self.last_print = 0.0
        if mode == 'aeb':
            self.create_subscription(AebStatus, '/safety/aeb_status', self._aeb_cb, 10)
        else:
            self.cmd = Twist()
            self.target = float('nan')
            self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
            self.create_subscription(Float64, '/maneuver/target_speed', self._target_cb, 10)
            self.create_subscription(String, '/control/active_source', self._source_cb, 10)

    def _emit(self, key, line):
        now = time.monotonic()
        changed = key != self.last_key
        if changed or now - self.last_print >= HEARTBEAT_SEC:
            print(f'{">>" if changed else "  "} {line}', flush=True)
            self.last_key = key
            self.last_print = now

    def _aeb_cb(self, m):
        state = STATE_NAMES.get(m.state, str(m.state))
        dist = 'none' if m.obstacle_distance_m < 0 else _num(m.obstacle_distance_m, 6, 2)
        self._emit(state,
                   f'{state:<10} emerg={int(m.emergency_active)} dist={dist:>6} '
                   f'ttc={_num(m.time_to_collision_sec, 6, 2)} '
                   f'd_safe={_num(m.safe_braking_distance_m, 5, 2)} '
                   f'decel={_num(m.requested_deceleration_mps2, 4, 1)}  {m.reason}')

    def _cmd_cb(self, m):
        self.cmd = m

    def _target_cb(self, m):
        self.target = m.data

    def _source_cb(self, m):
        target = 'none' if math.isnan(self.target) else f'{self.target:.1f}'
        self._emit(m.data,
                   f'source={m.data:<22} v={self.cmd.linear.x:5.2f} m/s  '
                   f'w={self.cmd.angular.z:+5.2f} rad/s  target_speed={target}')


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else ''
    if mode not in ('aeb', 'ctrl'):
        sys.exit('usage: _course_demo_watch.py aeb|ctrl')
    rclpy.init()
    try:
        rclpy.spin(Watch(mode))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
