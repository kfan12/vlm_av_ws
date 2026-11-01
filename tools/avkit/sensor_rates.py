#!/usr/bin/env python3
"""sensor_rates.py - wall-clock message rates, worst gaps and the sim real-time factor.

  python3 tools/avkit/sensor_rates.py --duration 120 --out data/sensor_load/lidar_on.json

Subscribes raw (no deserialisation, so the probe itself stays cheap) to each
TOPIC:TYPE and to /clock. Reports per topic: messages, mean Hz, median and max
inter-arrival gap, 99th-percentile gap, silence at the end; plus sim seconds
per wall second. The time after a topic's last message up to the end of the
probe counts as a gap, so a stall late in the window is not hidden.
Wall clock throughout - the question is what the CPU-bound sim delivers.
"""
import argparse
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message

DEFAULT_TOPICS = [
    '/camera/front/image_raw:sensor_msgs/msg/Image',
    '/camera/front/depth/image_raw:sensor_msgs/msg/Image',
    '/lidar/front/points:sensor_msgs/msg/PointCloud2',
    '/lane/path_odom:nav_msgs/msg/Path',
    '/safety/aeb_status:av_feature_msgs/msg/AebStatus',
    '/odom_truth:nav_msgs/msg/Odometry',
]


class RateProbe(Node):
    def __init__(self, specs):
        super().__init__('sensor_rates')
        self.arrivals = {}
        for spec in specs:
            topic, type_name = spec.split(':')
            self.arrivals[topic] = []
            self.create_subscription(get_message(type_name), topic,
                                     lambda _m, t=topic: self.arrivals[t].append(time.monotonic()),
                                     qos_profile_sensor_data, raw=True)
        self.clock = []   # (wall, sim)
        self.create_subscription(Clock, '/clock', self._clock_cb, qos_profile_sensor_data)

    def _clock_cb(self, m):
        self.clock.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9))


def stats(ts, end):
    """Rates over [first arrival, end of the probe]: the silence after the last
    message counts as a gap, so a sensor (or the whole sim) that stops before
    the probe ends shows up as a long max gap and a lower rate, not as a clean
    shorter run. The window starts at the first arrival to leave discovery out."""
    if len(ts) < 2:
        return {'messages': len(ts), 'hz': 0.0}
    gaps = sorted([b - a for a, b in zip(ts, ts[1:])] + [end - ts[-1]])
    return {'messages': len(ts),
            'hz': round(len(ts) / (end - ts[0]), 2),
            'median_gap_s': round(gaps[len(gaps) // 2], 3),
            'p99_gap_s': round(gaps[min(len(gaps) - 1, int(0.99 * len(gaps)))], 3),
            'max_gap_s': round(gaps[-1], 3),
            'silent_at_end_s': round(end - ts[-1], 3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=120.0)
    ap.add_argument('--topic', action='append', default=None, metavar='TOPIC:TYPE')
    ap.add_argument('--out', default=None)
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init(args=sys.argv)
    node = RateProbe(args.topic or DEFAULT_TOPICS)
    deadline = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    end = time.monotonic()
    out = {'duration_s': args.duration, 'topics': {t: stats(ts, end) for t, ts in node.arrivals.items()}}
    if len(node.clock) >= 2:
        # over the whole window, so a /clock that stops counts as sim time not advancing
        (w0, s0), (_, s1) = node.clock[0], node.clock[-1]
        out['real_time_factor'] = round((s1 - s0) / (end - w0), 3)
    print(json.dumps(out, indent=1))
    if args.out:
        os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
        with open(args.out, 'w') as f:
            json.dump(out, f, indent=1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
