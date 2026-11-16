#!/usr/bin/env python3
"""ros_probe.py - read a running stack without the ros2 CLI's pitfalls.

  ros_probe.py field TOPIC FIELD [--for SEC] [--every N]   one "t value" line per message
  ros_probe.py rate TOPIC [TOPIC ...] [--for SEC]          rate, worst gap and count per topic
  ros_probe.py graph [--for SEC]                           every node with its publishers and subscribers

  ros_probe.py field /odom_truth pose.pose.position.x --for 10
  ros_probe.py rate /camera/front/depth/image_raw /lidar/front/points --for 15
  ros_probe.py graph --for 30

Why not `ros2 topic echo | awk`: in a pipe the CLI block-buffers, so a timeout
or Ctrl-C loses the output; and `ros2 node list` / `ros2 node info` see only
part of the graph on this WSL setup. Every line here is flushed as it is
printed, the run ends by itself after --for seconds (wall clock), and the graph
is the union of what was discovered over the whole window.
t is the message's header stamp when it has one (sim time), else seconds since
the first message. Exit 1 when a topic never appeared or sent nothing.
"""
import argparse
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def wait_type(node, topic, timeout=8.0):
    """The topic's message type, once discovery has seen it."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        for name, types in node.get_topic_names_and_types():
            if name == topic:
                return types[0]
        rclpy.spin_once(node, timeout_sec=0.2)
    return None


def get_field(msg, path):
    for part in path.split('.'):
        if part.endswith(']'):
            name, idx = part[:-1].split('[')
            msg = getattr(msg, name)[int(idx)]
        else:
            msg = getattr(msg, part)
    return msg


def stamp_of(msg):
    header = getattr(msg, 'header', None)
    if header is None:
        return None
    t = header.stamp.sec + header.stamp.nanosec * 1e-9
    return t if t > 0 else None  # an unstamped message falls back to elapsed time


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, end - time.monotonic())))


def cmd_field(node, a):
    ty = wait_type(node, a.topic)
    if ty is None:
        print(f'{a.topic}: no such topic within 8 s', file=sys.stderr)
        return 1
    state = {'n': 0, 't0': None}

    def on_msg(msg):
        state['n'] += 1
        if (state['n'] - 1) % a.every:
            return
        now = time.monotonic()
        if state['t0'] is None:
            state['t0'] = now
        t = stamp_of(msg)
        t = now - state['t0'] if t is None else t
        try:
            value = get_field(msg, a.field)
        except (AttributeError, IndexError, ValueError) as e:
            value = f'<{type(e).__name__}: {e}>'
        print(f'{t:.3f} {value}', flush=True)

    # best effort matches both reliable and best-effort publishers
    node.create_subscription(get_message(ty), a.topic, on_msg, qos_profile_sensor_data)
    spin_for(node, a.seconds)
    if state['n'] == 0:
        print(f'{a.topic}: no message in {a.seconds:.0f} s', file=sys.stderr)
        return 1
    return 0


def cmd_rate(node, a):
    stats = {}
    # resolve every type before subscribing, so no topic counts while another is still being looked up
    types = {topic: wait_type(node, topic) for topic in a.topics}
    for topic, ty in types.items():
        if ty is None:
            stats[topic] = None
            continue
        s = stats[topic] = {'n': 0, 'first': None, 'last': None, 'gap': 0.0}

        def on_msg(_msg, s=s):
            now = time.monotonic()
            if s['last'] is not None:
                s['gap'] = max(s['gap'], now - s['last'])
            if s['first'] is None:
                s['first'] = now
            s['last'] = now
            s['n'] += 1

        node.create_subscription(get_message(ty), topic, on_msg, qos_profile_sensor_data)
    spin_for(node, a.seconds)
    rc = 0
    for topic, s in stats.items():
        if s is None or s['n'] == 0:
            print(f'{topic}: ' + ('no such topic' if s is None else f'no message in {a.seconds:.0f} s'), flush=True)
            rc = 1
            continue
        span = s['last'] - s['first']
        hz = (s['n'] - 1) / span if span > 0 else 0.0
        print(f'{topic}: {hz:.2f} Hz, worst gap {s["gap"]:.3f} s, {s["n"]} messages in {a.seconds:.0f} s (wall)', flush=True)
    return rc


def cmd_graph(node, a):
    graph = {}
    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=1.0)
        for name, ns in node.get_node_names_and_namespaces():
            if name == node.get_name():
                continue
            g = graph.setdefault(ns.rstrip('/') + '/' + name, {'pub': {}, 'sub': {}})
            try:
                for t, ty in node.get_publisher_names_and_types_by_node(name, ns):
                    g['pub'][t] = ty[0]
                for t, ty in node.get_subscriber_names_and_types_by_node(name, ns):
                    g['sub'][t] = ty[0]
            except Exception:  # a node that left between the two calls
                pass
    for full in sorted(graph):
        print(full)
        print('  Subscribers:')
        for t, ty in sorted(graph[full]['sub'].items()):
            print(f'    {t}: {ty}')
        print('  Publishers:')
        for t, ty in sorted(graph[full]['pub'].items()):
            print(f'    {t}: {ty}')
    print(f'{len(graph)} nodes over {a.seconds:.0f} s', flush=True)
    return 0 if graph else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    f = sub.add_parser('field')
    f.add_argument('topic')
    f.add_argument('field')
    f.add_argument('--every', type=int, default=1, help='print every Nth message')
    r = sub.add_parser('rate')
    r.add_argument('topics', nargs='+')
    g = sub.add_parser('graph')
    for p, default in ((f, 10.0), (r, 10.0), (g, 30.0)):
        p.add_argument('--for', dest='seconds', type=float, default=default, help='wall-clock seconds')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('ros_probe')
    try:
        return {'field': cmd_field, 'rate': cmd_rate, 'graph': cmd_graph}[a.cmd](node, a)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
