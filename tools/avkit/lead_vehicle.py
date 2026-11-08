#!/usr/bin/env python3
"""lead_vehicle.py - a drivable lead vehicle for ACC scenarios.

  python3 tools/avkit/lead_vehicle.py spawn --s 25            # right lane, course arc length
  python3 tools/avkit/lead_vehicle.py drive --profile 0:1.5 --ros-args -p use_sim_time:=true
  python3 tools/avkit/lead_vehicle.py remove

spawn: a 6.0 x 2.1 x 3.0 m box truck (red cab, blue cargo box, three axles)
named lead_vehicle with VelocityControl (/model/lead_vehicle/cmd_vel) and
OdometryPublisher (/model/lead_vehicle/odometry, world frame, at the centre of
the footprint on the ground) attached; one frictionless collision box, so the
commanded velocity is what it does. Bridge both topics to ROS as /lead/cmd_vel and
/lead/odom_truth (run_acc_scenario.sh does this).

drive: pure pursuit along the right-lane line of maps/<world>.json at the
speed of a piecewise-linear profile "t0:v0,t1:v1,..." (sim seconds since the
driver started, m/s; holds the last value). Publishes zero on exit.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aeb_obstacle import (detect_world, ign_call, load_course, model_names,  # noqa: E402
                          pose_at_s, wait_for)

NAME = 'lead_vehicle'
# Overall footprint of the box truck (along road, across road, height) [m].
# The model origin is on the ground at the centre of the footprint, so the
# odometry's position is LENGTH / 2 ahead of the rear face (analyze_acc_run.py
# subtracts that).
SIZE = (6.0, 2.1, 3.0)
LENGTH, WIDTH, HEIGHT = SIZE
REAR_X = -LENGTH / 2.0

# Saturated, non-yellow colours: lane_node finds paint as low-saturation bright
# pixels (white) or yellow hues, so a white or yellow truck would turn into lane
# markings wherever the camera sees it.
CAB_RGBA = '0.60 0.08 0.08 1'      # red
CARGO_RGBA = '0.12 0.28 0.60 1'    # blue
DARK_RGBA = '0.06 0.06 0.06 1'     # chassis, tyres, bumper
GLASS_RGBA = '0.08 0.10 0.14 1'
LAMP_RGBA = '0.80 0.05 0.05 1'


def _visual(name, size, xyz, rgba, rpy=(0.0, 0.0, 0.0), cylinder=False):
    if cylinder:   # size = (radius, length)
        geom = f"<cylinder><radius>{size[0]}</radius><length>{size[1]}</length></cylinder>"
    else:
        geom = f"<box><size>{size[0]} {size[1]} {size[2]}</size></box>"
    return (f"<visual name='{name}'><pose>{xyz[0]} {xyz[1]} {xyz[2]} {rpy[0]} {rpy[1]} {rpy[2]}</pose>"
            f"<geometry>{geom}</geometry>"
            f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual>")


def truck_visuals():
    """A box truck: cargo box behind a cab, on a chassis with three axles.
    Rendered sensors (camera, gpu_lidar) see these; the rear face of the cargo
    box and the underride bar below it are what a follower's lidar hits."""
    cargo_len, cargo_x0 = 3.9, REAR_X                    # cargo box from the rear face forward
    cab_len = LENGTH - cargo_len - 0.1                   # 0.1 m gap between box and cab
    cab_x0 = cargo_x0 + cargo_len + 0.1
    r_wheel, w_tyre = 0.45, 0.30
    y_wheel = WIDTH / 2.0 - w_tyre / 2.0
    v = [
        _visual('cargo', (cargo_len, WIDTH, 2.1), (cargo_x0 + cargo_len / 2, 0, 0.9 + 2.1 / 2), CARGO_RGBA),
        _visual('cab', (cab_len, WIDTH - 0.1, 1.8), (cab_x0 + cab_len / 2, 0, 0.7 + 1.8 / 2), CAB_RGBA),
        _visual('windshield', (0.02, WIDTH - 0.4, 0.7), (LENGTH / 2 + 0.005, 0, 2.0), GLASS_RGBA),
        _visual('chassis', (LENGTH - 0.2, 1.0, 0.3), (0, 0, 0.65), DARK_RGBA),
        # rear underride bar, as on real trucks: low enough for a close follower to see
        _visual('underride', (0.1, WIDTH - 0.2, 0.2), (REAR_X + 0.05, 0, 0.5), DARK_RGBA),
    ]
    for side, y in (('l', y_wheel), ('r', -y_wheel)):
        # 1 cm proud of the rear face (flush faces flicker in the render)
        v.append(_visual(f'lamp_{side}', (0.02, 0.25, 0.15), (REAR_X, y * 0.9, 1.1), LAMP_RGBA))
        for i, x in enumerate((REAR_X + 1.0, REAR_X + 2.0, LENGTH / 2 - 0.9)):
            v.append(_visual(f'wheel_{side}{i}', (r_wheel, w_tyre), (x, y, r_wheel), DARK_RGBA,
                             rpy=(1.5708, 0, 0), cylinder=True))
    return ''.join(v)


def lead_sdf():
    l, w, h = SIZE
    m = 3000.0
    inertia = (f"<inertia><ixx>{m * (w * w + h * h) / 12:.1f}</ixx><iyy>{m * (l * l + h * h) / 12:.1f}</iyy>"
               f"<izz>{m * (l * l + w * w) / 12:.1f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>")
    # one frictionless box for physics (VelocityControl sets the motion); the
    # truck shape is visual only
    return (f"<sdf version='1.9'><model name='{NAME}'><link name='body'>"
            f"<inertial><pose>0 0 {h / 2} 0 0 0</pose><mass>{m}</mass>{inertia}</inertial>"
            f"<collision name='c'><pose>0 0 {h / 2} 0 0 0</pose>"
            f"<geometry><box><size>{l} {w} {h}</size></box></geometry>"
            f"<surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>"
            f"{truck_visuals()}"
            f"</link>"
            f"<plugin filename='ignition-gazebo-velocity-control-system' "
            f"name='ignition::gazebo::systems::VelocityControl'/>"
            f"<plugin filename='ignition-gazebo-odometry-publisher-system' "
            f"name='ignition::gazebo::systems::OdometryPublisher'>"
            f"<odom_topic>/model/{NAME}/odometry</odom_topic><dimensions>3</dimensions>"
            f"<odom_frame>world</odom_frame><robot_base_frame>{NAME}</robot_base_frame></plugin>"
            f"</model></sdf>")


def lane_line(course, lane_width):
    """Right-lane centre line as [(x, y)], from the course centerline."""
    out = []
    for i in range(len(course)):
        s = course[i][0]
        cx, cy, h = pose_at_s(course, s)
        out.append((cx + (lane_width / 2.0) * math.sin(h), cy - (lane_width / 2.0) * math.cos(h)))
    return out


def spawn(a):
    world = a.world or detect_world()
    course, lane_width = load_course(world)
    if course is None:
        sys.exit(f'needs maps/{world}.json')
    if NAME in model_names(world):
        sys.exit(f'{NAME} already exists; remove it first')
    cx, cy, yaw = pose_at_s(course, a.s)
    x, y = cx + (lane_width / 2.0) * math.sin(yaw), cy - (lane_width / 2.0) * math.cos(yaw)
    req = (f'sdf: "{lead_sdf()}" name: "{NAME}" allow_renaming: false '
           f'pose {{ position {{ x: {x:.3f} y: {y:.3f} z: 0.010 }} '
           f'orientation {{ z: {math.sin(yaw / 2):.6f} w: {math.cos(yaw / 2):.6f} }} }}')
    ok, out = ign_call(world, 'create', 'ignition.msgs.EntityFactory', req)
    if not ok or not wait_for(world, NAME, present=True):
        sys.exit(f'spawn failed: {out}')
    print(f'spawned {NAME} at s={a.s} ({x:.2f}, {y:.2f}) in {world}')


def remove(a):
    world = a.world or detect_world()
    if NAME not in model_names(world):
        sys.exit(f'no {NAME} in {world}')
    ok, out = ign_call(world, 'remove', 'ignition.msgs.Entity', f'name: "{NAME}" type: MODEL')
    if not ok or not wait_for(world, NAME, present=False):
        sys.exit(f'remove failed: {out}')
    print(f'removed {NAME}')


def parse_profile(text):
    pts = sorted((float(t), float(v)) for t, v in (p.split(':') for p in text.split(',')))
    if not pts:
        sys.exit('empty --profile')
    return pts


def profile_speed(pts, t):
    if t <= pts[0][0]:
        return pts[0][1]
    for (t0, v0), (t1, v1) in zip(pts, pts[1:]):
        if t <= t1:
            return v0 + (v1 - v0) * (t - t0) / max(1e-9, t1 - t0)
    return pts[-1][1]


def drive(a):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    world = a.world or detect_world()
    course, lane_width = load_course(world)
    lane = lane_line(course, lane_width)
    profile = parse_profile(a.profile)

    class Driver(Node):
        def __init__(self):
            super().__init__('lead_driver')
            self.odom = None
            self.t0 = None
            self.i = 0   # nearest lane index, monotone along the course
            self.pub = self.create_publisher(Twist, '/lead/cmd_vel', 10)
            self.create_subscription(Odometry, '/lead/odom_truth', self._odom, 10)
            self.create_timer(1.0 / a.rate, self.tick)

        def _odom(self, m):
            self.odom = m

        def tick(self):
            if self.odom is None:
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.t0 is None:
                self.t0 = now
            v = profile_speed(profile, now - self.t0)
            p = self.odom.pose.pose.position
            q = self.odom.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            while self.i + 1 < len(lane) and \
                    math.dist(lane[self.i + 1], (p.x, p.y)) <= math.dist(lane[self.i], (p.x, p.y)):
                self.i += 1
            j = self.i
            while j + 1 < len(lane) and math.dist(lane[j], (p.x, p.y)) < a.lookahead:
                j += 1
            tx, ty = lane[j]
            alpha = math.atan2(ty - p.y, tx - p.x) - yaw
            alpha = math.atan2(math.sin(alpha), math.cos(alpha))
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = 2.0 * v * math.sin(alpha) / max(a.lookahead, 1e-3)
            self.pub.publish(cmd)

    rclpy.init(args=sys.argv)
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main():
    import rclpy.utilities
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    sp = sub.add_parser('spawn')
    sp.add_argument('--s', type=float, required=True)
    sp.add_argument('--world', default=None)
    dp = sub.add_parser('drive')
    dp.add_argument('--profile', required=True, help='t0:v0,t1:v1,... (sim s since start : m/s)')
    dp.add_argument('--lookahead', type=float, default=5.0)
    dp.add_argument('--rate', type=float, default=20.0)
    dp.add_argument('--world', default=None)
    rp = sub.add_parser('remove')
    rp.add_argument('--world', default=None)
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    {'spawn': spawn, 'drive': drive, 'remove': remove}[a.cmd](a)


if __name__ == '__main__':
    main()
