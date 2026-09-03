#!/bin/bash
source /opt/ros/humble/setup.bash
cd ~/vlm_av_ws
source install/setup.bash
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

rclpy.init()
n = Node('probe')
got = {}
def mk(name):
    def cb(msg):
        if name in got: return
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = math.atan2(2*(o.w*o.z+o.x*o.y), 1-2*(o.y*o.y+o.z*o.z))
        t = msg.twist.twist
        got[name] = (msg.header.frame_id, msg.child_frame_id, p.x, p.y, p.z, yaw,
                     t.linear.x, t.linear.y, t.angular.z)
    return cb
for topic,name in [('/odom_truth','truth'),('/odom','wheel'),('/odom_ekf','ekf')]:
    n.create_subscription(Odometry, topic, mk(name), 10)

import time
end = time.time()+12
while time.time()<end and len(got)<3:
    rclpy.spin_once(n, timeout_sec=0.2)

for name in ('truth','wheel','ekf'):
    if name in got:
        f,c,x,y,z,yaw,vx,vy,wz = got[name]
        print(f"{name:6s} frame={f}/{c}  pos=({x:+.4f}, {y:+.4f}, {z:+.4f})  yaw={yaw:+.4f}  vel=({vx:+.4f},{vy:+.4f}) wz={wz:+.4f}")
    else:
        print(f"{name:6s} NO MESSAGE")
n.destroy_node()
rclpy.shutdown()
PY
