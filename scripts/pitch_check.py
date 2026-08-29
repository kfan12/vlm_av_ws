import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

CAM_Z = 1.4      # cam_z from sedan.urdf.xacro:35
FX, FY, CX, CY = 310.7886722081518, 310.7886722081518, 320.0, 180.0

def solve_pitch(cam_z, depth, k):
    C = cam_z / depth
    R = math.hypot(1.0, k)
    phi = math.atan2(k, 1.0)
    arg = max(-1.0, min(1.0, C / R))
    return math.asin(arg) - phi

class Check(Node):
    def __init__(self):
        super().__init__('pitch_check')
        self.create_subscription(Image, '/camera/front/depth/image_raw', self.cb, 10)

    def cb(self, msg):
        print(f"encoding={msg.encoding} step={msg.step} w={msg.width} h={msg.height} "
              f"expected_step={msg.width * 4}")
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        for fu in (0.35, 0.5, 0.65):
            for fv in (0.92, 0.80, 0.68):
                u, v = int(msg.width * fu), int(msg.height * fv)
                d = float(depth[v, u])
                if not math.isfinite(d) or d < 0.5 or d > 30.0:
                    print(f"u={u:4d} v={v:4d}  d={d!r} (rejected)")
                    continue
                k = (v - CY) / FY
                j = (u - CX) / FX
                p_zdepth = solve_pitch(CAM_Z, d, k)
                z_from_range = d / math.sqrt(1.0 + j * j + k * k)
                p_range = solve_pitch(CAM_Z, z_from_range, k)
                print(f"u={u:4d} v={v:4d}  d={d:8.4f}  "
                      f"p_if_Zdepth={p_zdepth:+.5f}  p_if_range={p_range:+.5f}")
        rclpy.shutdown()

rclpy.init()
rclpy.spin(Check())