"""sign_maneuver_node — v2 sign→action pipeline (course demo).

The v2 port of v1's sign geometry + arbitration (vlm_node Phases 1–3+5),
decoupled from any path planning: the road curvature steers the car (lane_node
→ local_planner → MPC); signs only shape SPEED and add a stop point — the v2
advisory invariant (caps only, never raises) holds by construction.

Pipeline, all reused v1 logic where it was unit-tested:
  camera depth → elevated-blob board detection (z-band for full-size boards)
  → odom board latch + label lock + pixel-ROI hand-off to vlm_sign_node (Qwen)
  → SignLabelLatch (capture-time distance band, v1 module, unchanged)
  → ManeuverStateMachine (v1 module, unchanged; exit fed by /lane/path_odom)
  → the v1 wire contract, consumed directly by mpc_tracker_v2:
      /maneuver/target_speed (Float64, absolute setpoint: cruise_speed on
      straights, per-maneuver caps in turns/winding, creep→0 for stop)
      /maneuver/state (String, plain state name; left/right selects the
      MPC's turn heading lookahead)
  → /vlm/sign_maneuver JSON kept as a DIAGNOSTIC:
      {"stamp": t, "maneuver": "straight|left|right|winding|stop",
       "speed_cap": float|null, "stop_dist_m": float|null,
       "sign": {"label": ..., "dist": ..., "locked": ..., "armed": ...}}
The MPC takes min(profile, setpoint) — signs can only lower speed (advisory
invariant by construction). The stop sign halts purely through the speed
channel: creep at stop_creep_cap, then 0 inside stop_standoff_m (v1-style;
no planner stop-point plumbing).

Runs under the torch venv alongside vlm_sign_node (which does the actual Qwen
reads — this node never touches the GPU):

    source ~/venvs/vlm_robot/bin/activate
    python3 -m vlm_planner_py.sign_maneuver_node --ros-args \
        --params-file src/vlm_planner_py/config/sign_maneuver_params.yaml
    python3 -m vlm_planner_py.vlm_sign_node --ros-args -p vlm_query_rate_hz:=0.5

All tunables live in config/sign_maneuver_params.yaml (defaults match the code);
copy it per tuning cycle and point --params-file at the copy.
"""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
import cv2

from vlm_planner_py.sign_latch import SignLabelLatch, SIGN_LABELS
from vlm_planner_py.maneuver import ManeuverStateMachine, STRAIGHT, LEFT, RIGHT, WINDING, STOP
from vlm_planner_py.img_convert import img_msg_to_bgr, img_msg_to_depth, bgr_to_img_msg

def depth_msg_to_m(msg):
    """sensor_msgs/Image depth → float32 metres array (row-padding aware)."""
    return img_msg_to_depth(msg)

class SignManeuverNode(Node):
    def __init__(self):
        super().__init__('sign_maneuver')
        p = self.declare_parameter

        # --- topics
        self.image_topic = p('image_topic', '/camera/front/image_raw').value
        self.depth_topic = p('depth_topic', '/camera/front/depth/image_raw').value
        self.odom_topic = p('odom_topic', '/odom_ekf').value
        self.lane_topic = p('lane_topic', '/lane/path_odom').value
        self.sign_topic = p('sign_topic', '/vlm/sign').value           # Qwen labels in
        self.roi_topic = p('sign_roi_topic', '/vlm/sign_roi').value    # ROI out
        self.out_topic = p('out_topic', '/vlm/sign_maneuver').value
        self.speed_topic = p('target_speed_topic', '/maneuver/target_speed').value
        self.state_topic = p('maneuver_state_topic', '/maneuver/state').value

        # --- camera intrinsics/extrinsics (bridge camera_info is WRONG — F2;
        #     derive from params exactly like the C++ nodes do)
        self.img_w = int(p('img_w', 424).value)
        self.img_h = int(p('img_h', 300).value)
        self.hfov = float(p('hfov', 1.9).value)
        self.cam_x = float(p('cam_x', 2.2).value)
        self.cam_z = float(p('cam_z', 1.4).value)
        self.cam_pitch = float(p('cam_pitch', 0.06).value)

        # --- board detection (urban boards: 2.0 m face centered at z 2.2,
        #     spans z [1.2, 3.2]; see generate_course_world.py SIGN_BOARD_*)
        self.stride = int(p('px_stride', 4).value)
        self.z_min = float(p('sign_z_min_m', 1.0).value)
        self.z_max = float(p('sign_z_max_m', 3.4).value)
        self.x_min = float(p('sign_x_min_m', 3.0).value)
        self.x_max = float(p('sign_x_max_m', 35.0).value)
        self.y_min = float(p('sign_y_min_m', -9.0).value)   # right shoulder…
        self.y_max = float(p('sign_y_max_m', 2.0).value)    # …tolerant in curves
        self.min_pts = int(p('sign_min_pts', 12).value)     # stride-res blob size
        self.match_radius = float(p('board_match_radius_m', 2.5).value)
        self.board_lost_s = float(p('board_lost_s', 2.0).value)
        # after board_lost_s with no detection, COAST (freeze the odom latch +
        # identity) for this much longer before truly abandoning the board.
        # Bridges S-curve dropouts so the ~2 s Qwen read doesn't come back
        # 'stale-board'. board pass-behind (bx<0.5) still drops it immediately.
        self.board_lost_grace_s = float(p('board_lost_grace_s', 3.0).value)

        # --- colour gate: an elevated depth blob is only a sign board if its
        # RGB footprint is mostly red (stop octagon) or yellow (warning
        # diamond). Rejects buildings / poles / foliage that sit in the same
        # z/x/y box. HSV is OpenCV convention (H 0-180). Red wraps, so two hue
        # bands. Fraction is over the blob's bbox, so keep the floor lenient
        # (white border + black glyph + off-axis views dilute it).
        self.color_gate = bool(p('sign_color_gate', True).value)
        self.color_frac_min = float(p('sign_color_frac_min', 0.10).value)
        self.color_s_min = int(p('sign_color_s_min', 90).value)
        self.color_v_min = int(p('sign_color_v_min', 80).value)
        self.yellow_h = (int(p('sign_yellow_h_min', 18).value),
                         int(p('sign_yellow_h_max', 40).value))
        self.red_h_lo = int(p('sign_red_h_lo_max', 12).value)   # 0..this
        self.red_h_hi = int(p('sign_red_h_hi_min', 168).value)  # this..180

        # --- v1 sign geometry, retuned for the sedan world
        # Label band: a read is attributed to the frame's CAPTURE-time board
        # distance and accepted only inside this window. It must be wide enough
        # that several VLM inferences land in it on approach; at cruise_speed
        # 1.5 m/s and vlm_query_rate_hz 0.5 (~2 s Qwen latency) a 3 m window
        # caught ~1 read with no retry margin, so the first sign (approached at
        # full cruise) was routinely lost. 10 m window is ~6-7 reads. The floor
        # stays well above the ~2.5 m board-top crop distance.
        self.label_band = (float(p('sign_label_min_dist_m', 4.0).value),
                           float(p('sign_label_max_dist_m', 14.0).value))
        self.lock_dist = float(p('sign_label_lock_dist_m', 2.0).value)
        self.reach_dist = float(p('sign_reach_dist_m', 22.0).value)
        self.commit_dist_turn = float(p('commit_dist_turn_m', 8.0).value)
        self.commit_dist = {LEFT: self.commit_dist_turn,
                            RIGHT: self.commit_dist_turn,
                            WINDING: float(p('commit_dist_winding_m', 12.0).value),
                            STOP: float(p('commit_dist_stop_m', 10.0).value)}

        # --- speeds (MPC takes min(profile, setpoint) — caps never raise)
        self.cruise_speed = float(p('cruise_speed', 1.5).value)
        self.turn_cap = float(p('turn_speed_cap', 1.0).value)
        self.winding_cap = float(p('winding_speed_cap', 1.0).value)
        self.stop_creep_cap = float(p('stop_creep_cap', 0.5).value)
        # Gap we want between the FRONT BUMPER and the stop line at rest. The
        # frozen stop point is the sign's along-track position (== the map's
        # stop_line x on this course); base_link is the wheelbase midpoint, so
        # the bumper is front_overhang_m ahead of it and must be subtracted too
        # or the car noses ~0.75 m past the line.
        self.stop_standoff = float(p('stop_standoff_m', 1.0).value)
        self.front_overhang = float(p('veh_front_overhang_m', 2.25).value)

        # release after 3 m of continuously-straight lane centerline
        self.fsm = ManeuverStateMachine(
            engage_tol_rad=float(p('engage_tol_rad', 0.35).value),
            straight_tol_rad=float(p('straight_tol_rad', 0.20).value),
            release_dist_m=float(p('release_dist_m', 3.0).value),
            min_len_m=float(p('exit_min_len_m', 4.0).value))

        # hold the car (cap 0) until the FIRST Qwen label arrives — Qwen takes
        # ~1 min to load and the first sign is only ~40 m out (v1's
        # wait_for_first_sign, moved to the cap channel)
        self.wait_for_vlm = bool(p('wait_for_vlm', True).value)

        # /maneuver/debug_image: the raw camera frame with the board boxes, the
        # Qwen ROI and the live FSM/label/speed readout drawn on it (RViz Image
        # display, same role as lane_node's /lane/debug_image).
        self.debug_image = bool(p('debug_image', True).value)

        # intrinsics (square pixels, fx from hfov — pinhole_from_config twin)
        self.fx = (self.img_w / 2.0) / math.tan(self.hfov / 2.0)
        self.cx = self.img_w / 2.0
        self.cy = self.img_h / 2.0

        self.latch = SignLabelLatch(min_dist_m=self.label_band[0],
                                    max_dist_m=self.label_band[1])

        self.board_odom = None      # np.array([x, y]) latched board in odom
        self.board_seen_t = -1e9    # last tick the board was re-observed
        self.board_committed = False  # this board's maneuver already committed
        self.stop_point_odom = None   # frozen odom point of the STOP board
        self.vlm_alive = False
        self.pose = None            # (x, y, yaw)
        self.last_pose_xy = None
        self.lane_pts = None
        self.rgb = None
        self.depth = None
        self.last_boards = []        # last _detect_boards() output, for the overlay
        self.color_rejects = []      # blobs the colour gate dropped, for the overlay
        self.rgb_bgr = None          # last decoded camera frame (detect + overlay)
        self.roi_px = None           # last ROI box handed to Qwen (x0, y0, x1, y1)
        self.vlm_last = None         # newest /vlm/sign read: dict(label, accepted,
                                     # why, t_recv) — raw Qwen action, pre-latch

        self.pub_out = self.create_publisher(String, self.out_topic, 10)
        self.pub_roi = self.create_publisher(String, self.roi_topic, 10)
        self.pub_speed = self.create_publisher(Float64, self.speed_topic, 10)
        self.pub_state = self.create_publisher(String, self.state_topic, 10)
        self.pub_dbg_img = (self.create_publisher(Image, '/maneuver/debug_image', 1)
                            if self.debug_image else None)
        self.create_subscription(Image, self.image_topic, self._rgb_cb, 5)
        self.create_subscription(Image, self.depth_topic, self._depth_cb, 5)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self.create_subscription(Path, self.lane_topic, self._lane_cb, 5)
        self.create_subscription(String, self.sign_topic, self._label_cb, 10)
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'sign_maneuver up: boards z[{self.z_min},{self.z_max}] '
            f'label band {self.label_band} lock {self.lock_dist} '
            f'commit {self.commit_dist} caps turn={self.turn_cap} '
            f'winding={self.winding_cap} stop_creep={self.stop_creep_cap}')

    # ------------------------------------------------------------- callbacks
    def _rgb_cb(self, m):
        self.rgb = m

    def _depth_cb(self, m):
        self.depth = m

    def _lane_cb(self, m):
        self.lane_pts = [(ps.pose.position.x, ps.pose.position.y) for ps in m.poses]

    def _odom_cb(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def _label_cb(self, m):
        try:
            d = json.loads(m.data)
            label, t_cap = d.get('sign'), d.get('stamp')
        except (ValueError, TypeError):
            label, t_cap = m.data.strip(), None   # legacy plain-string label
        self.vlm_alive = True
        acc, why = self.latch.on_label(label, t_cap)
        self.vlm_last = {'label': label, 'accepted': acc, 'why': why,
                         't_recv': self.get_clock().now().nanoseconds * 1e-9}
        if label in SIGN_LABELS:
            self.get_logger().info(f'label {label!r}: {"ACCEPTED" if acc else "rejected"} ({why})')


    # ------------------------------------------------------------- geometry
    def _base_to_odom(self, bx, by):
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([x + c * bx - s * by, y + s * bx + c * by])

    def _odom_to_base(self, pt):
        x, y, yaw = self.pose
        dx, dy = pt[0] - x, pt[1] - y
        c, s = math.cos(-yaw), math.sin(-yaw)
        return (c * dx - s * dy, s * dx + c * dy)

    def _sign_colored_frac(self, bgr, sel_stride):
        """(frac, mean_bgr, mean_hsv) for the blob's ACTUAL RGB footprint.

        frac = fraction of the connected-component pixels (upsampled to full
        res, dilated one stride cell to absorb minor RGB/depth misalignment)
        that are sign-coloured (red octagon / yellow diamond). The two means
        are for diagnosing a near-zero frac: blue-ish -> channel swap; dark ->
        raise v_min / lower it; grey -> the footprint is off the sign."""
        st = self.stride
        H, W = bgr.shape[:2]
        m = np.repeat(np.repeat(sel_stride, st, 0), st, 1)[:H, :W].astype(np.uint8)
        m = cv2.dilate(m, np.ones((2 * st + 1, 2 * st + 1), np.uint8))
        px = bgr[m.astype(bool)]
        if px.size == 0:
            return 0.0, (0, 0, 0), (0, 0, 0)
        hsv = cv2.cvtColor(px.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)
        hue, sat, val = hsv[:, 0], hsv[:, 1], hsv[:, 2]
        chroma = (sat >= self.color_s_min) & (val >= self.color_v_min)
        yellow = (hue >= self.yellow_h[0]) & (hue <= self.yellow_h[1])
        red = (hue <= self.red_h_lo) | (hue >= self.red_h_hi)
        frac = float((chroma & (yellow | red)).mean())
        mean_bgr = tuple(int(v) for v in px.mean(axis=0))
        mean_hsv = tuple(int(v) for v in hsv.mean(axis=0))
        return frac, mean_bgr, mean_hsv

    def _detect_boards(self, depth_m, rgb_bgr=None):
        """Elevated blobs in the depth image → list of (bx, by, bbox_px, npts),
        bbox in FULL-RES pixel coords. Vectorized at stride resolution. When
        rgb_bgr is given and the colour gate is on, blobs whose RGB footprint
        is not mostly red/yellow are dropped (buildings, poles, foliage)."""
        st = self.stride
        d = depth_m[::st, ::st]
        h, w = d.shape
        us = (np.arange(w) * st - self.cx) / self.fx
        vs = (np.arange(h) * st - self.cy) / self.fx
        U, V = np.meshgrid(us, vs)
        valid = np.isfinite(d) & (d > 0.3) & (d < self.x_max + 10.0)
        Z = np.where(valid, d, 0.0)
        # optical → camera-aligned base axes (fwd, left, up), then pitch-down p
        xf, yf, zf = Z, -U * Z, -V * Z
        cp, sp = math.cos(self.cam_pitch), math.sin(self.cam_pitch)
        xb = self.cam_x + cp * xf + sp * zf
        zb = self.cam_z - sp * xf + cp * zf
        yb = yf
        mask = (valid & (zb > self.z_min) & (zb < self.z_max) &
                (xb > self.x_min) & (xb < self.x_max) &
                (yb > self.y_min) & (yb < self.y_max))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(
            mask.astype(np.uint8), connectivity=8)
        boards = []
        self.color_rejects = []      # (bbox, frac) blobs the colour gate dropped
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] < self.min_pts:
                continue
            sel = lab == i
            bx = float(np.median(xb[sel]))
            by = float(np.median(yb[sel]))
            x0 = int(stats[i, cv2.CC_STAT_LEFT]) * st
            y0 = int(stats[i, cv2.CC_STAT_TOP]) * st
            x1 = x0 + int(stats[i, cv2.CC_STAT_WIDTH]) * st
            y1 = y0 + int(stats[i, cv2.CC_STAT_HEIGHT]) * st
            bbox = (x0, y0, x1, y1)
            if self.color_gate and rgb_bgr is not None:
                frac, mbgr, mhsv = self._sign_colored_frac(rgb_bgr, sel)
                if frac < self.color_frac_min:
                    self.color_rejects.append((bbox, frac, mbgr, mhsv))
                    self.get_logger().warn(
                        f'colour-gate drop @x~{bx:.0f}m: frac={frac:.2f} '
                        f'meanBGR={mbgr} meanHSV={mhsv}',
                        throttle_duration_sec=2.0)
                    continue
            boards.append((bx, by, bbox, int(stats[i, cv2.CC_STAT_AREA])))
        return boards

    # ------------------------------------------------------------- main tick
    def _tick(self):
        if self.pose is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9

        # travelled distance since last tick (feeds the FSM's metric debounce)
        step = 0.0
        if self.last_pose_xy is not None:
            step = math.hypot(self.pose[0] - self.last_pose_xy[0],
                              self.pose[1] - self.last_pose_xy[1])
        self.last_pose_xy = (self.pose[0], self.pose[1])

        frame_t = None
        dist = None
        if self.depth is not None and self.rgb is not None:
            frame_t = (self.depth.header.stamp.sec +
                       self.depth.header.stamp.nanosec * 1e-9)
            try:
                self.rgb_bgr = img_msg_to_bgr(self.rgb)
                boards = self._detect_boards(depth_msg_to_m(self.depth),
                                             self.rgb_bgr)
            except (ValueError, TypeError) as e:
                self.get_logger().warn(f'depth/rgb decode/detect failed: {e}',
                                       throttle_duration_sec=5.0)
                boards = []
            self.last_boards = boards
            self._update_board(boards, now, frame_t)

        # current along-track distance of the latched board
        if self.board_odom is not None:
            bx, by = self._odom_to_base(self.board_odom)
            dist = bx
            # board passed behind → this board is done; free the latch for the
            # next one (v1 clear_board semantics)
            if bx < 0.5:
                self.latch.clear_board()
                self.board_odom = None
                self.board_committed = False
                dist = None

        # record the (frame, dist) pair for capture-time label attribution
        if frame_t is not None:
            self.latch.record(frame_t, dist)
        if dist is not None and dist < self.lock_dist:
            self.latch.lock()

        # ---- maneuver entry: arm within reach, commit level-triggered
        pending = 'none'
        commit = False
        if dist is not None and not self.board_committed:
            label = self.latch.label
            if label is not None and 0.0 < dist < self.reach_dist:
                pending = label
            armed = pending if pending in self.commit_dist else \
                (self.fsm._armed if self.fsm._armed in self.commit_dist else None)
            if armed is not None and 0.0 < dist < self.commit_dist[armed]:
                commit = True
        if self.fsm.on_sign(pending, commit):
            self.board_committed = True
            if self.fsm.state == STOP:
                self.stop_point_odom = self.board_odom.copy()
            self.get_logger().info(
                f'maneuver COMMITTED: {self.fsm.state} (board at {dist and round(dist, 2)} m)')

        # ---- maneuver exit: lane-path straightness over travelled distance
        if self.lane_pts and len(self.lane_pts) >= 3:
            self.fsm.on_path(self.lane_pts, step)

        # ---- outputs
        cap = None
        stop_dist = None
        st = self.fsm.state
        if st in (LEFT, RIGHT):
            cap = self.turn_cap
        elif st == WINDING:
            cap = self.winding_cap
        elif st == STOP:
            cap = self.stop_creep_cap
            if self.stop_point_odom is not None:
                along = self._odom_to_base(self.stop_point_odom)[0]
                stop_dist = max(0.0, along - self.stop_standoff - self.front_overhang)
        if self.wait_for_vlm and not self.vlm_alive:
            cap = 0.0   # hold at spawn until Qwen produced its first read

        # ---- v1 wire contract into the MPC: absolute setpoint + state name.
        # stop: creep cap while the frozen stop point is beyond the standoff,
        # hard 0 once inside it (stop_dist hits 0.0 exactly there).
        speed = self.cruise_speed if cap is None else cap
        if st == STOP and stop_dist is not None and stop_dist <= 0.0:
            speed = 0.0
        self.pub_speed.publish(Float64(data=float(speed)))
        self.pub_state.publish(String(data=st))

        out = {'stamp': now, 'maneuver': st,
               'speed_cap': cap, 'stop_dist_m': stop_dist,
               'sign': {'label': self.latch.label,
                        'dist': None if dist is None else round(dist, 2),
                        'locked': self.latch.locked,
                        'armed': self.fsm._armed,
                        'vlm_alive': self.vlm_alive}}
        self.pub_out.publish(String(data=json.dumps(out)))

        if self.pub_dbg_img is not None and self.rgb is not None:
            self._publish_debug_image(out, speed)

    def _publish_debug_image(self, out, speed):
        """Raw camera frame + board boxes + Qwen ROI + FSM/label/speed readout."""
        if self.rgb_bgr is not None:
            frame = self.rgb_bgr.copy()
        else:
            try:
                frame = img_msg_to_bgr(self.rgb)
            except (ValueError, TypeError) as e:
                self.get_logger().warn(f'debug image decode failed: {e}',
                                       throttle_duration_sec=5.0)
                return

        # every candidate blob this frame, thin grey
        for (_bx, _by, (x0, y0, x1, y1), _n) in self.last_boards:
            cv2.rectangle(frame, (x0, y0), (x1, y1), (150, 150, 150), 1)

        # blobs the colour gate rejected (not red/yellow): magenta box, the
        # frac, and the footprint's mean HSV (for diagnosing a ~0 frac)
        for ((x0, y0, x1, y1), frac, _mbgr, mhsv) in self.color_rejects:
            cv2.rectangle(frame, (x0, y0), (x1, y1), (200, 0, 200), 1)
            cv2.putText(frame, '{:.2f} hsv{}'.format(frac, mhsv), (x0, max(9, y0 - 2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 0, 200), 1, cv2.LINE_AA)

        # the tracked board: green once its maneuver is committed, amber while
        # only armed, red otherwise
        if self.board_odom is not None and self.last_boards:
            trk = min(self.last_boards,
                      key=lambda b: np.linalg.norm(
                          self._base_to_odom(b[0], b[1]) - self.board_odom))
            x0, y0, x1, y1 = trk[2]
            col = ((0, 200, 0) if self.board_committed
                   else (0, 190, 255) if self.fsm._armed else (0, 0, 230))
            cv2.rectangle(frame, (x0, y0), (x1, y1), col, 2)

        # the exact crop handed to Qwen, cyan (matches vlm_sign_node's overlay)
        if self.roi_px is not None:
            rx0, ry0, rx1, ry1 = self.roi_px
            cv2.rectangle(frame, (rx0, ry0), (rx1, ry1), (255, 255, 0), 1)

        s = out['sign']
        cap = out['speed_cap']
        cap_txt = '-' if cap is None else '{:.2f}'.format(cap)
        dist_txt = '' if s['dist'] is None else '  {:.1f} m'.format(s['dist'])
        lock_txt = '  LOCKED' if s['locked'] else ''
        WHITE, GREEN, AMBER = (255, 255, 255), (0, 230, 0), (0, 190, 255)

        # live Qwen action straight off /vlm/sign, before the latch decides —
        # green if the latch took it, amber if it bounced it (with the reason),
        # dimmed once it goes stale.
        v = self.vlm_last
        if v is None:
            vlm_line, vlm_col = 'vlm:   waiting for first read', AMBER
        else:
            age = out['stamp'] - v['t_recv']
            verdict = 'ACCEPT' if v['accepted'] else 'rej: ' + v['why']
            vlm_line = 'vlm:   {}  [{}]  {:.1f}s'.format(v['label'], verdict, age)
            vlm_col = GREEN if v['accepted'] else AMBER
            if age > 5.0:
                vlm_col = tuple(c // 2 for c in vlm_col)

        lines = [
            ('state: {}'.format(out['maneuver']), WHITE),
            ('v_cmd: {:.2f}  cap: {}'.format(speed, cap_txt), WHITE),
            ('latch: {}{}{}'.format(s['label'] or '-', dist_txt, lock_txt), WHITE),
            ('armed: {}'.format(s['armed'] or '-'), WHITE),
            (vlm_line, vlm_col),
        ]
        if out['stop_dist_m'] is not None:
            lines.append(('stop:  {:.1f} m'.format(out['stop_dist_m']), WHITE))
        y = 16
        for ln, col in lines:
            cv2.putText(frame, ln, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, ln, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        col, 1, cv2.LINE_AA)
            y += 16

        self.pub_dbg_img.publish(bgr_to_img_msg(frame, header=self.rgb.header))

    def _update_board(self, boards, now, frame_t):
        """Track ONE board: nearest detection ahead; identity sticky within
        match_radius; a locked label freezes the identity (v1 label-lock)."""
        if not boards:
            if self.board_odom is not None:
                gone = now - self.board_seen_t
                ahead = self._odom_to_base(self.board_odom)[0]
                # inside lock range: ride the frozen latch, never drop.
                # short gap (board_lost_s..+grace): COAST — keep board_odom and
                # the latch identity frozen so a re-acquisition within
                # match_radius resumes the SAME board and any in-flight Qwen
                # read stays valid (no stale-board churn through the S-curve).
                # only past the grace window is it a real loss.
                if (ahead > self.lock_dist and
                        gone > self.board_lost_s + self.board_lost_grace_s):
                    self.latch.clear_board()
                    self.board_odom = None
                    self.board_committed = False
            return
        cands = [(bx, by, bbox, n) for (bx, by, bbox, n) in boards if bx > 0.5]
        if not cands:
            return
        best = None
        if self.board_odom is not None:
            # re-observe the SAME board
            for c in cands:
                pos = self._base_to_odom(c[0], c[1])
                if np.linalg.norm(pos - self.board_odom) < self.match_radius:
                    if best is None or c[0] < best[0]:
                        best = c
            if best is None:
                if self.latch.locked or \
                        self._odom_to_base(self.board_odom)[0] < self.lock_dist:
                    return          # identity frozen; ignore other blobs
                # not locked: allow a NEARER board to steal (v1 behavior)
                nearest = min(cands, key=lambda c: c[0])
                if nearest[0] < self._odom_to_base(self.board_odom)[0] - 1.0:
                    self.latch.clear_board()
                    self.board_committed = False
                    best = nearest
                else:
                    return
        else:
            best = min(cands, key=lambda c: c[0])
        pos = self._base_to_odom(best[0], best[1])
        if self.board_odom is None:
            self.board_odom = pos
        else:
            self.board_odom = 0.7 * self.board_odom + 0.3 * pos   # EMA smooth
        self.board_seen_t = now
        # ROI hand-off (padded bbox, stamped with the FRAME time)
        x0, y0, x1, y1 = best[2]
        pw, ph = int(0.2 * (x1 - x0)) + self.stride, int(0.2 * (y1 - y0)) + self.stride
        rx0, ry0 = max(0, x0 - pw), max(0, y0 - ph)
        rx1, ry1 = min(self.img_w, x1 + pw), min(self.img_h, y1 + ph)
        self.roi_px = (rx0, ry0, rx1, ry1)
        self.pub_roi.publish(String(data=json.dumps(
            {'x0': rx0, 'y0': ry0, 'x1': rx1, 'y1': ry1, 'stamp': frame_t})))


def main(args=None):
    rclpy.init(args=args)
    node = SignManeuverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

