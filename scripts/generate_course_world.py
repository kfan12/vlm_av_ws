#!/usr/bin/env python3
"""generate_course_world.py — v2 course demo world (v1 path-demo recreated urban).

One two-way street (no intersections): straight -> right 90 -> straight ->
winding S (3 arcs) -> straight -> left 90 -> final straight with a stop line.
VLM warning signs (scaled-up v1 boards, Qwen-proven textures) stand on the
right shoulder before each feature; the AV reads them and adjusts speed
(v1 principle: direction lives in the path, signs only set speed).

Visual language matches generate_urban_world.py so lane_node's HSV gates work
unchanged: dark ground, 0.85-white solid edge lines at +-LW, 3 m / 4.5 m yellow
center dash on the road centerline, right-hand traffic (lane centers +-LW/2).
All road paint + asphalt ribbons are VISUALS OF ONE STATIC MODEL ("road") to
keep the model count low (grid world: 154 models -> 6.9 FPS, KB issue #5).

Usage:
  python3 scripts/generate_course_world.py --seed 42 --out urban_course
Outputs:
  src/urban_gazebo/worlds/<out>.world.sdf
  maps/<out>.json
"""
import argparse
import json
import math
import random
from pathlib import Path

WS = Path(__file__).resolve().parents[1]

# ---------------- course definition (centerline of the two-way road) --------
# ('straight', length_m) | ('arc', radius_m, dtheta_rad)  (+left / -right)
COURSE = [
    ("straight", 60.0),
    ("arc", 15.0, -math.pi / 2),      # right 90
    ("straight", 20.0),
    ("arc", 18.0, +math.pi / 4),      # winding S: left 45
    ("arc", 18.0, -math.pi / 2),      #            right 90
    ("arc", 18.0, +math.pi / 4),      #            left 45 (net straight again)
    ("straight", 15.0),
    ("arc", 15.0, +math.pi / 2),      # left 90
    ("straight", 45.0),
]
# sign kind -> arc-length where its FEATURE starts (filled in by walk_course)
SIGN_LEAD_M = 18.0                    # board this far before the feature start
STOP_LINE_FROM_END_M = 8.0            # stop line this far before the road ends


def walk_course(ds=0.5):
    """Sample the centerline: list of (s, x, y, heading). Also returns the
    arc-length at the START of each course segment (for sign placement)."""
    pts = []
    seg_starts = []
    x, y, h, s = 0.0, 0.0, 0.0, 0.0
    for seg in COURSE:
        seg_starts.append(s)
        if seg[0] == "straight":
            L = seg[1]
            n = max(1, int(round(L / ds)))
            for k in range(1, n + 1):
                d = L * k / n
                pts.append((s + d, x + d * math.cos(h), y + d * math.sin(h), h))
            x += L * math.cos(h)
            y += L * math.sin(h)
            s += L
        else:
            _, r, dth = seg
            L = abs(dth) * r
            n = max(4, int(round(L / ds)))
            side = 1.0 if dth > 0 else -1.0
            # arc center: r along the left normal (-sin h, cos h) for a left
            # turn, along the right normal for a right turn
            cx = x + side * r * -math.sin(h)
            cy = y + side * r * math.cos(h)
            a0 = math.atan2(y - cy, x - cx)
            for k in range(1, n + 1):
                a = a0 + dth * k / n
                pts.append((s + L * k / n,
                            cx + r * math.cos(a), cy + r * math.sin(a),
                            h + dth * k / n))
            x = cx + r * math.cos(a0 + dth)
            y = cy + r * math.sin(a0 + dth)
            h = h + dth
            s += L
    if not pts or pts[0][0] > 1e-9:
        pts.insert(0, (0.0, 0.0, 0.0, 0.0))
    return pts, seg_starts, s


def pose_at(pts, s_query):
    """Interpolated (x, y, heading) at arc length s_query."""
    if s_query <= pts[0][0]:
        return pts[0][1], pts[0][2], pts[0][3]
    for i in range(1, len(pts)):
        if pts[i][0] >= s_query:
            s0, x0, y0, h0 = pts[i - 1]
            s1, x1, y1, h1 = pts[i]
            t = (s_query - s0) / max(1e-9, s1 - s0)
            dh = math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))
            return x0 + t * (x1 - x0), y0 + t * (y1 - y0), h0 + t * dh
    return pts[-1][1], pts[-1][2], pts[-1][3]


def left_of(x, y, h, d):
    """Point offset d to the LEFT of heading h (negative d = right)."""
    return x - d * math.sin(h), y + d * math.cos(h)


def visual(name, x, y, z, sx, sy, sz, rgba, yaw=0.0):
    return f"""
      <visual name="{name}">
        <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
      </visual>"""


def box_model(name, x, y, z, sx, sy, sz, rgba, yaw=0.0):
    return f"""
    <model name="{name}"><static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name="l">
        <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>
        <visual name="v">
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
      </visual></link></model>"""


def ribbon(vis, tag, pts, lateral, width, z, thick, rgba,
           dash=None, max_run_m=8.0, bend_tol=0.045):
    """Emit boxes covering the polyline offset `lateral` (left+) from the
    centerline. Merges near-collinear runs into single boxes (short chords on
    arcs, one long box on straights). dash=(paint_m, gap_m) for dashed lines,
    pattern measured along the CENTERLINE arc-length."""
    n = 0
    run = []  # list of (s, x, y, h) offset points in the current run

    def flush():
        nonlocal n
        if len(run) < 2:
            run.clear()
            return
        x0, y0 = run[0][1], run[0][2]
        x1, y1 = run[-1][1], run[-1][2]
        L = math.hypot(x1 - x0, y1 - y0)
        if L < 0.05:
            run.clear()
            return
        yaw = math.atan2(y1 - y0, x1 - x0)
        n += 1
        vis.append(visual(f"{tag}_{n:04d}", (x0 + x1) / 2, (y0 + y1) / 2, z,
                          L + 0.06, width, thick, rgba, yaw))
        run.clear()

    for (s, x, y, h) in pts:
        if dash is not None:
            period = dash[0] + dash[1]
            if (s % period) >= dash[0]:
                if run:  # close the box at the paint-interval edge, not at the
                    px, py = left_of(x, y, h, lateral)  # last in-paint SAMPLE
                    run.append((s, px, py, h))          # (~ds short otherwise)
                flush()
                continue
        px, py = left_of(x, y, h, lateral)
        if run:
            dh = abs(math.atan2(math.sin(h - run[0][3]), math.cos(h - run[0][3])))
            seg = s - run[0][0]
            if dh > bend_tol or seg > max_run_m:
                seam = run[-1]  # carry the last point into the next box so
                flush()         # consecutive chords ABUT -- otherwise every
                run.append(seam)  # flush drops one ds segment (dashed look on curves)
        run.append((s, px, py, h))
    flush()
    return n


SIGN_BOARD_M = 1.8      # urban board face size (the mesh quad is 1x1 m)
SIGN_BOARD_Z = 2.2      # board center height
SIGN_POST_LEN = 1.6

def emit_sign_models():
    """Write sign_<kind>_urban model dirs into src/urban_gazebo/models/.
    From-empty deviation: each model references ITS OWN meshes/sign.obj
    (generated by scripts/generate_sign_textures.py), not the v1 meshes."""
    for kind in ("right", "winding", "left", "stop"):
        d = WS / "src/urban_gazebo/models" / f"sign_{kind}_urban"
        d.mkdir(parents=True, exist_ok=True)
        (d / "model.config").write_text(f"""<?xml version="1.0"?>
<model>
  <name>sign_{kind}_urban</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <description>Urban-scale ({SIGN_BOARD_M:.1f} m) VLM sign board ({kind}); self-generated mesh.</description>
</model>
""", encoding="utf-8")
        (d / "model.sdf").write_text(f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="sign_{kind}_urban">
    <static>true</static>
    <link name="link">
      <visual name="post">
        <pose>0 0 {SIGN_POST_LEN / 2:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>0.06</radius><length>{SIGN_POST_LEN:.3f}</length></cylinder></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material>
      </visual>
      <visual name="board_face">
        <pose>0 0 {SIGN_BOARD_Z:.3f} 0 0 0</pose>
        <geometry><mesh>
          <uri>model://sign_{kind}_urban/meshes/sign.obj</uri>
          <scale>{SIGN_BOARD_M:.2f} {SIGN_BOARD_M:.2f} {SIGN_BOARD_M:.2f}</scale>
        </mesh></geometry>
      </visual>
      <collision name="board_col">
        <pose>0 0 {SIGN_BOARD_Z:.3f} 0 0 0</pose>
        <geometry><box><size>0.06 {SIGN_BOARD_M:.3f} {SIGN_BOARD_M:.3f}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
""", encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lane-width", type=float, default=3.5)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default="urban_course")
    ap.add_argument("--buildings", type=int, default=20)
    args = ap.parse_args()

    LW = args.lane_width
    lane = LW / 2
    rng = random.Random(args.seed)

    pts, seg_starts, total_len = walk_course(ds=0.5)

    xs = [p[1] for p in pts]
    ys = [p[2] for p in pts]
    x0, x1 = min(xs) - 30, max(xs) + 30
    y0, y1 = min(ys) - 30, max(ys) + 30

    sdf = []
    road_vis = []

    # ---------------- ground --------------------------------------------------
    sdf.append(f"""
    <model name="ground"><static>true</static>
      <pose>{(x0+x1)/2:.1f} {(y0+y1)/2:.1f} 0 0 0 0</pose>
      <link name="l">
        <collision name="c"><geometry><plane><normal>0 0 1</normal>
          <size>{x1-x0:.0f} {y1-y0:.0f}</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal>
          <size>{x1-x0:.0f} {y1-y0:.0f}</size></plane></geometry>
          <material><ambient>0.16 0.16 0.17 1</ambient><diffuse>0.16 0.16 0.17 1</diffuse></material>
        </visual></link></model>""")

    # ---------------- road ribbons (single static model) ----------------------
    # asphalt slab (slightly lighter than ground, below the paint)
    ribbon(road_vis, "asfalt", pts, 0.0, 2 * LW + 0.6, 0.003, 0.006,
           "0.21 0.21 0.22 1", max_run_m=10.0, bend_tol=0.06)
    # solid white edge lines at +-LW
    ribbon(road_vis, "edge_l", pts, +LW, 0.15, 0.010, 0.010, "0.85 0.85 0.85 1")
    ribbon(road_vis, "edge_r", pts, -LW, 0.15, 0.010, 0.010, "0.85 0.85 0.85 1")
    # dashed yellow center line: 4.5 m paint / 3.0 m gap (paint-dominant so it
    # reads clearly as a centre line from the FPV; max_run_m must exceed the
    # paint length or each dash splits into two boxes on the straights)
    ribbon(road_vis, "dash_c", pts, 0.0, 0.15, 0.010, 0.010, "0.85 0.75 0.1 1",
           dash=(4.5, 3.0), max_run_m=5.0)

    # stop line across the OUTBOUND (right-hand) lane near the road end
    s_stop = total_len - STOP_LINE_FROM_END_M
    sx_, sy_, sh_ = pose_at(pts, s_stop)
    slx, sly = left_of(sx_, sy_, sh_, -lane)
    road_vis.append(visual("stop_line", slx, sly, 0.010, 0.4, LW, 0.010,
                           "0.85 0.85 0.85 1", sh_))

    sdf.append(f"""
    <model name="road"><static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="l">{''.join(road_vis)}
      </link></model>""")

    # ---------------- signs ---------------------------------------------------
    # feature starts: right turn = COURSE[1], winding = COURSE[3], left = COURSE[7]
    sign_defs = [
        ("right",   seg_starts[1] - SIGN_LEAD_M),
        ("winding", seg_starts[3] - SIGN_LEAD_M),
        ("left",    seg_starts[7] - SIGN_LEAD_M),
        ("stop",    s_stop),
    ]
    signs_json = []
    for kind, s in sign_defs:
        x, y, h = pose_at(pts, s)
        # right shoulder: 1.6 m outside the road edge, facing the approaching car
        bx, by = left_of(x, y, h, -(LW + 1.6))
        yaw = math.atan2(math.sin(h + math.pi), math.cos(h + math.pi))
        sdf.append(f"""
    <include>
      <name>sign_{kind}</name>
      <uri>model://sign_{kind}_urban</uri>
      <pose>{bx:.3f} {by:.3f} 0 0 0 {yaw:.4f}</pose>
    </include>""")
        signs_json.append({"kind": kind, "x": round(bx, 3), "y": round(by, 3),
                           "s": round(s, 2), "yaw": round(yaw, 4)})

    # ---------------- buildings ----------------------------------------------
    placed = []
    n_bldg = 0
    tries = 0
    while n_bldg < args.buildings and tries < 400:
        tries += 1
        s = rng.uniform(5.0, total_len - 5.0)
        side = rng.choice([-1.0, 1.0])
        w = rng.uniform(8, 14)      # along road
        dep = rng.uniform(8, 14)    # away from road
        hgt = rng.uniform(5, 14)
        setback = rng.uniform(6.0, 10.0)
        x, y, h = pose_at(pts, s)
        cx_, cy_ = left_of(x, y, h, side * (LW + setback + dep / 2))
        # reject if any part of the footprint could reach the road corridor:
        # conservative circle test against the centerline polyline
        rad = math.hypot(w, dep) / 2
        ok = True
        for (ps, px, py, ph) in pts[:: 4]:
            if math.hypot(px - cx_, py - cy_) < rad + LW + 1.5:
                ok = False
                break
        if ok:
            for (ox, oy, orad) in placed:
                if math.hypot(ox - cx_, oy - cy_) < rad + orad + 2.0:
                    ok = False
                    break
        if not ok:
            continue
        n_bldg += 1
        placed.append((cx_, cy_, rad))
        shade = rng.uniform(0.3, 0.6)
        tint = rng.choice([(1, 0.95, 0.9), (0.9, 0.95, 1), (1, 1, 0.95)])
        rgba = f"{shade*tint[0]:.2f} {shade*tint[1]:.2f} {shade*tint[2]:.2f} 1"
        sdf.append(box_model(f"bldg_{n_bldg:02d}", cx_, cy_, hgt / 2,
                             w, dep, hgt, rgba, yaw=h))

    # ---------------- map json ------------------------------------------------
    spawn_x, spawn_y, spawn_h = pose_at(pts, 3.0)
    spx, spy = left_of(spawn_x, spawn_y, spawn_h, -lane)   # right lane center
    mapj = {
        "seed": args.seed, "lane_width": LW, "speed_limit_default": 8.0,
        "spawn": {"x": round(spx, 3), "y": round(spy, 3),
                  "yaw": round(spawn_h, 4)},
        "intersections": [], "lights": [], "speed_zones": [],
        "crosswalks": [], "signs": signs_json,
        "stop_line": {"x": round(slx, 3), "y": round(sly, 3),
                      "heading": round(sh_, 4), "s": round(s_stop, 2)},
        "course": {"length_m": round(total_len, 2),
                   "centerline": [[round(p[1], 2), round(p[2], 2)]
                                  for p in pts[:: 4]]},
    }

    # ---------------- write ---------------------------------------------------
    world_name = args.out
    header = f"""<?xml version="1.0"?>
<sdf version="1.7">
  <world name="{world_name}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="libignition-gazebo-physics-system.so"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-user-commands-system.so"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="libignition-gazebo-sensors-system.so"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="libignition-gazebo-imu-system.so"
            name="ignition::gazebo::systems::Imu"/>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.55 0.7 0.9 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
"""
    footer = "\n  </world>\n</sdf>\n"

    emit_sign_models()
    world_path = WS / "src/urban_gazebo/worlds" / f"{world_name}.world.sdf"
    world_path.write_text(header + "".join(sdf) + footer, encoding="utf-8")
    map_path = WS / "maps" / f"{world_name}.json"
    map_path.write_text(json.dumps(mapj, indent=1), encoding="utf-8")

    n_models = sum(s.count("<model") for s in sdf) + 4  # + sign includes
    n_vis = sum(s.count("<visual") for s in sdf)
    print(f"wrote {world_path}\n  course {total_len:.1f} m, {n_models} models, "
          f"{n_vis} visuals, {n_bldg} buildings, stop line at s={s_stop:.1f}")
    print(f"wrote {map_path}\n  spawn {mapj['spawn']}  signs at "
          f"{[(sj['kind'], sj['s']) for sj in signs_json]}")


if __name__ == "__main__":
    main()