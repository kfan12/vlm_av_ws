# vlm_av_ws

A ROS2 + Gazebo autonomous-driving simulation stack: vision-based lane perception, a curvature-aware local planner, and an OSQP QP model-predictive controller, running a full-size sedan through a generated urban road course.
A second pipeline layers a vision-language model on top: it reads road signs (stop / left / right / winding) from the front camera and turns them into speed caps and a stop, on top of the vision-only lane following.
A third, independent layer adds safety: an automatic emergency braking (AEB) node that watches for obstacles in the car's predicted path, and a control arbiter that decides which command actually reaches the car.
AEB reads either the depth camera or an optional front lidar; everything after the obstacle points (corridor, braking model, state machine, arbiter) is shared, so the two are interchangeable.
The VLM pipeline and the AEB layer run in separate simulation setups, never together: `vlm`, `aeb` (depth AEB) and `lidar` (lidar AEB) (see [Running it](#running-it)).

This project is a continuation of [kfan12/vlm_robot_public](https://github.com/kfan12/vlm_robot_public).
It carries forward the same ROS2/Gazebo simulation approach, now built around a full-size sedan model (Ackermann steering, RGB-D front camera) instead of the original robot, and a more realistic generated road world with lane markings, curves, intersections, and street furniture.

**Status: under active development.**
The core stack runs end to end (perception -> planning -> control -> sim), and the VLM sign-reading pipeline (Qwen2.5-VL-3B) runs alongside it and drives the demo course (straight -> right 90 -> winding S -> left 90 -> STOP).
Tuning and new features are ongoing; expect frequent changes to camera/perception parameters, MPC weights, VLM sign-arbitration tuning, and the road-world generator as the project evolves.

## Screenshots

| Gazebo simulation | RViz (lane debug image + planned path) |
| --- | --- |
| ![Gazebo](docs/snapshots/gazebo.png) | ![RViz](docs/snapshots/rviz.png) |

## Stack overview

| Package | Role |
| --- | --- |
| `sedan_description` | V2 full-size sedan URDF: 2.7 m wheelbase, Ackermann steering, windshield-height RGB-D camera, and an optional front-grille lidar (xacro arg `lidar`, off by default). |
| `urban_gazebo` | Generated urban world/course tooling: road geometry, lane paint, signage, traffic-light control. |
| `robot_bringup` | Top-level launch files, `ros_gz` bridge configs (the lidar cloud on a bridge node of its own), RViz config with one filtered copy per setup. |
| `av_perception_cpp` | `lane_node`: HSV-based lane-paint detection, chain clustering/merging, ground-plane projection, debug image/markers. |
| `av_behavior_cpp` | `local_planner` (curve splicing + speed profiling) and `mpc_tracker_v2` (OSQP QP tracker on a linearized bicycle model). |
| `robotcar_localization` | EKF configuration (`robot_localization`) fusing wheel odometry and IMU. |
| `av_common_cpp` | Shared C++ library: geometry/polyline math, projection, point-cloud filtering and obstacle clustering, JSON map I/O, debug tap. |
| `vlm_planner_py` | VLM sign pipeline: `vlm_sign_node` (Qwen2.5-VL-3B sign reading) + `sign_maneuver_node` (board detection, sign-to-speed arbitration). Runs under a separate Python venv (torch); see below. |
| `av_safety_cpp` | `aeb_node`: obstacle clusters from the depth camera or the lidar (`obstacle_source`), swept collision corridor along the predicted path, TTC / safe braking distance, six-state AEB state machine. Also `obstacle_probe`, a measurement node that runs both obstacle extractors side by side. |
| `av_control_cpp` | `control_arbiter_node`: the single command authority for `/cmd_vel` when AEB is on (e-stop > AEB > fault stop > MPC). |
| `av_feature_msgs` | Typed feature interfaces (`AebStatus.msg`). |

Perception, planning, and control run as separate ROS2 nodes at their own rates, tied together through `av_stack.launch.py`, which also fans out the map's spawn pose so every node agrees on the same odom-frame origin.
The VLM pipeline runs as two extra nodes launched separately (not part of `av_stack.launch.py`, since it needs the torch venv) and only ever *caps* speed or adds a stop - it never steers, so the stack still drives lane-only with it off.

```
camera (RGB + depth) --> lane_node --> local_planner --> mpc_tracker_v2
                                                              |
             vlm setup (aeb:=false):                          +--> /cmd_vel
             aeb / lidar setup (aeb:=true):                   +--> /control/mpc_cmd --> control_arbiter --> /cmd_vel
                                                                   aeb_node --> /safety/aeb_status, /control/aeb_cmd --^
                                                                      ^
             depth image (aeb setup) or /lidar/front/points (lidar setup)
```

## VLM sign pipeline

Two nodes, both under `vlm_planner_py`, run in the torch venv alongside (not inside) `av_stack.launch.py`:

- **`vlm_sign_node`** - subscribes to the front camera, classifies the nearest/latched road sign with Qwen2.5-VL-3B-Instruct (4-bit, on its own ~0.5 Hz timer since inference takes ~2 s) and publishes `/vlm/sign` (`{"sign": <label>, "stamp": <capture time>}`, label in `{right,left,winding,stop,none,unparsed}`) plus an annotated `/vlm/sign_image`.
  It loads the model at startup, not on first frame, because the cold load (~2-3 min) pegs the CPU and would otherwise starve the Gazebo camera render and trip the MPC's path-stale safety.
- **`sign_maneuver_node`** - detects candidate sign boards from the depth image (elevated blob + color gate), latches one board by odom position, hands its pixel ROI to `vlm_sign_node` so Qwen reads the latched board instead of the nearest board-shaped thing, and runs the v1 `SignLabelLatch` / `ManeuverStateMachine` arbitration on the result.
  It publishes `/maneuver/target_speed` (absolute setpoint) and `/maneuver/state` (state name); `mpc_tracker_v2` takes `min(planner speed profile, this setpoint)`, so a sign can only lower speed, never steer or raise it.
  A `left`/`right` state also selects the MPC's turn-mode heading lookahead.
  `/vlm/sign_maneuver` publishes the full diagnostic JSON (maneuver, speed cap, stop distance, sign latch state) for the demo's narrative pane.

Tunables (camera geometry, board-detection z/x/y bands, color gate, label band, cruise/turn/winding speed caps, stop creep + standoff) live in `src/vlm_planner_py/config/sign_maneuver_params.yaml`; copy it per tuning cycle rather than editing code.

Requires a Python venv with `torch`, `transformers`, and (for the 4-bit GPU path) `bitsandbytes` - default location `~/venvs/vlm_robot` (override with `VLM_VENV`).
Without a CUDA GPU, Qwen falls back to full-precision CPU inference, which is slow enough to be impractical for the live demo.

Run it manually (two extra terminals, after `av_stack.launch.py` is up):

```bash
source ~/venvs/vlm_robot/bin/activate
python3 -m vlm_planner_py.vlm_sign_node --ros-args -p use_sim_time:=true -p vlm_query_rate_hz:=0.5
# wait for the "VLM ready" log line, then:
python3 -m vlm_planner_py.sign_maneuver_node --ros-args \
    --params-file src/vlm_planner_py/config/sign_maneuver_params.yaml -p use_sim_time:=true
```

Toggle Qwen querying live, without restarting: `ros2 param set /vlm_sign vlm_query_enabled false|true`.

## AEB safety layer

Two C++ nodes, started by `av_stack.launch.py` when `aeb:=true` (the default):

- **`aeb_node`** (`av_safety_cpp`) - an independent safety monitor at 20 Hz.
  It clusters obstacle points from the depth image or the lidar cloud (ground removed, height and range gated; see [Lidar-based AEB](#lidar-based-aeb)), sweeps the car's footprint along the predicted path (the MPC's path, extended by a kinematic prediction so a stopped car still sees what is in front of it), and computes time-to-collision and safe braking distance for the nearest obstacle in that corridor.
  A six-state machine (`INACTIVE`, `MONITORING`, `WARNING`, `BRAKING`, `STOP_HOLD`, `FAULT`) turns that into `/safety/aeb_status` (`av_feature_msgs/AebStatus`: state, emergency flag, distance, TTC, requested deceleration, reason) and an emergency command on `/control/aeb_cmd`.
  A stale obstacle sensor (depth or lidar) or stale odometry puts it in `FAULT`, and a fault never releases an active brake or hold.
  For RViz it publishes `/safety/aeb/debug_image`: the front camera with the swept corridor of both predicted paths, the `d_safe` braking line, obstacle clusters (orange inside the corridor, red with its distance for the nearest one), and a state-coloured banner with distance / TTC / `d_safe`, prefixed `AEB lidar` when the lidar feeds it (param `debug_image`, view only - the safety logic never reads the RGB frame).
- **`control_arbiter`** (`av_control_cpp`) - the only node allowed to publish `/cmd_vel` in this setup.
  Each tick it picks one source by priority: manual e-stop (`/safety/manual_estop`) > AEB emergency (`/control/aeb_cmd`) > stop if the AEB status is missing or in `FAULT` > the MPC's nominal command (`/control/mpc_cmd`) > stop.
  So no AEB means no driving, never "drive on unprotected".
  It reports its choice on `/control/active_source`, shown in RViz as a `ctrl: <source>` label above the car.

Tunables live in `src/av_safety_cpp/config/aeb_params.yaml` (camera and lidar geometry, obstacle gating, corridor margins, TTC thresholds, braking model, staleness timeouts) and `src/av_control_cpp/config/control_arbiter_params.yaml` (timeouts, speed / yaw-rate clamp); pass a copy with `aeb_config:=` / `arbiter_config:=`.
Purpose-built test worlds put a barrier on the course: `urban_course_roadblock` (barrier in the ego lane 32 m after spawn) and `urban_course_roadblock_oncoming` (barrier in the oncoming lane, which must *not* trigger braking).

### AEB in operation: the roadblock scenario

On `urban_course_roadblock` a barrier blocks the ego lane ahead of the car.
Nothing in the lane-following chain knows about it: the planner and the MPC keep asking for the fixed target speed, so stopping is entirely up to `aeb_node` and the arbiter.

![Gazebo: the sedan approaching the barrier in its lane](docs/snapshots/gazebo_roadbllock.png)

How to read the AEB debug image (`/safety/aeb/debug_image` in RViz):

- **Banner:** the state name in its colour, the ego speed `v`, the distance `dist` to the nearest obstacle in the corridor, `ttc`, `d_safe`, which predicted paths are in use (`[mpc+kin]`, or `[kin]` when the MPC path is stale) and the number of obstacle clusters (`cl`).
- **Corridor:** the car's footprint (plus a 0.3 m lateral margin) swept along the predicted paths and filled in the state colour; the cyan outline follows the MPC's path, the white one the kinematic prediction.
- **Yellow `d_safe` line:** the safe braking distance at the current speed, `d_safe = v * t_response + v^2 / (2 * a_brake) + offset` (0.3 s, 6 m/s^2 and 1.0 m by default).
- **Cluster stalks:** green outside the corridor, orange inside it, and red with a distance label for the nearest one.
- **Distance:** measured from the front bumper plus a 0.5 m longitudinal margin to the cluster's leading edge, so it reads 0.5 m shorter than the true gap (conservative on purpose).

| MONITORING | WARNING |
| --- | --- |
| ![AEB monitoring](docs/snapshots/AEB_Monitoring.png) | ![AEB warning](docs/snapshots/AEB_Warning.png) |
| **BRAKING** | **STOP_HOLD** |
| ![AEB braking](docs/snapshots/AEB_Braking.png) | ![AEB stop hold](docs/snapshots/AEB_Stop.png) |

The run goes through the states in order:

1. **MONITORING** (green): the barrier is still far down the road and no obstacle cluster is in the corridor (`corridor clear`, `0 cl`).
   The arbiter passes the MPC command through (`ctrl: mpc`).
   At 1.7 m/s, `d_safe` is 0.51 + 0.24 + 1.0 = 1.76 m, as the banner shows.
2. **WARNING** (orange): an obstacle is in the corridor and either TTC < 2.5 s or the distance is under 1.5 x `d_safe`, for 3 consecutive ticks (one noisy frame never triggers anything).
   Here the barrier's clusters are 5.4 m ahead with a TTC of 1.88 s.
   WARNING is advisory only: the MPC still drives, and the state drops back to MONITORING once the concern has been clear for 0.5 s.
3. **BRAKING** (red): TTC < 1.5 s **and** the distance is inside `d_safe`, again for 3 ticks.
   `aeb_node` publishes a zero-speed command on `/control/aeb_cmd` (keeping the last yaw rate so the car stays on its line), and the arbiter switches its source from `mpc` to `aeb`.
   BRAKING never releases while the car is moving; it only ends by reaching a standstill.
   The snapshot is the end of the stop: `v` is down to 0.1 m/s, so TTC is large again, but the state stays latched.
4. **STOP_HOLD** (magenta): the car is below 0.1 m/s, 1.7 m (reported, about 2.2 m true gap) short of the barrier.
   AEB keeps commanding zero speed for as long as anything is in the corridor.
   It hands control back (to MONITORING) only after the corridor has been completely empty for more than 1 s, for example when the barrier is removed; low TTC or a small `d_safe` at standstill does not count as clear, so the hold never releases with the obstacle still in front of the car.

A fifth state, **FAULT**, covers stale inputs (no depth or lidar for 0.5 s, or no odometry for 0.3 s): the arbiter stops the car, and a fault that interrupts BRAKING or STOP_HOLD resumes that state afterwards instead of releasing it.
On `urban_course_roadblock_oncoming`, the barrier is outside the corridor, and the shadow-mode tests recorded no WARNING or BRAKING at all.

### Lidar-based AEB

The same `aeb_node` can take its obstacles from a front lidar instead of the depth camera: `av_stack.launch.py aeb_sensor:=lidar` (or the `lidar` demo setup) sets its `obstacle_source` parameter to `lidar`, puts the sensor in the robot model and starts its bridge.
There is no separate lidar AEB node on purpose: the sensor only changes three things in `aeb_node` - which topic it subscribes to, its staleness check, and the extractor that turns a message into obstacle clusters.
The corridor, the braking model, the state machine with all of its hold and fault rules, and the arbiter contract are the same code for both sensors, so a fix to one is a fix to both.

![Lidar AEB braking for the barrier: Gazebo, the AEB and lane debug images, and the lidar cloud in RViz](docs/snapshots/AEB_Lidar.png)

The lidar demo on `urban_course_roadblock`, braking for the barrier at 3.0 m/s.
Left, Gazebo; middle top, the AEB debug image, whose banner names the sensor (`AEB lidar BRAKING`, distance 2.45 m, TTC 0.82 s, `d_safe` 2.64 m, 10 clusters); middle bottom, the lane debug image; right, RViz with the lidar cloud coloured by height (building walls as stacked ring lines, the ground rings arcing in front of the car, the barrier's returns ahead) and the arbiter's `ctrl: aeb` label above the car.

**The sensor.**
A Gazebo `gpu_lidar` on the front grille, 2.30 m ahead of the chassis centre and 0.60 m above the road, like a production AEB radar: no blind zone in front of the bumper, and few ground returns.
It scans 480 x 16 rays over 120 deg x 20 deg at 10 Hz with 2 cm Gaussian range noise; the lowest ring first reaches the road about 3.4 m ahead.
The cloud arrives as `/lidar/front/points` (`sensor_msgs/PointCloud2`, frame `front_lidar_link`) through its own bridge node, `gz_ros_bridge_lidar`, separate from the bridge that carries `/clock`, `/cmd_vel` and the camera, so the lidar can fail on its own.
The sensor exists in the robot model only when a setup asks for it (`aeb_sensor:=lidar` or `lidar:=true`); without it the robot description is byte-identical to the camera-only car, so the vlm and aeb setups pay no rendering cost.

**From cloud to obstacles.**
`av_common/point_cloud.hpp` holds the steps both sensors share: points are filtered by range, by lateral distance and by height above the road (the mount is unrotated, so the sensor-to-car transform is a translation), then bucketed into 0.5 m ground-plane cells, and a cell with enough points becomes one obstacle cluster.
The depth extractor feeds back-projected pixels into the same clustering, so a cluster means the same thing for both sensors; the lidar only needs fewer points per cluster (3 instead of 5), because a small box at 30 m is a handful of returns.
Like the camera, the lidar mount (`lidar_x` / `lidar_z` in `aeb_params.yaml`) is a parameter kept in sync with the URDF by hand, not a TF lookup.

**Measured behaviour.**

- Range, car parked, a 0.6 m box in the lane: the lidar sees it on every sample out to about 37 m from the sensor (reading 3-6 cm short), the depth extractor to about 25 m.
  The lidar extractor takes about 1 ms per cloud, the depth extractor about 30 ms per frame, against the 50 ms AEB tick.
- Closed-loop scenarios (box in the lane at 1, 2 and 3 m/s, half in the lane, on the right turn, appearing about 2.7 m ahead, beside the lane, in the oncoming lane, MPC killed mid-drive), each run with both sensors: lidar AEB matches depth AEB row by row.
  Both stop every in-lane case short of the box with no contact, the stop gaps differ by at most about 0.2 m, and neither sensor raised a false warning or brake for the off-lane boxes.
- In those scenarios the reported first detection is the same for both sensors (about 5, 9 and 15 m at 1, 2 and 3 m/s): AEB only reports what is inside the swept corridor, whose length grows with speed, so the lidar's longer reach does not change when it brakes at these speeds.
- **Sensor dropout:** killing only `gz_ros_bridge_lidar` while driving at 2 m/s puts `aeb_node` into `FAULT` (`stale lidar`) about 0.55 s later; the arbiter switches to its fault stop, the car is stationary about 1 s after the kill and does not move again.
  The depth camera cannot be tested this way, because its bridge also carries `/clock` and `/cmd_vel`.
- Lane following is unchanged with the lidar rendered: on the short barrier track the camera, depth and lane-path rates stay at 10 Hz, and depth AEB and lidar AEB brake at the same point and stop about 0.1 m apart.

The simulated lidar is deliberately simple: an instantaneous snapshot with no scan-time motion distortion, constant intensity, and no material dropouts or weather.

## Running it

Build first:

```bash
colcon build --symlink-install
source install/setup.bash

# scripts/ can lose the executable bit (e.g. after a Windows-side checkout);
# re-set it if you get "Permission denied":
chmod +x ./scripts/*.sh
```

### Three simulation setups

The VLM pipeline and the AEB layer never run together; pick one setup per run:

| Setup | Speed source | `av_stack.launch.py` | AEB reads | Who publishes `/cmd_vel` |
| --- | --- | --- | --- | --- |
| **vlm** | VLM sign chain (`/maneuver/target_speed` from `sign_maneuver_node`) | `aeb:=false` - no `aeb_node`, no arbiter | - | `mpc_tracker_v2` |
| **aeb** | fixed `/maneuver/target_speed` (no VLM) | `aeb:=true` (default) - `aeb_node` + `control_arbiter` | depth camera | `control_arbiter` |
| **lidar** | fixed `/maneuver/target_speed` (no VLM) | `aeb_sensor:=lidar` - as aeb, plus the front lidar and its bridge | front lidar | `control_arbiter` |

Without anything on `/maneuver/target_speed`, the MPC deliberately holds the car at 0.

### Demo (terminator grid)

`scripts/start_course_demo.sh [vlm|aeb|lidar]` lays out a 3x2 `terminator` grid (needs `terminator` and an X server; WSLg on Windows 11 provides one).
The lidar setup uses the aeb panes, except that pane 2 launches with `aeb_sensor:=lidar` and pane 5 also watches `/lidar/front/points`:

| Pane | vlm setup | aeb setup |
| --- | --- | --- |
| 1 | VLM pipeline (venv): `vlm_sign_node`, then `sign_maneuver_node` once Qwen is loaded | fixed `/maneuver/target_speed` publisher |
| 2 | `av_stack.launch.py aeb:=false`, held until Qwen is loaded | `av_stack.launch.py aeb:=true` |
| 3 | sign / maneuver narrative (`/vlm/sign_maneuver`) | every `/safety/aeb_status` state change + 1 Hz heartbeat |
| 4 | `/maneuver` setpoint + raw `/vlm/sign` | arbiter's active source + `/cmd_vel` |
| 5 | topic-rate health (camera, depth, lane path, sign reads) | topic-rate health (camera, depth, lane path, AEB status, `/cmd_vel`) |
| 6 | free shell with handy commands | free shell with handy commands |

```bash
./scripts/start_course_demo.sh                    # vlm setup (default)
STACK_NOWAIT=1 ./scripts/start_course_demo.sh     # don't wait for Qwen before starting the sim
DEMO_WORLD=urban_tight ./scripts/start_course_demo.sh

./scripts/start_course_demo.sh aeb                # aeb setup, default world urban_course_roadblock
DEMO_WORLD=urban_course_roadblock_oncoming ./scripts/start_course_demo.sh aeb
DEMO_SPEED=2.0 ./scripts/start_course_demo.sh aeb # lower speed cap (default 8.0 = planner profile governs)
DEMO_GZ_GL=software ./scripts/start_course_demo.sh aeb  # render Gazebo on the CPU instead of the GPU (see below)

./scripts/start_course_demo.sh lidar              # lidar setup, default world urban_course_roadblock
```

RViz shows each setup's own views: the lane debug image in all of them, plus `/maneuver/debug_image` (VLM sign view) in the vlm setup, or `/safety/aeb/debug_image`, the AEB obstacle / corridor markers and the arbiter's `ctrl:` label in the aeb and lidar setups; the lidar setup also shows the point cloud, coloured by height.
`av_stack.launch.py` writes one RViz config per setup (`$TMPDIR/av_stack_{vlm,aeb,lidar}.rviz`) from `src/robot_bringup/config/av_stack.rviz`, dropping the other setups' displays by name; edit the base file, not the copies.
Other env vars: `DEMO_RVIZ=false`, `DEMO_CURVE_SPLICE=true`, `VLM_VENV`; the header of `scripts/start_course_demo.sh` documents them all.
Teardown is closing the terminator window, or Ctrl-C in pane 2: pane 2 stops the launch and any Gazebo server or GUI process it left behind (with the GUI, Gazebo runs those two in process groups of their own, which a closing terminal never reaches).

### Gazebo rendering: GPU or software

Gazebo renders the RGB-D camera (and the lidar, when present) either on the GPU or on the CPU, selected by the `gz_gl` launch argument:

| Mode | How it renders | Default for |
| --- | --- | --- |
| `gpu` | Mesa's d3d12 driver on the WSLg GPU (Windows 11 + WSL2) | the short-track demos (`start_course_demo.sh aeb` and `lidar`) |
| `software` | llvmpipe on the CPU (`LIBGL_ALWAYS_SOFTWARE=1`) | the vlm demo, `av_stack.launch.py`, `urban_world.launch.py`, `run_headless_lap.sh`, `run_aeb_scenario.sh` |

```bash
./scripts/start_course_demo.sh aeb                          # gpu (demo default)
DEMO_GZ_GL=software ./scripts/start_course_demo.sh aeb      # software
ros2 launch robot_bringup av_stack.launch.py gz_gl:=gpu     # stack only
GZ_GL=gpu ./scripts/run_headless_lap.sh novlm my_lap        # headless lap
```

The demo prints the selected mode when it starts (`Gazebo rendering: gpu (Mesa d3d12 on the WSLg GPU)`) and again in pane 2, and rejects any value other than `gpu` or `software`.

On one measured aeb-setup `urban_course` lap per mode, `gpu` gave depth at 9.7 Hz instead of 8.6 Hz, 2 depth gaps over 0.3 s instead of 18, and roughly half the Gazebo CPU load (about 195 % instead of 420 %), with the same lap metrics.
With the lidar added, both modes still deliver the camera, depth and lidar at 10 Hz on the short barrier track.
The full-track vlm demo, the stack-only launch, the headless laps and the AEB scenarios stay on `software`, so new runs remain comparable with the recorded baselines.

Notes:

- The mode only applies to the Gazebo process; RViz and every other node keep software GL.
  Every rendering sensor lives in that one process, so the camera and the lidar always switch together.
- `gpu` removes `LIBGL_ALWAYS_SOFTWARE` from Gazebo's environment and sets `GALLIUM_DRIVER=d3d12` and `MESA_EXTENSION_OVERRIDE=GL_ARB_copy_image`.
  Mesa's d3d12 driver advertises only OpenGL 4.2, and without that extension Gazebo's renderer (Ogre-Next 2.2) aborts while creating the scene.
- Launch Gazebo with `-r` (the launch files do), never with `--headless-rendering`: on WSL2 that freezes the camera feed after 40-90 s.
- The vlm demo stays on `software` by default: on the GPU, Gazebo would share it with the Qwen model.
- Gazebo's GUI can occasionally fail to create its OpenGL context at startup (`[QT] Failed to create OpenGL context`), which takes the simulation down too; it has not been reproducible on demand.
  The arbiter then stays in `fault_aeb_status` and the car never moves; close the window and start the demo again.

### Stack only (launch args directly)

```bash
# aeb setup: Gazebo + sedan + bridge + EKF + perception + planning + MPC + aeb_node + arbiter + RViz
ros2 launch robot_bringup av_stack.launch.py
ros2 topic pub -r 10 /maneuver/target_speed std_msgs/msg/Float64 '{data: 8.0}'   # second terminal: let it drive

# vlm setup: same without aeb_node / arbiter, then start the two VLM nodes (see above)
ros2 launch robot_bringup av_stack.launch.py aeb:=false

# lidar setup: AEB on the front lidar (adds the sensor and its bridge)
ros2 launch robot_bringup av_stack.launch.py world:=urban_course_roadblock aeb_sensor:=lidar gz_gl:=gpu
# the lidar as a sensor only, with AEB still on the depth camera
ros2 launch robot_bringup av_stack.launch.py lidar:=true

# a tighter-radius course, a barrier world, or without RViz
ros2 launch robot_bringup av_stack.launch.py world:=urban_tight
ros2 launch robot_bringup av_stack.launch.py world:=urban_course_roadblock
ros2 launch robot_bringup av_stack.launch.py rviz:=false

# tune without touching code
ros2 launch robot_bringup av_stack.launch.py mpc_config:=/path/to/mpc_params_tuned.yaml
ros2 launch robot_bringup av_stack.launch.py aeb_config:=/path/to/aeb_params_tuned.yaml
```

See the docstring of `src/robot_bringup/launch/av_stack.launch.py` for the full set of launch arguments (lane-detection debug surfaces, the local planner's road-curvature assist, etc).

### Recorded laps and AEB scenarios

| Script | What it does |
| --- | --- |
| `scripts/run_headless_lap.sh novlm\|vlm LABEL [DURATION_S] [WORLD]` | One reproducible headless lap: kills leftovers, launches the stack (`novlm` = aeb setup, `vlm` = vlm setup with the real sign chain), records, cleans up. `STACK_ARGS="aeb_sensor:=lidar"` makes a `novlm` lap the lidar setup. |
| `scripts/record_av_lap.sh --label L --duration S --world W` | Records a lap from a running stack into `data/av_laps/<session>/`. |
| `scripts/analyze_av_lap.sh latest --plots` | Cross-track error, speed tracking, completion, AEB events -> `metrics.json`, `report.md`, `lap.png`; a change is a regression only relative to a recorded baseline lap. |
| `scripts/record_aeb_scenario.sh --label L --duration S --world W --target-speed V` | Records an AEB scenario (status, distance, TTC, commands) into `data/aeb_scenarios/<session>/`. |
| `scripts/analyze_aeb_scenario.sh latest --plots` | Metrics for a recorded scenario (AEB distance vs. ground-truth bumper gap to the obstacle, measured along the lane, state timeline, arbiter hand-offs) -> `metrics.json`, `report.md`, plots. |
| `scripts/run_aeb_scenario.sh ROW SPEED` | One headless closed-loop AEB scenario end to end on `urban_course` (box in the lane, half in it, beside it, in the oncoming lane, on the turn, appearing suddenly, MPC killed, lidar dropout), recorded and analyzed. `AEB_SENSOR=lidar` runs it in the lidar setup (session label suffix `_lidar`). |
| `tools/avkit/compare_aeb_sessions.py --since STAMP` | Depth vs lidar table: pairs each lidar scenario session with the newest depth session of the same row and speed. |
| `scripts/probe_obstacle_range.sh` | Standstill detection range of the depth and lidar obstacle extractors: spawns a box at increasing distances in front of the parked car and tabulates what each sensor sees. |
| `tools/avkit/sensor_rates.py --duration S` | Wall-clock rate, worst gap and sim real-time factor for the camera, depth, lidar, lane path and AEB status topics. |

```bash
./scripts/run_headless_lap.sh novlm my_lap 240        # aeb setup, planner speed
./scripts/run_headless_lap.sh vlm my_vlm_lap 320      # vlm setup (needs the torch venv)
./scripts/analyze_av_lap.sh latest --plots

./scripts/run_aeb_scenario.sh inlane 2.0                          # depth AEB, box in the lane
AEB_SENSOR=lidar ./scripts/run_aeb_scenario.sh inlane 2.0         # the same with lidar AEB
AEB_SENSOR=lidar ./scripts/run_aeb_scenario.sh sensor_dropout 2.0 # lidar bridge killed mid-approach
```

`tools/avkit/aeb_test_driver.py` drives AEB scenarios: a fixed target speed, optional stop at a given x or on BRAKING, barrier removal after a hold, and resume.

## Roadmap

This is a work in progress.
Planned areas of continued work include further MPC/perception tuning, further VLM sign-arbitration robustness (board latch, ROI hand-off) tuning, a composable velocity-constraint coordinator with lidar adaptive cruise control as its first feature (following a moving lead vehicle, so features like ACC or a stop line plug in without another `min()` in the MPC), a more realistic lidar model (a real unit's scan pattern, scan-time motion distortion, material and weather effects), additional road-world variety, and expanded sensing/behavior features.
