---
name: run-av-stack
description: This skill should be used when the user asks to "run the demo", "launch the stack", "start the simulation", "build and run the AV stack", "run the sim", "launch av_stack", "see it drive", "start the acc demo", or wants to build, launch, or debug the vlm_av_ws Gazebo/ROS 2 driving stack in any of its four setups (vlm, aeb, lidar, acc). Last verified against the code 2026-11-13.
---

# Running the vlm_av_ws AV stack

Build, launch and check the stack.
`CLAUDE.md` has the setup table and the invariants; this skill has the mechanics.
Every ROS command runs inside WSL; follow the `wsl-ros-ops` skill for how to run them from this Windows-side session.

## Build

```bash
cd ~/vlm_av_ws
colcon build --symlink-install                               # whole workspace
colcon build --symlink-install --packages-select <pkg> [...]  # a few packages
source install/setup.bash
chmod +x ./scripts/*.sh                                      # if a checkout dropped the mode bits
```

## Pick the setup, then the entry point

Four setups, never more than one at once; which one is the first question to settle (ask if the request does not say):

| Setup | Launch args | What runs on top of lane following |
| --- | --- | --- |
| vlm | `aeb:=false` | VLM nodes started by hand in the venv; the MPC publishes `/cmd_vel` |
| aeb | (default) | `aeb_node` on depth + `control_arbiter` |
| lidar | `aeb_sensor:=lidar` | `aeb_node` on the lidar + arbiter; the lidar is rendered only here and in acc |
| acc | `aeb_sensor:=lidar acc:=true` | plus `acc_node`, `cruise_constraint`, `trajectory_coordinator`; the MPC reads `/planning/authorized/*` |

**Demo window** (terminator, needs an X server; WSLg provides one):

```bash
./scripts/start_course_demo.sh vlm|aeb|lidar|acc
./scripts/start_course_demo.sh -h            # every pane, env var and default
```

Defaults: vlm and acc on the full `urban_course` with software GL; aeb and lidar on `urban_course_roadblock` with gpu GL.
Env vars: `DEMO_WORLD`, `DEMO_SPEED`, `DEMO_GZ_GL=gpu|software`, `DEMO_RVIZ=false`, `STACK_NOWAIT=1` (vlm), `DEMO_LEAD=0`, `DEMO_LEAD_S`, `DEMO_LEAD_PROFILE` (acc).
In the vlm setup the car does not move until pane 1 prints "VLM ready" (2-3 min cold load); not a hang.
In the acc setup pane 1 waits for the stack, spawns and drives the lead truck, then publishes the cruise target.
Teardown: close the window or Ctrl-C in pane 2; pane 2 stops the launch and any `ign gazebo server|gui` left behind.

**Stack only**:

```bash
ros2 launch robot_bringup av_stack.launch.py [aeb:=false | aeb_sensor:=lidar [acc:=true]] [world:=W] [rviz:=false] [gz_gl:=gpu] [gz_gui:=false]
ros2 topic pub -r 10 /maneuver/target_speed std_msgs/msg/Float64 '{data: 8.0}'   # nothing moves without this (or the VLM)
```

The docstring of `src/robot_bringup/launch/av_stack.launch.py` lists every argument; read it before guessing a flag.
`acc:=true` without `aeb_sensor:=lidar`, or `aeb_sensor:=lidar` with `aeb:=false`, are refused at launch on purpose.

**Headless, scripted** (the right choice for anything unattended): `scripts/run_headless_lap.sh`, `run_aeb_scenario.sh`, `run_acc_scenario.sh` (see `validate-av-lap` and `run-scenario`).

**VLM nodes** (vlm setup, two terminals, after the stack is up):

```bash
source ~/venvs/vlm_robot/bin/activate
python3 -m vlm_planner_py.vlm_sign_node --ros-args -p use_sim_time:=true -p vlm_query_rate_hz:=0.5
python3 -m vlm_planner_py.sign_maneuver_node --ros-args --params-file src/vlm_planner_py/config/sign_maneuver_params.yaml -p use_sim_time:=true
```

## Before launching

```bash
pgrep -af 'ign gazebo|parameter_bridge|lead_vehicle|ros2 launch'   # leftovers block or corrupt the next run
```

If something is there and the user did not start it on purpose, stop it (`source scripts/_av_stack_common.sh; av_cleanup`); if a `pkill` is refused by the permission mode, report the PIDs and ask once.

## Verify it is working

```bash
ros2 topic hz /camera/front/depth/image_raw      # lane_node ticks on this; a stall freezes planning silently
ros2 topic hz /lane/path_odom /plan/path_odom
ros2 topic info /cmd_vel -v                      # exactly one publisher: mpc_tracker_v2 (vlm) or control_arbiter (others)
ros2 topic echo /control/active_source           # aeb/lidar/acc: mpc / aeb / fault_aeb_status / stop_no_fresh_source
ros2 param get /aeb_node obstacle_source         # lidar/acc: must say lidar
ros2 topic echo /planning/binding_constraint     # acc: cruise / acc / hold
```

RViz gets a per-setup config (`$TMPDIR/av_stack_<setup>.rviz`); edit `src/robot_bringup/config/av_stack.rviz`, not the copy.

## Known failure modes

- Car never moves, no VLM: no `/maneuver/target_speed` (fail-safe), or in acc `binding` says `hold:` (a required constraint is missing, usually `acc_node` waiting for odometry and the plan).
- Arbiter stuck on `fault_aeb_status` right after launch: Gazebo's GUI failed to create its GL context (`[QT] Failed to create OpenGL context`); relaunch.
- Everything publishes but nothing changes: sim time frozen; `scripts/_av_stack_common.sh`'s `stack_healthy` is the check.
- A node planning in the wrong frame was started outside `av_stack.launch.py` (no `map_origin_*`).
- DDS errors (`rtps_init`, `ddsi_udp_conn_write`): a shell with a different `CYCLONEDDS_URI`, or WSL back in mirrored networking.

## How to check this skill is still right

```bash
ls scripts/start_course_demo.sh scripts/run_headless_lap.sh scripts/run_aeb_scenario.sh scripts/run_acc_scenario.sh
grep -n "DeclareLaunchArgument('" src/robot_bringup/launch/av_stack.launch.py | sed "s/.*('\([a-z_]*\)'.*/\1/"
./scripts/start_course_demo.sh -h | grep -E '^  (vlm|aeb|lidar|acc) '
```

The argument list and the mode list above must match those outputs.
