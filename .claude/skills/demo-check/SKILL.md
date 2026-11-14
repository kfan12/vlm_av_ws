---
name: demo-check
description: This skill should be used when the user asks to "check the demo", "verify the demo still works", "test the terminator demo", "check the panes", "check the RViz displays", or after a change to start_course_demo.sh, _course_demo_pane.sh, _course_demo_watch.py, the launch file or the RViz config. Args: mode (vlm|aeb|lidar|acc). Last verified against the code 2026-11-13.
---

# Checking a demo mode

The demo is the user-facing entry point, so a change to it is verified by running it, pane by pane, and by confirming the teardown.
The GUI opens on the user's screen; say so before starting, and prefer the headless pane check below when only the panes changed.

## Headless pane check (no window)

Each pane is a script that can run on its own against a headless stack:

```bash
source scripts/_av_stack_common.sh; av_cleanup
ros2 launch robot_bringup av_stack.launch.py world:=W rviz:=false gz_gui:=false <mode args> > /tmp/stack.log 2>&1 &
DEMO_MODE=<mode> DEMO_WORLD=W DEMO_RUNDIR=/tmp/demo_check timeout -s INT 120 bash scripts/_course_demo_pane.sh 1 > pane1.log 2>&1 < /dev/null &
DEMO_MODE=<mode> DEMO_WORLD=W DEMO_RUNDIR=/tmp/demo_check timeout -s INT 60 bash scripts/_course_demo_pane.sh 3 > pane3.log 2>&1 < /dev/null
DEMO_MODE=<mode> bash scripts/_course_demo_pane.sh 6 < /dev/null | head -20      # prints the handy commands and stops
av_cleanup
```

Pane 6 and `start_course_demo.sh -h` work with no stack at all and are the first check after any script edit.

## Full demo check (window)

Run `./scripts/start_course_demo.sh <mode>` and go through the gate table; the user confirms the visual items.

| Pane / item | vlm | aeb | lidar | acc |
| --- | --- | --- | --- | --- |
| 1 | "VLM ready" then `sign_maneuver_node` running | target speed published at 10 Hz | same | lead spawned and driving, then the cruise target |
| 2 | stack up, Gazebo and RViz open, no GL-context error | same | same, `aeb_node obstacle_source` lidar | same, plus `acc_node`, `cruise_constraint`, `trajectory_coordinator` in the launch output |
| 3 | sign narrative lines | AEB state changes with a 1 Hz heartbeat; BRAKING at the barrier | same | ACC lines: lead gap, speed, cap, binding, AEB state |
| 4 | setpoint and raw sign | `ctrl: mpc`, then `aeb` at the barrier | same | `ctrl: mpc` throughout a follow |
| 5 | camera, depth, lane path, sign rates | plus AEB status and `/cmd_vel` | plus `/lidar/front/points` at about 10 Hz | plus ACC state, authorized profile, lead odometry |
| RViz | lane debug image, `/maneuver/debug_image` | AEB debug image, obstacle and corridor markers, `ctrl:` label | plus the point cloud coloured by height | plus the ACC marker and `ACC:` label |
| Car | drives the course, slows for signs | brakes short of the barrier, no contact | same | follows the lead, no AEB braking |
| Teardown | closing the window leaves no `ign gazebo`, `parameter_bridge`, `lead_vehicle.py` or `ros2 topic pub` process (`pgrep -af`) | same | same | same, and no lead bridge or driver |

A GL-context failure at startup (`[QT] Failed to create OpenGL context`) is a known intermittent WSLg fault: relaunch once, report if it repeats.

## Report

Per pane: what it printed (one line) and pass/fail; the RViz and car items as the user confirmed them; the `pgrep` after teardown.

## How to check this skill is still right

```bash
./scripts/start_course_demo.sh -h | sed -n '/pane/,/col3 bottom/p'
grep -nE '^  [a-z0-9:|]+\)' scripts/_course_demo_pane.sh      # every MODE:PANE case the pane script handles
```
