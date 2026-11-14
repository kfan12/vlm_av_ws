---
name: add-av-feature
description: This skill should be used when the user asks to "add a safety feature", "add a speed constraint", "add a stop line / traffic light / zone limit feature", "add a new driving feature", "coordinate a new feature with the MPC", "add lane change / obstacle avoidance", or otherwise wants to extend the vlm_av_ws driving stack with new perception, safety or behavior capability. Last verified against the code 2026-11-13.
---

# Adding a new AV feature to vlm_av_ws

Extend the stack the way its architecture review (`local/notes/av_upgrade.pdf`) intends: a new, independent package wired in by launch remapping, never an edit to `lane_node.cpp`, `local_planner.cpp` or the MPC solver.
Before anything, check what exists: `ls src/`, `ros2 pkg executables <pkg>`, and the status tables (`local/notes/aeb_status_and_lessons.md` section 1, `local/notes/lidar_implementation.md` section 1).
Do not trust this file or any plan for what is built.

## Step 1: classify the feature

A feature's class decides where it plugs in; never "wire it into the MPC and `std::min()` it".

| Class | Examples | Where it plugs in |
| --- | --- | --- |
| 1. Emergency override | AEB, manual e-stop, watchdog | Bypasses planning; publishes a candidate command that `control_arbiter` (`av_control_cpp`) selects by priority. Must work with the VLM, planner or MPC unhealthy. |
| 2. Mandatory stop | red light, stop sign, blocked road | A stop point in the speed profile at an arc length: a `VelocityConstraint` with `stop_distance_m` (the message carries the field; the coordinator's merge does not use it yet, so this class needs the coordinator extended, not a new mechanism). |
| 3. Restrictive speed cap | ACC, speed limit, curve speed, school zone, low VLM confidence | A `VelocityConstraint` (`maximum_speed_mps`, `valid_until`) on its own topic, listed in `trajectory_coordinator`'s `constraint_topics`. |
| 4. Path-modifying | lane change, obstacle avoidance, intersection turn | Exclusive path arbitration; not designed in this repo beyond the review's sketch (section 14). Flag it as new design work. |

Unsure: does it ever need to override the command regardless of planning?
Then class 1.
Otherwise: does it change where the car goes (class 4) or only how fast (2 or 3)?

## Step 2: reuse the existing extension points

- **Class 1:** a package like `av_safety_cpp` (own node, own typed status in `av_feature_msgs`, candidate command on `/control/<feature>_cmd`), picked up by the arbiter.
  Never a second `/cmd_vel` publisher.
- **Class 3 (worked example: ACC):** `src/av_features_cpp/src/acc_node.cpp` publishes `/feature/acc/constraint`; `src/av_planning_coordinator/config/coordinator_lidar_acc.yaml` lists it in `constraint_topics` (and in `required_constraint_topics` when its silence must hold the car); `av_stack.launch.py`'s `acc` block starts the nodes and remaps only the MPC onto `/planning/authorized/*`.
  A new constraint is one more node, one more topic in the YAML, one more launch block, no coordinator or MPC change.
  `constraint_from_maneuver_node` shows the minimal publisher (a source name, a cap, a validity window, a timer on the node clock).
- **The coordinator runs only in the acc setup today;** a feature that must also work in the vlm or aeb setup needs the setup to adopt the coordinator (a launch change plus a cruise constraint), which is a user decision to ask for.
- **Class 2:** extend `trajectory_coordinator_node.cpp`'s merge to honour `stop_distance_m` as a stop point in the profile; design it, say so, then build.

## Step 3: conventions

- C++ for anything in the driving or safety chain; `ament_cmake`, `src/<pkg>/{src,include/<pkg>,config}`, `-Wall -Wextra -Wpedantic`, link `av_common_cpp::av_common`, every tunable a `declare_parameter` with a comment, a `config/<node>_params.yaml`.
- Sensor geometry: derived intrinsics, extrinsics duplicated from the xacro and added to `tools/avkit/param_sync_rules.yaml`; point clouds through `av_common/point_cloud.hpp`; no `cv_bridge` (`scripts/check_no_cv_bridge.sh`).
- Typed messages with source, priority and expiry (`AebStatus`, `VelocityConstraint`), consumers reject stale data; never a bare `Bool`/`Float64` for a safety signal.
- Launch: the new nodes in `av_stack.launch.py`'s `TimerAction`, behind a launch argument with `choices`, with a guard that refuses an inconsistent setup; a RViz display added to `config/av_stack.rviz` and to the right `*_ONLY_DISPLAYS` set.
- Fail-safe direction: a missing input holds or stops the car (required constraint, arbiter stop), never "drive on unprotected".
- No plan references in any shipped file.

## Step 4: validate like the existing features

A scenario runner (`run_aeb_scenario.sh` / `run_acc_scenario.sh` shape, sourcing `scripts/_av_stack_common.sh`), a recorder stream and an analyzer metric for the new behaviour, a reference row in a `docs/reports/` file, and laps in the untouched setups to prove they are unchanged (`validate-av-lap`).
Then update the notes' status table, the index (`local/kb/index.yaml`) and the per-directory `CLAUDE.md` (`sync-knowledge-base`).

## How to check this skill is still right

```bash
ls src/ ; ls src/av_feature_msgs/msg
grep -n 'constraint_topics' src/av_planning_coordinator/config/*.yaml src/av_planning_coordinator/src/*.cpp
grep -n "acc = _bool_arg\|mpc_remaps" src/robot_bringup/launch/av_stack.launch.py
```
