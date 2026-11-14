---
name: explain-av-code
description: This skill should be used when the user asks to "explain this code", "explain how X works", "walk me through <file/node/function>", "why is it implemented this way", "what does this parameter do", "how does lane_node / local_planner / the MPC / AEB / the lidar / ACC / the coordinator / the VLM sign chain work", or otherwise wants to understand an implementation in the vlm_av_ws stack, including the design reasoning and theory from the project's notes and plans. Last verified against the code 2026-11-13.
---

# Explaining vlm_av_ws code with the knowledge base

The code in `src/` and `tools/` says what happens.
The knowledge base under `local/` says why: design intent, rejected alternatives, derivations, tuning history and pitfalls.
A good explanation joins the two and says where they disagree.
`local/` is gitignored and may lag the code; the code is the source of truth for behaviour, the knowledge base for intent.

## Step 1: read the code first

Read the file(s) in full, plus what the explanation needs: headers, `CMakeLists.txt`/`package.xml`, the launch block and YAML that set its parameters, and the topics it publishes and subscribes.
The directory's own `CLAUDE.md` lists the invariants that code participates in.

## Step 2: find the matching knowledge-base material

Start from `local/kb/index.yaml`: every node has its `plans`, `notes` and `reports` fields (`grep -n '<node or topic>' local/kb/index.yaml`).
Then `grep -rn '<file, class, function or parameter name>' local/` for anything else.
The map, for orientation:

| Code | Plans (`local/plans/`) | Notes and reports |
| --- | --- | --- |
| `av_common_cpp` geometry, projection, map I/O, debug tap | `day01.md` | `local/notes/ground_plane_theory_and_derivation.md` |
| `av_common/point_cloud.hpp` (shared clustering) | `lidar/l1_lidar_obstacle_extractor.md` | `lidar_implementation.md` section 4 |
| `sedan_description` (URDF, camera and lidar mounts) | `day02.md`, `lidar/l0_lidar_sensor_spike.md` | `lidar_implementation.md` section 3 |
| `urban_gazebo`, `scripts/generate_course_world.py`, signs | `day03.md` | - |
| `robot_bringup` launch, bridges, RViz; `robotcar_localization` | `day04.md`, `day14.md`, `lidar/l3_lidar_setup_demo.md` | `map_sensor_fusion_localization_survey.md` |
| `lane_node.cpp`, `ground_cal.hpp`, `img_access.hpp` | `day05-07.md`, `lane_node_day5_day6_guide.md`, `lane_node_day6_day7_completion_guide.md` | `ground_plane_theory_and_derivation.md` |
| `local_planner.cpp` | `day08.md`, `local_planner_day8_guide.md` | `map_sensor_fusion_localization_survey.md` (curve splice vs drift) |
| `av_behavior_cpp/src/mpc/*`, `mpc_tracker_v2_node.cpp` | `day09-11.md` | `src/av_behavior_cpp/src/mpc/README.md`, `docs/baselines/README.md` |
| `vlm_planner_py` | `day12.md`, `day13.md` | - |
| `tools/avkit` lap kit | `day15.md` | `docs/baselines/README.md` |
| `av_safety_cpp` AEB (`aeb_node`, extractors, corridor, braking model) | `day16.md`, `lidar/l2_aeb_obstacle_source.md` | `aeb_status_and_lessons.md` (deviations), `av_upgrade.pdf`, `docs/reports/aeb_roadblock_test.md` |
| `lidar_obstacle_extractor.hpp`, `obstacle_probe`, `probe_obstacle_range.sh` | `lidar/l1_lidar_obstacle_extractor.md` | `lidar_implementation.md` section 4, `docs/reports/lidar_obstacle_range.md` |
| `av_control_cpp` arbiter | `day17.md` | `av_upgrade.pdf` |
| AEB scenario tooling, `run_aeb_scenario.sh`, `compare_aeb_sessions.py` | `day18.md`, `lidar/l4_lidar_aeb_validation.md` | `docs/reports/aeb_scenario_matrix.md`, `lidar_aeb_matrix.md` |
| `av_planning_coordinator` (coordinator, `constraint_from_maneuver_node`, `VelocityConstraint`) | `lidar/l5_lidar_acc_coordinator.md` tasks 2-3 (`day19.md` is the design history) | `av_upgrade.pdf`, `lidar_implementation.md` section 6.7 |
| `av_features_cpp/acc_node`, `lead_vehicle.py`, `analyze_acc_run.py`, `run_acc_scenario.sh` | `lidar/l5_lidar_acc_coordinator.md` tasks 1, 4-7 | `lidar_implementation.md` sections 6.6-6.7 |
| demo scripts (`start_course_demo.sh`, `_course_demo_pane.sh`, `_course_demo_watch.py`) | `day14.md`, `lidar/l3_lidar_setup_demo.md` | - |

`local/plans/README.md` has the day index and ground rules; `local/plans/lidar/README.md` the lidar track's conventions; `local/kb/decisions.md` the user decisions that explain choices the code cannot.

Large or non-Markdown sources, extract only the relevant part:

- `local/plans/av_minimal_stack_daily_plan.html` (about 370 KB): `grep -n '<summary><code>src/<path></code>'`, then read to the closing `</details>`; entity-escaped.
- `local/plans/av_build_log.html` (about 330 KB): `grep -n -i '<term>'` for the history of a bug or decision.
- `local/notes/av_upgrade.pdf`: the Read tool's `pages` parameter; skim the contents first.

## Step 3: compare plan against implementation

Diff the real file against the plan's embedded source or described design: renamed or added parameters, changed defaults, extra or missing logic, different topic names.
Plans carry dated "Revised" markers where the code deliberately differs; the notes' deviation sections (`aeb_status_and_lessons.md` section 2) and `git log -p --follow -- <file>` give the reasons.
Say every difference explicitly ("the plan sets X to 0.5, the code uses 0.8") and never present a plan's rationale as applying to code that no longer does what the plan describes.
Check `src/` for what exists before describing a planned part as built.

## Step 4: write the explanation

Pitch it at the question; a one-line "what does this do" gets a short answer.
For a full walkthrough:

1. **Role in the stack**: where it sits (the diagram in `README.md` or `CLAUDE.md`), what it consumes and produces, which setups run it (`index.yaml` `setups`).
2. **How it works**: execution order (callbacks, timers, main loop) with clickable references (`[aeb_node.cpp:120](src/av_safety_cpp/src/aeb_node.cpp#L120)`), small snippets only where they carry the point.
3. **Why it is built this way**: the reasoning from plans, notes and decisions, with the source cited; include the theory when it is what makes the code make sense.
4. **Invariants and pitfalls**: the directory `CLAUDE.md` rules it participates in, and recorded pitfalls.
5. **Plan vs code differences** from step 3.

Mark which statements come from the code and which from the knowledge base.
If the knowledge base says nothing about a choice, say so and label your own reasoning as inference.
Flag any bug, stale comment or unintended plan/code mismatch at the end, separately.

## How to check this skill is still right

```bash
python3 tools/avkit/kb_index.py --check        # the index matches the code
ls local/plans/lidar local/notes docs/reports
```

Every row in the table above must name files that exist.
