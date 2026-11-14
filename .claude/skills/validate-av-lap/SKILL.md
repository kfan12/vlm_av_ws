---
name: validate-av-lap
description: This skill should be used when the user asks to "record a lap", "analyze lap metrics", "check for a regression", "validate the AV stack", "measure cross-track error", "compare against a baseline", "run the AEB matrix", "validate ACC", or wants quantified (not just visual) evidence that a change to the vlm_av_ws driving stack did or did not break something. Last verified against the code 2026-11-13.
---

# Validating a change to the vlm_av_ws driving stack

There is no unit-test suite.
Evidence is a recorded run with a number compared against a recorded reference, and a change is a regression only relative to that number and beyond the known noise.
Reproduce the problem end to end before fixing it, and rerun the same scenario after.

## Which run answers which question

| Question | Run | Reference |
| --- | --- | --- |
| Did lane following or the MPC change? | one lap per affected setup, `run_headless_lap.sh` | `docs/baselines/README.md` (software GL, `urban_course`; noise about +/-0.02 m at 3 m/s) |
| Did AEB change? | the scenario rows, `run_aeb_scenario.sh ROW SPEED`, with `AEB_SENSOR=lidar` for the lidar setup | `docs/reports/aeb_scenario_matrix.md`, `docs/reports/lidar_aeb_matrix.md` |
| Did the lidar path change? | `AEB_SENSOR=lidar` rows plus `sensor_dropout 2.0`; the short-track lap pair (`run_headless_lap.sh novlm L 150 urban_course_roadblock` with and without `STACK_ARGS="aeb_sensor:=lidar"`) | `lidar_aeb_matrix.md` (matrix and the lap-pair table) |
| Did ACC change? | `run_acc_scenario.sh follow|lead_brake|stop_go|curve|cut_in [SET_SPEED]`, plus a no-lead lap with `STACK_ARGS="aeb_sensor:=lidar acc:=true"` | the Accept table in `local/plans/lidar/l5_lidar_acc_coordinator.md`; `docs/reports/lidar_acc.md` once it exists |
| Did a solver or weight change help? | `ros2 run av_behavior_cpp mpc_closed_loop_sim <lane.csv> v=3.0` first (`tools/avkit/export_lane_csv.py maps/<world>.json`), then a lap | the same sim on the previous code |

Worlds: laps on `urban_course` (full) for the baselines, `urban_course_roadblock` for anything lidar that drives, except the approved exceptions (scenario rows and all ACC runs on `urban_course`); `run_headless_lap.sh` defaults to `urban_course`, so pass the world.
GL: software for everything that is compared with a baseline or a report.

## Commands

```bash
./scripts/run_headless_lap.sh novlm LABEL 240 urban_course            # aeb setup lap
STACK_ARGS="aeb_sensor:=lidar" ./scripts/run_headless_lap.sh novlm LABEL 150 urban_course_roadblock
./scripts/analyze_av_lap.sh latest --plots                            # data/av_laps/<session>/{metrics.json,report.md,lap.png}

./scripts/run_aeb_scenario.sh inlane 2.0                              # depth; rows: inlane outlane outlane_left partial curve sudden mpc_timeout
AEB_SENSOR=lidar ./scripts/run_aeb_scenario.sh sensor_dropout 2.0     # lidar only
python3 tools/avkit/compare_aeb_sessions.py --since YYYYMMDD_HHMMSS   # depth vs lidar table per row and speed

./scripts/run_acc_scenario.sh follow 3.0                              # data/aeb_scenarios/<stamp>_acc_follow_3.0mps/{acc_metrics.json,acc.png}
```

Each runner kills leftovers, launches headless, waits for the health gate (sim time advancing, odometry, depth, lidar, `aeb_node` and the required nodes alive), records, analyzes, and prints the recorder's row counts.

## Reading a result

1. `INVALID RUN` in the runner output voids the session; fix the cause (see `log/<runner>/<label>/stack.log`), do not analyze it.
2. Look at the plot (`lap.png`, `timeline.png`, `acc.png`) with the Read tool; a metric can pass while the plot shows a stall or a spike.
3. Compare the metrics with the reference row: sequence, first seen, stop gap, gate response, time to standstill, contact, false positives for AEB; cross-track RMSE and max, speed RMSE, completion, AEB events for laps; gap errors, time gap, cap latency, restart lag, binding counts for ACC.
4. Report pass/fail per metric with both numbers and the noise band; one lap beyond the band is a reason to rerun, two are a regression.

## Recorder and analyzer rules (when a tool is missing)

Build only what the question needs, in the shape of the existing kit (`tools/avkit/CLAUDE.md`): `spin_once` against a wall-clock deadline, never `rclpy.shutdown()` from a callback, numpy only, plots behind `--plots`, `STREAMS` kept identical between recorder and analyzer.

## How to check this skill is still right

```bash
ls docs/baselines docs/reports
for f in scripts/run_aeb_scenario.sh scripts/run_acc_scenario.sh; do awk '/^case "\$ROW" in/{n++} n==1 && /^esac/{exit} n==1' "$f" | grep -oE '^\s+[a-z_]+\)'; done
```

The row lists above must match those `case` labels.
