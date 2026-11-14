---
name: run-scenario
description: This skill should be used when the user asks to "run the inlane row", "run an AEB scenario", "run the lidar dropout test", "run the ACC follow scenario", "rerun row X", "run the matrix", or wants one recorded AEB or ACC scenario executed end to end and judged against the published reference row. Args: aeb|acc ROW [SPEED] [depth|lidar]. Last verified against the code 2026-11-13.
---

# Run one scenario and judge it

One scenario is one command, a health gate, a recording, an analysis and a verdict against a reference row.
Run it through the `wsl-ros-ops` rules (script file, background, log file).

## 1. Resolve the arguments

| Kind | Rows | Speed | Sensor | World | GL |
| --- | --- | --- | --- | --- | --- |
| aeb | `inlane outlane outlane_left partial curve sudden mpc_timeout sensor_dropout` | required (1.0, 2.0, 3.0 in the matrices) | `AEB_SENSOR=depth` (default) or `lidar`; `sensor_dropout` is lidar only | `urban_course` (fixed by the script) | software |
| acc | `follow lead_brake stop_go curve cut_in` | set speed, default 3.0 | lidar (fixed) | `urban_course` (fixed) | software |

Unknown row or an aeb row with no speed: stop and ask.

## 2. Check for leftovers, then run

```bash
pgrep -af 'ign gazebo|parameter_bridge|lead_vehicle|ros2 launch'
cd ~/vlm_av_ws
[AEB_SENSOR=lidar] ./scripts/run_aeb_scenario.sh ROW SPEED   > ~/scenario_work/ROW.log 2>&1
./scripts/run_acc_scenario.sh ROW [SPEED]                     > ~/scenario_work/ROW.log 2>&1
```

A run takes 2-4 minutes (stack up to 120 s, recording 40-90 s, analysis).

## 3. Validate the run before reading numbers

In the log, in this order:

1. `stack never came up healthy` or `arbiter never handed over`: look at `log/<runner>/<label>/stack.log` (GL context failure, frozen sim, a crashed node) and rerun once; report if it repeats.
2. Any `INVALID RUN:` line voids the session (no odometry, no AEB status, no lead odometry, no ACC state, dead lead driver); say which and why, do not analyze.
3. The `rows:` counts: `aeb_status` about 20 per second of recording, `odom_truth` about 40 per second; far fewer means a stall.
4. The `control source` lines: `none -> fault_aeb_status -> (stop_no_fresh_source ->) mpc` at start is normal.

## 4. Compare with the reference row

- aeb, depth: `docs/reports/aeb_scenario_matrix.md` (2026-10-25) and the same-day depth rows in `docs/reports/lidar_aeb_matrix.md` (2026-11-08, preferred: same environment).
- aeb, lidar: `docs/reports/lidar_aeb_matrix.md`; `sensor_dropout` has its own table there (fault within 0.7 s, standstill within 1.5 s, never moves again).
- acc: the Accept table in `local/plans/lidar/l5_lidar_acc_coordinator.md` (and `docs/reports/lidar_acc.md` once written); `analyze_acc_run.py` prints every metric that table names.

Metrics to put side by side: `source_sequence`, `first_braking.gt_standoff_m` (first seen), `standstill_gt_standoff_m` (stop gap), `command_gate_response_sec`, `time_to_standstill_s`, `contact`, `false_positive_brakes`; for ACC `min_gap_m`, `contact`, `aeb_intervened`, `steady_gap_error_m`, `steady_time_gap_s`, `acc_gap_error_m`, `cap_drop_latency_s`, `restart_lag_s`, `max_over_set_speed_mps`, `binding_counts`.
Tolerances: stop gaps within about 0.2 m of the reference row (the measured depth-lidar spread), first seen within 1 m, 0 false positives, no contact; one run outside that is a reason to rerun, two are a finding.
Look at `timeline.png` / `acc.png` before writing the verdict.

## 5. Report

Three lines minimum: the session name and log path; pass/fail per metric with both numbers; what was invalid or unusual.
If the run changed a published result, say that the report needs updating (`write-test-report`), do not edit the report silently.

## How to check this skill is still right

```bash
sed -n '/^case "\$ROW" in/,/^esac/p' scripts/run_aeb_scenario.sh scripts/run_acc_scenario.sh
grep -n 'INVALID RUN' scripts/_av_stack_common.sh scripts/run_aeb_scenario.sh scripts/run_acc_scenario.sh
```
