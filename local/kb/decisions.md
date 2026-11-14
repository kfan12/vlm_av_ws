# Decisions the code does not explain

One dated line per user decision that shapes the stack but cannot be derived from the code or the git history.
Newest at the bottom.
Memory files and plan "Revised" notes point here instead of repeating these.

- 2026-09-07: `curve_splice` stays off until the EKF drift is measured (a map-fixed curve spliced onto a drifting odom frame reads as fake curvature).
- 2026-10-12: Day 16's AEB is kept with fixes A-F rather than rebuilt from the plan; `day16.md` code must never be pasted over `src/av_safety_cpp`.
- 2026-10-18: `depth_stale_sec` 0.5 s (0.3 tripped a one-tick FAULT about every 45 s at 8.5 Hz depth).
- 2026-10-18: lap acceptance is against `docs/baselines/` numbers, with the lap-to-lap noise band; never against a feeling.
- 2026-10-25: Day 18 is closed with row 6 (depth dropout) as an accepted open exception (one bridge carries `/clock`).
- 2026-10-25: Day 19's coordinator is deferred until a second speed constraint exists; the trigger was ACC (built 2026-11-08 to 11-09, lidar setup only, in its corrected L5 form).
- 2026-10-25: DDS on Cyclone defaults; the loopback XML is not to be re-enabled without a passing `lane_node` run.
- 2026-10-25: the lidar is a third demo setup of its own; the vlm and aeb setups stay byte-identical (robot description unchanged with `lidar:=false`).
- 2026-10-25: lidar mount on the front grille (x 2.30, z 0.60); one `aeb_node` with an `obstacle_source` parameter, no forked lidar node; a simple `gpu_lidar` with Gaussian noise first, a realistic sensor model (scan pattern, scan timing, returns, weather) kept as a TODO.
- 2026-11-01: WSL `networkingMode=nat` (mirrored mode let host Wi-Fi blips kill DDS mid-lap).
- 2026-11-01: `gz_gl:=gpu` is for the short-track aeb and lidar demos only; full-track vlm sims, the launch default, headless laps and scenarios stay on software GL so runs stay comparable with the baselines.
- 2026-11-01: the L1 range probe runs on `urban_course` (its boxes reach past the roadblock barrier; the car never drives).
- 2026-11-08: lidar tests that drive use `urban_course_roadblock`; approved full-track exceptions are the range probe, `run_aeb_scenario.sh` rows (they pair with the depth matrix) and every L5 ACC run and lap (the barrier would be a stopped lead).
- 2026-11-08: L3 and L4 laps on the short track; L5 ACC scenarios on the full track.
- 2026-11-08: the L5 lead-vehicle spike is closed by user decision with fact 3 (AEB ignores the moving lead) not measured; the scenarios cover it.
- 2026-11-08: L5 is written self-contained; Day 19 is design history, not a prerequisite.
- 2026-11-09: the lead vehicle is a 6.0 x 2.1 x 3.0 m box truck (visuals only; one frictionless collision box).
- 2026-11-13: the AI harness is built per `local/plans/ai_harness_plan.md`; status lives in the notes' status tables, not in `CLAUDE.md`.
