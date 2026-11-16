---
name: feature-scout
description: "This skill should be used when the user asks to \"research new features\", \"what features can we add\", \"scout features for the stack\", \"what should we build next\", \"find ADAS features to implement\", \"rank candidate features\", or otherwise wants candidate driving features found in outside sources (ADAS taxonomies, test protocols, open-source AV stacks, papers) and scored against what the vlm_av_ws stack can support. It researches and ranks; it never builds. Last verified against the code 2026-11-16."
---

# Scouting new features for vlm_av_ws

The output is a ranked, sourced list of candidate features, each scored against this stack's sensors, world, extension rules and validation tooling.
It ends with the user picking one; building it is the `add-av-feature` skill's job, never this one's.
Do not trust this file for what exists: every "already built" or "missing" claim comes from `src/`, the status tables or the index, checked in this run.

## Step 1: collect the local context (before any web search)

```bash
cd ~/vlm_av_ws
ls src/ ; ls src/av_feature_msgs/msg
sed -n '/^## 1/,/^## 2/p' local/notes/lidar_implementation.md local/notes/aeb_status_and_lessons.md
cat local/kb/decisions.md
grep -n 'description' local/kb/index.yaml | head -40
grep -n 'sensor\|lidar\|camera' src/sedan_description/urdf/sedan.urdf.xacro | head -40   # what the sedan carries
ls src/urban_gazebo/worlds src/urban_gazebo/models                                       # what the world can contain
```

Write down, as the constraint list every candidate is scored against:
- **Sensors:** the RGB-D front camera (resolution, FOV, depth rate under software GL) and the optional front lidar (FOV, range, lateral crop).
  Anything needing rear, side or radar sensing is a new sensor, which is a xacro, bridge and `param_sync_rules.yaml` change.
- **World:** the generated course (`src/urban_gazebo/worlds/`) holds buildings, road and ground only; signs, lead vehicle and obstacles are spawned by scripts.
  Anything needing lights, pedestrians, crossings, other lanes of traffic or road markings beyond the lane paint is a world-generation change.
- **Localization:** the EKF is not trusted (user decision); a candidate that needs map-frame accuracy scores low on fit unless it brings its own localization.
- **Extension rules** (root `CLAUDE.md`, `add-av-feature` step 1): a new package wired by launch remapping; features never steer and never raise speed; the four feature classes (emergency override, mandatory stop, restrictive speed cap, path-modifying); C++ in the driving and safety chains; class 2 needs the coordinator's `stop_distance_m` merge, class 4 is new design work.
- **Setups:** which of vlm, aeb, lidar, acc the feature would live in, and whether it needs the coordinator in a setup that does not run it today (a user decision).
- **Open problems:** the status tables' open items; a candidate that fixes one is worth more.

## Step 2: research outside sources

Use the `deep-research` skill when the session has it (it coordinates parallel researchers and a report writer); otherwise search directly.
Give the researchers the constraint list from step 1, so they look for features that fit, not a generic ADAS catalogue.
Split the research along these angles (one researcher each):

| Angle | Sources to prioritize |
| --- | --- |
| Feature taxonomies and test protocols | SAE J3016 and J3063 (ADAS terms), Euro NCAP assessment protocols (AEB car-to-car and VRU, lane support, speed assist, driver monitoring), UN R152 / R157 / R79, NHTSA pre-crash scenario typology |
| Open-source AV stacks | Autoware Universe planning and perception modules (behavior velocity planner modules: stop line, traffic light, crosswalk, intersection, blind spot, occlusion spot, run-out), Baidu Apollo scenarios, openpilot longitudinal and lateral features |
| Simulation and research | CARLA leaderboard and ScenarioRunner scenarios, OpenSCENARIO catalogues, recent papers on camera/lidar ADAS functions feasible with one front RGB-D camera and one front lidar, VLM-in-the-loop driving |

Every candidate a source suggests carries the citation it came from.

## Step 3: score every candidate

Score each criterion 1 (poor) to 5 (good), then rank by the weighted total.
Development cost gets little weight (user rule: prefer quality, robustness and maintainability over effort).

| Criterion | Weight | 5 means | 1 means |
| --- | --- | --- | --- |
| Value | 3 | closes a real safety or capability gap, or an open status-table item | a demo nicety |
| Rule fit | 3 | class 1-3, a new package plus a launch block | needs to steer, raise speed or edit `lane_node`, `local_planner` or the MPC |
| Sensor and world readiness | 2 | works with today's sensors and a spawn script | needs a new sensor and a world-generation change |
| Validation | 2 | a recorded scenario row with a clear pass metric, like the AEB and ACC rows | only judged by eye |
| Localization independence | 1 | sensor-relative only | needs map-frame accuracy from the EKF |
| Effort (inverse) | 1 | one node, days | new subsystem, weeks |

Never round a rule bend into a good fit score: name the rule it bends and leave it to the user.

## Step 4: write the report

Save it under `local/` (gitignored): research notes in `local/research_notes/<title>/`, the report at `local/reports/<title>.md`, one sentence per line, no em dashes.
Contents:
1. **Summary:** the top three with one sentence each on why.
2. **Constraint list** from step 1, with the commands' evidence.
3. **Ranked table:** feature, class, setup, score per criterion, weighted total.
4. **Per candidate:** what it is (with sources), value here, sensors and world assets needed, rule fit or the rule it bends, implementation sketch (nodes, topics, which extension point), validation (scenario rows, metric, pass bound), effort, risks.
5. **Rejected:** candidates dropped and why (steering, rear sensing, map dependence, out of scope).
6. **Sources.**

## Step 5: hand off

Report the top of the ranking to the user and ask which one to build; do not start building.
Once they choose, record the choice and its reason in `local/kb/decisions.md`, then follow `add-av-feature` for where it plugs in and `write-plan-step` for the plan file under `local/plans/`.
Never commit the report or the decision without asking.

## How to check this skill is still right

```bash
bash .claude/skills/feature-scout/check.sh
```
