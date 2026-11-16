---
name: debug-sim-run
description: "This skill should be used when the user asks to \"debug this run\", \"why did the car stop / weave / brake / lose the lead\", \"find the root cause\", \"investigate the short gap\", \"why did this scenario fail\", \"it worked yesterday\", or reports any wrong behaviour of the driving stack in the simulator. It is the evidence-first debugging procedure: reproduce as a recorded run, read the recording, one hypothesis, one change, the same run again. Args: a session or a symptom. Last verified against the code 2026-11-16."
---

# Debugging a simulator run

A fix is only as good as the reproduction it was tested against.
The order below is fixed: reproduce, read the evidence, hypothesize, change one thing, rerun the same thing.
For a long investigation hand it to the `sim-debugger` agent with the session and the symptom.

## 1. Reproduce it as a recorded run

- Find the run that shows the symptom the way a user met it: a scenario row (`run-scenario`), a lap (`validate-av-lap`) or the demo (`run-av-stack`).
  A symptom seen only in the demo is first reproduced headless with the same world, setup, speed and GL mode.
- If a session already shows it, start from that session; do not rerun before reading it.
- Check the run is valid first: no `INVALID RUN` line, the row counts sane, sim time advancing (`log/<runner>/<label>/stack.log`).
  A frozen sim, a GL-context failure or a leftover process from an earlier run explains many "bugs".
- No reproduction, no fix: say so and ask for the conditions.

## 2. Read the evidence before the code

1. The plot (`lap.png`, `timeline.png`, `acc.png`) with the Read tool: when does it start, what changes first.
2. The streams around that time (`data/<kind>/<session>/*.jsonl`, `t` is sim time): the arbiter's `active_source`, the AEB state, `binding`, the ACC state fields, speed and cross-track; a few lines of Python with numpy, never by eye over thousands of rows.
3. `stack.log` at the same sim time: node warnings, QP fallback, "no fresh depth", a restart.
4. A live look when the recording lacks the signal: `python3 tools/avkit/ros_probe.py field|rate|graph`, never a piped `ros2 topic echo`.
   If the signal is not recorded at all, adding a recorder stream or a debug topic is the first change, before any fix.
5. The notes for the same symptom: the lessons sections of `local/notes/aeb_status_and_lessons.md` and `local/notes/lidar_implementation.md`, and the directory's `CLAUDE.md`; many symptoms are recorded with their cause.

## 3. One hypothesis, stated so it can fail

Write it as: "X happens because Y; if so, signal Z shows W at time T".
Then check Z in the recording.
Prefer the cheapest test that can refute it:

| Question | Cheapest test |
| --- | --- |
| MPC weights, solver, path handling | `ros2 run av_behavior_cpp mpc_closed_loop_sim <lane.csv> key=value` before any Gazebo lap |
| a perception or clustering decision | replay the recorded input through the old and the new node (a bag or the recorded stream) |
| a timing or freshness fault | the timestamps in the recording against the timeout parameters |
| an analyzer number | rerun the analyzer on the same session; the golden test (`local/harness/golden/golden.py --check`) says whether the analyzer itself moved |
| "a known limitation" in a comment or a note | test it live; do not trust it (several were stale) |

A comment, a note or a memory that explains the symptom is a hypothesis too, not a finding.

## 4. Change one thing and rerun the same run

- One change per run; a second change waits for the first result.
- The same row, speed, world, sensor and GL mode as the reproduction, and the neighbouring rows that the change could move (a lap for anything in the driving chain).
- Compare numbers with the noise in mind: one run outside the band is a rerun, two are a result (`docs/baselines/README.md`).
- A fix in `lane_node`, `local_planner`, the MPC solver, `aeb_node` or a hand-build target is proposed first; the `guard-edit` hook asks for those paths.

## 5. Close it

- The report: symptom, reproduction (session), evidence (signal, time, value), cause, the change, the before and after numbers with both sessions, and what was not explained.
- "Not root-caused" is a valid result; write what was ruled out and what to record next.
- Write the cause into the note's lessons section and, if it is a standing trap, the directory's `CLAUDE.md` (`sync-knowledge-base`).

## How to check this skill is still right

```bash
bash .claude/skills/debug-sim-run/check.sh
```
