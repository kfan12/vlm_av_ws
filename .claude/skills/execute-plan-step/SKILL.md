---
name: execute-plan-step
description: "This skill should be used when the user asks to \"do task N of L5\", \"execute the next plan step\", \"implement day 19\", \"finish l5\", \"do steps 5-7 in the plan\", or otherwise wants a task from a plan file under local/plans/ built, verified and accepted the way this project does it. Args: plan file and task number(s). Last verified 2026-11-13."
---

# Executing a plan step

Plans under `local/plans/` are tutorials with full code, written before the code existed and revised afterwards.
Executing a step means building what it names, proving its Accept table, and writing back what changed, without undoing anything the code already learned.
Run the `plan-reviewer` agent first when the step is older than the last commit touching its packages.

## 1. Read the step the right way

1. The plan's revision block at the top and every dated "Revised" line in the task: they override the code blocks below them.
2. The step's Goal, Tasks, Deliverables, Verify, Accept and Pitfalls sections.
3. What the step assumes exists: `ls src/ tools/avkit scripts`, `ros2 pkg executables`, `git log --oneline -5 -- <path>`, and the status table in the matching note.
   A deliverable that already exists is checked against the plan, not rewritten.
4. The matching note's deviation section (`aeb_status_and_lessons.md` section 2, `lidar_implementation.md` section 5-6): code that carries tested fixes is never overwritten with plan code.

## 2. Build only the named deliverables

- Follow the per-directory `CLAUDE.md` of every directory touched.
- The plan's code is the starting point; correct it where the code base moved (topic names, parameter names, the shared helpers in `scripts/_av_stack_common.sh`, the recorder streams), and strip any plan reference from shipped files.
- Build with `colcon build --symlink-install --packages-select <the step's packages>`; run `scripts/check_no_cv_bridge.sh` and `scripts/check_param_sync.sh` when they apply.
- Keep work scripts under `~/<step>_work/`, not in the repo.

## 3. Verify and accept

Run every command in the step's Verify section and quote its output.
Then the Accept table: one result per row, with the number and the session that produced it; a row that cannot be measured is reported as not measured, never as passed.
Validation runs follow `validate-av-lap` and `run-scenario` (health gate, INVALID RUN, references).
Ask the user before closing a row by decision.

## 4. Write back

- The plan: a dated "Revised YYYY-MM-DD" line at each place the code differs from the plan text, and a bullet in the revision block at the top; regenerate code blocks from the built files when the plan carries full code (a script like `~/l5_work/rewrite_plan_567.py` shows how).
- The note's status table (`local/notes/aeb_status_and_lessons.md` or `lidar_implementation.md`), its measurements section, and its "How to run" section.
- `local/kb/index.yaml` for new nodes or topics (`python3 tools/avkit/kb_index.py --update`), `tools/avkit/param_sync_rules.yaml` for new duplicated numbers, `local/kb/decisions.md` for user decisions.
- A `docs/reports/<step>.md` when the step produced measurements (`write-test-report`).
- `README.md` only for user-visible modes or scripts.

Do not commit; the user commits.
End with: what was built, each Accept row's result, every deviation written back, and what is left.

## How to check this skill is still right

```bash
grep -n '^### [0-9]\.' local/plans/lidar/l5_lidar_acc_coordinator.md
grep -n 'Revised 2026' local/plans/lidar/*.md | wc -l
```
