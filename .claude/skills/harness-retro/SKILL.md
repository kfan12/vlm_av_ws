---
name: harness-retro
description: "This skill should be used when the user asks for a \"retro\", \"what went wrong this session\", \"what should the harness learn from this\", \"improve the harness from this session\", \"why did you keep doing X\", or at the end of a long session in which commands were refused, retried or corrected by the user. It turns the session's friction into proposed harness changes (a guard, a skill line, an eval, a memory) and applies none of them without the user. Last verified against the code 2026-11-16."
---

# Learning from a session

The harness exists because the same mistakes cost time more than once.
A retro finds the mistakes of this session and assigns each to the strongest layer that can own it.

## 1. Collect the friction

Go through the session (and `transcript_path` files of recent sessions when asked for more than this one) and list every:

- correction from the user ("no", "not like that", "I said", a repeated instruction);
- command that was refused by a hook or by the permission mode, and what was tried next;
- command that was run more than twice before it worked, and what finally differed;
- claim that turned out wrong (something "does not exist" that did, a number quoted from memory, a result reported without the run);
- step that was skipped and found later (a note not updated, a check not run, a file left non-executable);
- hook that fired wrongly, and hook that should have fired and did not.

One line each: what happened, what it cost, and the evidence (the command, the message).

## 2. Assign each to a layer

Strongest first; use the first one that can do the job.

| Layer | Use it when | Where |
| --- | --- | --- |
| A script that removes the failure | the right way can be one command | `scripts/`, `tools/avkit/` |
| A hook rule | the mistake is recognisable in a command or an edit | `.claude/hooks/hook.py`, `protected_paths.yaml`, with a case in `tests/run.sh` |
| A check that asserts | a document can go stale against the code | a skill's `check.sh`, `ref_lint.py`, `kb_index.py --check`, `invariants.yaml` |
| A skill or `CLAUDE.md` line | it needs judgement and context | the owning skill, or the directory's `CLAUDE.md` |
| An eval | the behaviour should be measured over time | `local/harness/evals/evals.yaml` |
| A memory | it is the user's preference and applies across sessions | a `feedback_*` memory, with the why |

Something that happened once and has no pattern gets no harness change; say so.
A rule that already exists and was not followed is not fixed by writing it again: move it one layer up.

## 3. Propose, then stop

Give the user a table: finding, evidence, proposed change, layer, the test that would prove it.
Apply nothing until they choose; a memory about their preferences is theirs to confirm.
For each accepted change: make it, add its test or eval, run `bash .claude/hooks/check_harness.sh`, and add a dated line to `local/plans/ai_harness_plan.md`.

## How to check this skill is still right

```bash
bash .claude/skills/harness-retro/check.sh
```
