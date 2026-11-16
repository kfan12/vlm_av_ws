---
name: sync-knowledge-base
description: This skill should be used at the end of any change to AEB, lidar, ACC, coordinator, launch or tooling code, or when the user asks to "update the notes", "sync the docs", "update CLAUDE.md status", "keep the knowledge base in sync", or when the stop-check hook or the session-start hook asks for it. Last verified 2026-11-16.
---

# Keeping the knowledge base in sync with the code

The notes say what is built and measured; `CLAUDE.md` files say the rules; the index says the wiring; the plans say what was intended and how the code deviates.
A code change is finished when all four agree with it.
When the change is a plan task the user built by hand, use `check-hand-build`: it reviews the code first and ends with this checklist.

## Checklist (do every line, say "no change" where that is true)

1. **Status table** in the matching note: `local/notes/lidar_implementation.md` section 1 for anything lidar, coordinator, ACC or overtake; `local/notes/aeb_status_and_lessons.md` section 1 for AEB, arbiter and scenario tooling.
   Update the row, the commit column and the "Last updated" line.
2. **The note's body**: the section describing the changed node (architecture, parameters and sync rules, measurements, "How to run"); add a dated subsection for a new step, never rewrite history.
3. **Deviations**: if the code now differs from a plan, a dated "Revised" line in the plan at that spot and a bullet in its revision block; the note's deviation section if the difference guards against a known bug.
4. **Index**: `python3 tools/avkit/kb_index.py --update` for new or changed topics, parameters and launch remaps; hand-edit the `plans`, `notes`, `reports`, `invariants` fields of new nodes, and the `packages` section for new code that is not a node; `--check` must pass (it also fails for a report that no entry names).
   Then rebuild the two generated pages (the Stop hook does it too, when a source is newer than a page); both must exit 0:
   `python3 local/kb/architecture/build.py` reports an edge with no route, a node in a demo setup with no position, a launch remap with no rule, or a planned node that now exists; fix it in `local/kb/architecture/layout.py` (a planned node drops `planned=True` and its `EXTERNAL` entry once built).
   `bash local/kb/source_guide/build.sh` reports a node without a write-up, a cycle step that no longer resolves, or a function without a description; add it to `local/kb/source_guide/descriptions.py`, written from reading the code.
   When a launch file, a topic or a remap changed: `bash local/harness/graph_check/run.sh aeb|lidar|acc` compares the running graph with the index (simulator, about 3 minutes per setup).
5. **Sync rules**: a new duplicated number (an extrinsic, a length mirrored in a tool) gets an entry in `tools/avkit/param_sync_rules.yaml`; `scripts/check_param_sync.sh` must pass.
6. **Per-directory `CLAUDE.md`**: a new invariant, pitfall or required check goes into the directory's file, not the root; the root only gets a new pointer if a new directory appears.
   A new rule also gets an entry in `local/kb/invariants.yaml` (owner, where it is restated, what enforces it).
7. **Decisions**: a user decision that the code cannot explain goes into `local/kb/decisions.md` with its date, in date order.
   A decision that changes a rule or an exceptions list is not done until the rule's text is changed too: look the rule up in `local/kb/invariants.yaml` and edit its `owner` and every file under `restated_in` in the same change (root `CLAUDE.md`, the per-directory files, the skills that restate it).
8. **README**: only for a new mode, script or user-visible behaviour, in the short style it has (modes, snapshots, running instructions; no design detail).
9. **Skills and agents**: if a command, row, argument or file one of them names changed, fix it; never write that something "does not exist yet" or exists "once written" (say where to check instead).
10. **Memory**: a correction from the user becomes a `feedback_*` memory; status never goes into memory.

## The checks to run at the end of every sync

```bash
bash .claude/hooks/check_harness.sh
```

One command: the hook tests, every skill's `check.sh`, the reference lint over skills, agents and `CLAUDE.md` files, the invariant registry, `kb_index.py --check`, the parameter and `cv_bridge` checks, both page builds, the text rules over shipped files, the analyzer golden test, and how far the evals are behind the harness.
It needs no model and no simulator; fix what it reports before going on.

When it passes, run `python3 .claude/hooks/hook.py mark-synced`: the SessionStart hook lists the code commits made after that point (the user commits by hand) and asks for the next sync.
End with the list of files touched and the status rows changed.

## How to check this skill is still right

```bash
bash .claude/skills/sync-knowledge-base/check.sh
```
