---
name: sync-knowledge-base
description: This skill should be used at the end of any change to AEB, lidar, ACC, coordinator, launch or tooling code, or when the user asks to "update the notes", "sync the docs", "update CLAUDE.md status", "keep the knowledge base in sync", or when the stop-check hook asks for it. Last verified 2026-11-13.
---

# Keeping the knowledge base in sync with the code

The notes say what is built and measured; `CLAUDE.md` files say the rules; the index says the wiring; the plans say what was intended and how the code deviates.
A code change is finished when all four agree with it.

## Checklist (do every line, say "no change" where that is true)

1. **Status table** in the matching note: `local/notes/lidar_implementation.md` section 1 for anything lidar, coordinator or ACC; `local/notes/aeb_status_and_lessons.md` section 1 for AEB, arbiter and scenario tooling.
   Update the row, the commit column and the "Last updated" line.
2. **The note's body**: the section describing the changed node (architecture, parameters and sync rules, measurements, "How to run"); add a dated subsection for a new step, never rewrite history.
3. **Deviations**: if the code now differs from a plan, a dated "Revised" line in the plan at that spot and a bullet in its revision block; the note's deviation section if the difference guards against a known bug.
4. **Index**: `python3 tools/avkit/kb_index.py --update` for new or changed topics, parameters and launch remaps; hand-edit the `plans`, `notes`, `reports`, `invariants` fields of new nodes; `--check` must pass.
5. **Sync rules**: a new duplicated number (an extrinsic, a length mirrored in a tool) gets an entry in `tools/avkit/param_sync_rules.yaml`; `scripts/check_param_sync.sh` must pass.
6. **Per-directory `CLAUDE.md`**: a new invariant, pitfall or required check goes into the directory's file, not the root; the root only gets a new pointer if a new directory appears.
7. **Decisions**: a user decision that the code cannot explain goes into `local/kb/decisions.md` with its date.
8. **README**: only for a new mode, script or user-visible behaviour, in the short style it has (modes, snapshots, running instructions; no design detail).
9. **Skills**: if a command, row, argument or file a skill names changed, fix the skill and run its "How to check this skill is still right" block.
10. **Memory**: a correction from the user becomes a `feedback_*` memory; status never goes into memory.

## How to check this skill is still right

The same commands are the checks to run at the end of every sync:

```bash
python3 tools/avkit/kb_index.py --check
bash scripts/check_param_sync.sh
grep -rn $'\xe2\x80\x94' local/notes/*.md local/kb/*.md CLAUDE.md */CLAUDE.md README.md   # no em dashes
grep -rnE 'Day [0-9]|day[0-9]+\.md|local/plans/' src scripts tools README.md --include=*.py --include=*.cpp --include=*.hpp --include=*.sh --include=*.yaml --include=*.md   # no plan references in shipped files
```

End with the list of files touched and the status rows changed.
