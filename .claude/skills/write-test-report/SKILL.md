---
name: write-test-report
description: "This skill should be used when the user asks to \"write up the results\", \"write the test report\", \"document the matrix\", \"create docs/reports/x.md\", \"summarize the ACC runs\", or after a validation step produced sessions whose numbers must become a report. Args: report name and the sessions (a --since stamp or a list). Last verified 2026-11-13."
---

# Writing a test report under docs/reports/

Reports are the reference numbers later runs are judged against, so every number in one must come from a recorded session and be reproducible from the files under `data/`.
`docs/reports/` is gitignored (local only); the template is `docs/reports/lidar_aeb_matrix.md`.

## 1. Collect the sessions

```bash
ls data/aeb_scenarios | tail -40 ; ls data/av_laps | tail -20
python3 tools/avkit/compare_aeb_sessions.py --since YYYYMMDD_HHMMSS      # newest session per row, speed and sensor
python3 tools/avkit/analyze_acc_run.py <session>                          # acc_metrics.json
./scripts/analyze_av_lap.sh <session>                                     # metrics.json
```

Re-analyze every session with the current analyzer before tabulating (an analyzer fix changes old numbers; say so if it did).
Exclude sessions whose runner log carries `INVALID RUN`, and say how many were excluded and why.

## 2. Structure (one sentence per line, no em dashes)

1. **Title and date**, the plan step it serves, the commit the code was at.
2. **Outcome**: the verdict in one bold sentence, then the two or three facts that carry it, including anything that had to be fixed before the runs could be scored.
3. **Setup**: the exact commands and env (`AEB_SENSOR`, world, GL, durations), the batch script, the time window, the analyzer version note.
4. **Matrix**: one table, rows as the plan lists them, columns being the metrics the Accept table names; rows that do not apply get a dash, never a blank.
5. **Special rows** with their own tables (dropout timing, lap pairs, ACC rows) where the matrix columns do not fit.
6. **Accept**: the plan's Accept table reproduced with PASS / FAIL / NOT MEASURED and the numbers.
7. **Findings**: what the numbers say that the Accept table does not ask, each as a bold claim followed by the evidence; open items named as open.
8. **Changes made during this step**: every code, script and tool change the runs required, so a reader knows which earlier reports are no longer comparable.

## 3. Rules

- Numbers come from `metrics.json` / `acc_metrics.json` / the comparison table; never retyped from memory, never rounded beyond what the tool prints.
- Name every session in the text or the tables so a reader can open it.
- A correction to an older report goes into that report as a dated "Correction" paragraph pointing at the new one (see `aeb_scenario_matrix.md`), and the new report explains what changed.
- Reports may cite plans (`local/plans/...`); shipped files may not.
- Look at every plot you cite.

## 4. Afterwards

Update the note's status table and `CLAUDE.md` pointers if the report closes a step (`sync-knowledge-base`); add the report path to the node's `reports` list in `local/kb/index.yaml`.

## How to check this skill is still right

```bash
head -12 docs/reports/lidar_aeb_matrix.md
grep -n '^## ' docs/reports/lidar_aeb_matrix.md
```
