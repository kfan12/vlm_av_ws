#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/../check_lib.sh"
T=docs/reports/lidar_aeb_matrix.md
need_file "$T"
# the template still has the sections the skill prescribes
for h in Outcome Setup Accept Findings 'Changes made'; do
    grep -qE "^## .*$h" "$T" || bad "the template $T has no '$h' section"
done
need_grep -- '--since' tools/avkit/compare_aeb_sessions.py
need_grep 'acc_metrics.json' tools/avkit/analyze_acc_run.py
need_grep 'metrics.json' tools/avkit/analyze_av_lap.py
need_grep 'Correction' docs/reports/aeb_scenario_matrix.md
git check-ignore -q docs/reports || bad "docs/reports is no longer gitignored (the skill says local only)"
# step 4: a report nobody names fails the index check
need_grep 'named by no node or package' tools/avkit/kb_index.py
done_
