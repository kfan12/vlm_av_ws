#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/../check_lib.sh"
# every tool the checklist names exists and has the mode it describes
need_file tools/avkit/kb_index.py scripts/check_param_sync.sh tools/avkit/param_sync_rules.yaml \
          local/kb/architecture/build.py local/kb/architecture/layout.py \
          local/kb/source_guide/build.sh local/kb/source_guide/descriptions.py \
          local/harness/graph_check/run.sh local/kb/invariants.yaml local/kb/decisions.md \
          .claude/hooks/check_harness.sh .claude/hooks/hook.py
for flag in -- --update --check --table; do [ "$flag" = -- ] || need_grep "'$flag'" tools/avkit/kb_index.py; done
need_grep "'mark-synced': mark_synced" .claude/hooks/hook.py
need_grep 'planned=True' local/kb/architecture/layout.py local/kb/architecture/build.py
# the status tables are where item 1 says they are
need_grep '^## 1\b' local/notes/lidar_implementation.md
need_grep '^## 1\b' local/notes/aeb_status_and_lessons.md
need_grep '^Last updated' local/notes/lidar_implementation.md
# item 7: the registry lists where each rule is restated
need_grep 'restated_in' local/kb/invariants.yaml
done_
