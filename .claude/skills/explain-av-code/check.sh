#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/../check_lib.sh"
python3 tools/avkit/kb_index.py --check > /dev/null || bad "kb_index.py --check fails (run it for the list)"
table=$(python3 tools/avkit/kb_index.py --table) || bad "kb_index.py --table failed"
[ "$(wc -l <<<"$table")" -gt 20 ] || bad "the code map has fewer rows than expected"
grep -q 'node `aeb_node`' <<<"$table" || bad "the code map has no aeb_node row"
python3 tools/avkit/kb_index.py --show aeb_node | grep -q 'subscribes' || bad "--show no longer prints a node's topics"
# the skill must not grow a hand-written code map again
[ "$(grep -c '^| `' "$SKILL_MD")" -le 2 ] || bad "the skill carries a hand table again; the map lives in local/kb/index.yaml"
need_file local/kb/source_guide.html local/kb/architecture.html local/kb/decisions.md local/plans/README.md local/plans/lidar/README.md local/notes/av_upgrade.pdf
done_
