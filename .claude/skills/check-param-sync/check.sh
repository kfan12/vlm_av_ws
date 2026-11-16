#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/../check_lib.sh"
bash scripts/check_param_sync.sh > /dev/null || bad "scripts/check_param_sync.sh fails (run it for the mismatch)"
bash scripts/check_param_sync.sh --if-listed README.md > /dev/null || bad "--if-listed is no longer silent for an unlisted file"
n=$(python3 -c "import yaml; print(len(yaml.safe_load(open('tools/avkit/param_sync_rules.yaml'))['rules']))") || bad "the rules file does not parse"
[ "${n:-0}" -ge 5 ] || bad "only ${n:-0} sync rules"
# the numbers the skill names are still the owner's values
need_grep 'cam_x' tools/avkit/param_sync_rules.yaml
need_grep 'lidar_x' tools/avkit/param_sync_rules.yaml
need_grep 'bumper_x_m.*2\.25' src/av_features_cpp/config/acc_params.yaml
skill_says '2\.25 m'
# the hook that runs it on every edit
need_grep "check_param_sync.sh', '--if-listed'" .claude/hooks/hook.py
done_
