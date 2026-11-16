#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/../check_lib.sh"
# every layer the table names exists where it says
need_file .claude/hooks/hook.py .claude/hooks/protected_paths.yaml .claude/hooks/tests/run.sh .claude/hooks/ref_lint.py \
          .claude/hooks/check_harness.sh local/kb/invariants.yaml local/harness/evals/evals.yaml \
          tools/avkit/kb_index.py local/plans/ai_harness_plan.md
done_
