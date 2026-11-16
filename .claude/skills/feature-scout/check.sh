#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/../check_lib.sh"
need_file src/av_feature_msgs/msg src/sedan_description/urdf/sedan.urdf.xacro src/urban_gazebo/worlds \
          local/kb/decisions.md local/kb/index.yaml local/notes/lidar_implementation.md local/notes/aeb_status_and_lessons.md
# the report and its research notes must stay out of git
git check-ignore -q local/reports || bad "local/reports is not gitignored"
git check-ignore -q local/research_notes || bad "local/research_notes is not gitignored"
# the class table the scoring leans on
need_grep '^\| 1\. Emergency override' .claude/skills/add-av-feature/SKILL.md
need_grep '^\| 4\. Path-modifying' .claude/skills/add-av-feature/SKILL.md
# the hand-off names skills that exist
need_file .claude/skills/add-av-feature/SKILL.md .claude/skills/write-plan-step/SKILL.md
done_
