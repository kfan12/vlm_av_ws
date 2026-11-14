---
name: check-param-sync
description: This skill should be used when the user asks to "check the extrinsics", "verify the camera/lidar mount parameters match", "move the lidar/camera", "change the sedan dimensions", or whenever a value that is duplicated by hand across the xacro, the YAML configs, node defaults or the analysis tools is edited. Last verified 2026-11-13.
---

# Duplicated numbers and how they are checked

This stack duplicates sensor extrinsics and vehicle dimensions by hand instead of looking them up (no TF, no shared header): the camera's `cam_x`/`cam_z`/`cam_pitch`, the lidar's `lidar_x`/`lidar_z`, the half-length 2.25 m (`ch_l / 2`) used as the bumper offset, the lead truck's length.
Each copy lives in a different file, and a mismatch is silent: perception projects onto the wrong ground plane, AEB reads the wrong distance, ACC's gap is off by a constant.

`tools/avkit/param_sync_rules.yaml` lists every copy as a rule: a name, the value's owner (the xacro, or the tool that defines it), and the files and regexes that must agree with it.
`scripts/check_param_sync.sh` evaluates them and exits non-zero with the mismatching file, line and both values.

## Changing a duplicated value

1. Change the owner first (the xacro for a mount; `lead_vehicle.py` for the lead's size).
2. `bash scripts/check_param_sync.sh` prints every copy that now disagrees; change each one.
3. Rebuild the packages whose YAML or defaults changed, and rerun the validation that the value affects (a lap for the camera, an AEB row for the lidar, `analyze_acc_run.py` on an existing session for the lead length).
4. Record the change in the note's parameters section (`lidar_implementation.md` section 5) and, for a mount, in `docs/baselines/README.md` if a baseline depends on it.

## Adding a new duplicated value

Add a rule to `param_sync_rules.yaml` in the same change as the new copy; the `param-sync` hook runs the check on every edit of a listed file.
Prefer not to add copies at all: a tool can read the xacro property with a regex instead of hard-coding the number.

## How to check this skill is still right

```bash
bash scripts/check_param_sync.sh && echo SYNC_OK
python3 -c "import yaml; print(len(yaml.safe_load(open('tools/avkit/param_sync_rules.yaml'))['rules']), 'rules')"
```
