---
name: wsl-ros-ops
description: This skill should be used whenever a command must run inside WSL from this Windows-side Claude Code session (any ros2, colcon, gazebo, python3 or script under ~/vlm_av_ws), when editing scripts on the \\wsl.localhost share, when reading ROS topics in a pipe, or when a sim is left running; it holds the operating rules that failed repeatedly before. Last verified 2026-11-16.
---

# Driving the WSL workspace from Windows

The workspace is `/home/kefan/vlm_av_ws` in WSL (`\\wsl.localhost\Ubuntu-22.04\home\kefan\vlm_av_ws` from Windows).
ROS, colcon and Gazebo only work inside WSL; files are read and written through the UNC path.

## Running commands

- **One-liners:** `MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu-22.04 -- bash -lc "cd ~/vlm_av_ws && <command>" < /dev/null`.
  `MSYS_NO_PATHCONV=1` stops Git Bash rewriting `/home/...` arguments; `< /dev/null` stops anything that reads stdin from hanging.
- **Anything with quotes, `$`, pipes, loops or more than one line:** write a script file under `/home/kefan/<step>_work/` (through the UNC path with the Write tool) and run `MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu-22.04 -- bash /home/kefan/<step>_work/x.sh < /dev/null`.
  Inline heredocs and `$(...)` through `wsl.exe -- bash -lc '...'` get mangled silently; the `guard-bash` hook refuses them, and it refuses a Git Bash `wsl.exe` call with a `/posix/path` argument and no `MSYS_NO_PATHCONV=1` (the path would become `C:/Program Files/Git/...`).
- **Python edits to repo files** go the same way: a patch script in the work directory, run from WSL, never a `python3 - <<EOF` inline.
- **Long runs** (a lap, a matrix, a scenario): `run_in_background: true`, then wait for the completion notification; do not poll or sleep.
  Write the run's output to a log file and read that file at the end.
- **PowerShell** is for Windows-side things only (listing the memory directory, `Start-Process` when a WSL process must outlive the call).

## Reading ROS topics

- Use `python3 tools/avkit/ros_probe.py`: `field TOPIC FIELD --for 10` prints one flushed `t value` line per message and ends by itself, `rate TOPIC... --for 10` gives rate and worst gap, `graph --for 30` the nodes with their publishers and subscribers (the CLI's `ros2 node list` sees only part of the graph here).
- If `ros2 topic echo` must be used in a pipe, it is block-buffered: `PYTHONUNBUFFERED=1 timeout -s INT 20 ros2 topic echo /topic msg/Type --field x | awk ...`.
  A plain `timeout` sends TERM and the buffered output is lost; Ctrl-C on a pipeline kills the consumer before it prints.
- `--once` with an explicit message type works even when the ros2 daemon is stale.
- Every echo and hz in a script gets a `timeout`.

## Editing files

- The Edit and Write tools drop the executable bit on this share: the `post-edit` hook restores the mode git records and sets it on a new file that starts with a shebang, and the Stop hook does the same for files written through a shell; check `git diff --summary` for `mode change` before a commit anyway.
- Some paths are guarded (`.claude/hooks/protected_paths.yaml`): generated pages, baseline and recorded sessions are refused; the driving chain, `aeb_node.cpp` and the hand-build targets ask the user first.
- Writing under `/home/kefan/...` works; `mkdir` under `//wsl.localhost/.../tmp` does not; symlinks under `install/` give I/O errors through the UNC path.
- Read PNGs (plots, debug frames) with the Read tool at the UNC path and look at them; a metric can pass while the plot shows the problem.

## Processes

- Before any launch: `bash scripts/stop_av_stack.sh -n` lists what is running.
  Leftovers block the busy guard or double a topic (two lead bridges forward `/lead/cmd_vel` twice).
- Stop everything with `bash scripts/stop_av_stack.sh` (the runners' process list, TERM then KILL, including a GUI Gazebo's server and client, which a terminal hangup never reaches).
- If the permission mode refuses it, do not look for another way to kill the processes: report the PIDs and what they block, and ask once.
- The Stop hook reports simulator processes this session started and left running; stop them or say why they stay.
- A batch of runs: `scripts/run_scenario_batch.sh ROWS_FILE`, never a hand-written loop.
- `git commit`, `push` and history rewrites ask the user (the `git-guard` rule); the user commits by hand, and a commit message never carries an agent co-author line.
- Never start a GUI Gazebo from a background shell; use `gz_gui:=false` for unattended runs.

## Environment

- `source /opt/ros/humble/setup.bash && source ~/vlm_av_ws/install/setup.bash` in every script; `export PYTHONNOUSERSITE=1` for system Python that imports `cv2`.
- `CYCLONEDDS_URI` stays unset; a shell with a different value cannot see the stack.
- The WSL wall clock can step by about a second; sim time (`/clock`) is what recordings use.

## How to check this skill is still right

```bash
bash .claude/skills/wsl-ros-ops/check.sh
```
