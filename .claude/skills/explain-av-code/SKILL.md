---
name: explain-av-code
description: This skill should be used when the user asks to "explain this code", "explain how X works", "walk me through <file/node/function>", "why is it implemented this way", "what does this parameter do", "how does lane_node / local_planner / the MPC / AEB / the lidar / ACC / the coordinator / the VLM sign chain work", or otherwise wants to understand an implementation in the vlm_av_ws stack, including the design reasoning and theory from the project's notes and plans. Last verified against the code 2026-11-16.
---

# Explaining vlm_av_ws code with the knowledge base

The code in `src/` and `tools/` says what happens.
The knowledge base under `local/` says why: design intent, rejected alternatives, derivations, tuning history and pitfalls.
A good explanation joins the two and says where they disagree.
`local/` is gitignored and may lag the code; the code is the source of truth for behaviour, the knowledge base for intent.

## Step 1: read the code first

Read the file(s) in full, plus what the explanation needs: headers, `CMakeLists.txt`/`package.xml`, the launch block and YAML that set its parameters, and the topics it publishes and subscribes.
The directory's own `CLAUDE.md` lists the invariants that code participates in.

## Step 2: find the matching knowledge-base material

The map from code to its design documents is generated from `local/kb/index.yaml`, which `kb_index.py --check` keeps true (every listed file must exist):

```bash
python3 tools/avkit/kb_index.py --table | grep -i '<node, package or file>'   # plans, notes and reports per piece of code
python3 tools/avkit/kb_index.py --show <node>                                 # one node's topics, parameters, setups, invariants
```

Nodes carry their own `plans`, `notes` and `reports`; code that is not a node (shared libraries, the robot description, the world, tooling, the overtake track) is under `packages`.
Then `grep -rn '<file, class, function or parameter name>' local/` for anything the index does not name.
A missing row is a gap to fix in the index, not a reason to keep a list here.

`local/plans/README.md` has the day index and ground rules; `local/plans/lidar/README.md` the lidar track's conventions; `local/kb/decisions.md` the user decisions that explain choices the code cannot.

For orientation before reading a node, two generated pages (rebuilt from the code by the Stop hook): `local/kb/source_guide.html` gives each node's cycle step by step and every function with its line range (the text is in `local/kb/source_guide/descriptions.py`), and `local/kb/architecture.html` the topics between nodes per setup.
Their descriptions are a summary; the code still decides.

Large or non-Markdown sources, extract only the relevant part:

- `local/plans/av_minimal_stack_daily_plan.html` (about 370 KB): `grep -n '<summary><code>src/<path></code>'`, then read to the closing `</details>`; entity-escaped.
- `local/plans/av_build_log.html` (about 330 KB): `grep -n -i '<term>'` for the history of a bug or decision.
- `local/notes/av_upgrade.pdf`: the Read tool's `pages` parameter; skim the contents first.

## Step 3: compare plan against implementation

Diff the real file against the plan's embedded source or described design: renamed or added parameters, changed defaults, extra or missing logic, different topic names.
Plans carry dated "Revised" markers where the code deliberately differs; the notes' deviation sections (`aeb_status_and_lessons.md` section 2) and `git log -p --follow -- <file>` give the reasons.
Say every difference explicitly ("the plan sets X to 0.5, the code uses 0.8") and never present a plan's rationale as applying to code that no longer does what the plan describes.
Check `src/` for what exists before describing a planned part as built.

## Step 4: write the explanation

Pitch it at the question; a one-line "what does this do" gets a short answer.
For a full walkthrough:

1. **Role in the stack**: where it sits (the diagram in `README.md` or `CLAUDE.md`), what it consumes and produces, which setups run it (`index.yaml` `setups`).
2. **How it works**: execution order (callbacks, timers, main loop) with clickable references (`[aeb_node.cpp:120](src/av_safety_cpp/src/aeb_node.cpp#L120)`), small snippets only where they carry the point.
3. **Why it is built this way**: the reasoning from plans, notes and decisions, with the source cited; include the theory when it is what makes the code make sense.
4. **Invariants and pitfalls**: the directory `CLAUDE.md` rules it participates in, and recorded pitfalls.
5. **Plan vs code differences** from step 3.

Mark which statements come from the code and which from the knowledge base.
If the knowledge base says nothing about a choice, say so and label your own reasoning as inference.
Flag any bug, stale comment or unintended plan/code mismatch at the end, separately.

## How to check this skill is still right

```bash
bash .claude/skills/explain-av-code/check.sh
```
