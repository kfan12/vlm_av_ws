#!/usr/bin/env python3
"""probe_sign_texture.py - offline Qwen read of a sign image (Day 12).

Feeds an image file through the same model_runner + prompt + parser stack the
node uses and prints the parsed label. Run under the venv with the workspace
sourced:

  source ~/venvs/vlm_robot/bin/activate && source install/setup.bash
  python3 scripts/probe_sign_texture.py src/urban_gazebo/models/sign_left_urban/meshes/sign.png
  python3 scripts/probe_sign_texture.py /tmp/fpv_screenshot.png

The bare texture is the easy case; a Gazebo FPV screenshot (small board, dark
software-GL rendering) is the honest one - test both.
"""
import sys

from PIL import Image

from vlm_planner_py.json_parser import parse_sign_class
from vlm_planner_py.model_runner import load_qwen_model, run_qwen_inference
from vlm_planner_py.vlm_prompts import SIGN_PROMPT

img = Image.open(sys.argv[1]).convert('RGB')
model, processor = load_qwen_model()
raw, dt = run_qwen_inference(model, processor, img, SIGN_PROMPT)
print(f'raw ({dt:.1f}s): {raw!r}')
print('parsed:', parse_sign_class(raw))
