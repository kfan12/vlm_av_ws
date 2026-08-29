#!/usr/bin/env python3
"""visualize_lane_dump.py - offline visualization of lane_node's ingest_mask()
pipeline (Day 5/6 debugging). No ROS needed: reads the PNGs/JSON that
lane_node.cpp's dump_debug_frame() writes when the dump_dir parameter is set.

Enable dumping on the node:
  ros2 run av_perception_cpp lane_node --ros-args \
    -p dump_dir:=/tmp/lane_dump -p dump_every_n:=10
  mkdir -p /tmp/lane_dump   # the node does not create the directory itself

Then visualize a frame:
  python3 scripts/visualize_lane_dump.py /tmp/lane_dump            # latest dumped frame
  python3 scripts/visualize_lane_dump.py /tmp/lane_dump --seq 12   # a specific frame
  python3 scripts/visualize_lane_dump.py /tmp/lane_dump --list     # show available frame indices

Eight panels, one row per color: raw RGB + a top-down ego-frame scatter of
the final clouds on the left, then white's three pipeline stages (raw color
gate -> after horizon-blank + drop_small_blobs -> the actual pixels that made
it into white_cloud_) and yellow's same three stages. The cloud-mask panels
are image-space, not the mask itself: they mark ONLY pixels that were
evaluated by the per-pixel loop AND passed depth-validity/z-gate/range, so
they'll look sparse/dotted next to the solid color masks -- that's expected,
not a bug, since the stride skips most columns entirely.
"""
import argparse
import glob
import json
import os
import re
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np


def available_seqs(dump_dir):
    files = glob.glob(os.path.join(dump_dir, "*_rgb.png"))
    seqs = []
    for f in files:
        m = re.match(r"(\d+)_rgb\.png", os.path.basename(f))
        if m:
            seqs.append(int(m.group(1)))
    return sorted(seqs)


def load_frame(dump_dir, seq):
    tag = f"{seq:05d}"

    def _read(name, flags):
        path = os.path.join(dump_dir, f"{tag}_{name}.png")
        img = cv2.imread(path, flags)
        if img is None:
            sys.exit(f"missing or unreadable: {path}")
        return img

    frame = {
        "rgb": cv2.cvtColor(_read("rgb", cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB),
        "white_raw": _read("white_raw", cv2.IMREAD_GRAYSCALE),
        "white_filtered": _read("white_filtered", cv2.IMREAD_GRAYSCALE),
        "white_cloud_mask": _read("white_cloud_mask", cv2.IMREAD_GRAYSCALE),
        "yellow_raw": _read("yellow_raw", cv2.IMREAD_GRAYSCALE),
        "yellow_filtered": _read("yellow_filtered", cv2.IMREAD_GRAYSCALE),
        "yellow_cloud_mask": _read("yellow_cloud_mask", cv2.IMREAD_GRAYSCALE),
    }
    with open(os.path.join(dump_dir, f"{tag}_clouds.json")) as f:
        clouds = json.load(f)
    return frame, clouds


def _npx(mask):
    return int((mask > 0).sum())


def plot_frame(dump_dir, seq):
    fr, clouds = load_frame(dump_dir, seq)

    fig, ax = plt.subplots(2, 4, figsize=(20, 9))
    fig.suptitle(f"lane_node ingest_mask() -- dump {dump_dir}, frame {seq:05d}")

    ax[0, 0].imshow(fr["rgb"])
    ax[0, 0].set_title("raw RGB")
    ax[0, 0].axis("off")

    ax[0, 1].imshow(fr["white_raw"], cmap="gray", vmin=0, vmax=255)
    ax[0, 1].set_title(f"white: raw color gate\n{_npx(fr['white_raw'])} px")
    ax[0, 1].axis("off")

    ax[0, 2].imshow(fr["white_filtered"], cmap="gray", vmin=0, vmax=255)
    ax[0, 2].set_title(f"white: horizon-blanked + blob-dropped\n{_npx(fr['white_filtered'])} px")
    ax[0, 2].axis("off")

    ax[0, 3].imshow(fr["white_cloud_mask"], cmap="gray", vmin=0, vmax=255)
    ax[0, 3].set_title(f"white: made it into white_cloud_\n{_npx(fr['white_cloud_mask'])} px")
    ax[0, 3].axis("off")

    ax[1, 1].imshow(fr["yellow_raw"], cmap="gray", vmin=0, vmax=255)
    ax[1, 1].set_title(f"yellow: raw color gate\n{_npx(fr['yellow_raw'])} px")
    ax[1, 1].axis("off")

    ax[1, 2].imshow(fr["yellow_filtered"], cmap="gray", vmin=0, vmax=255)
    ax[1, 2].set_title(f"yellow: horizon-blanked + blob-dropped\n{_npx(fr['yellow_filtered'])} px")
    ax[1, 2].axis("off")

    ax[1, 3].imshow(fr["yellow_cloud_mask"], cmap="gray", vmin=0, vmax=255)
    ax[1, 3].set_title(f"yellow: made it into yellow_cloud_\n{_npx(fr['yellow_cloud_mask'])} px")
    ax[1, 3].axis("off")

    white_xy = np.array(clouds.get("white", []))
    yellow_xy = np.array(clouds.get("yellow", []))
    ax[1, 0].set_title(f"ego-frame cloud (white={len(white_xy)}, yellow={len(yellow_xy)})")
    # x = forward (plotted as screen-up), y = left (plotted as screen-left),
    # matching the project's ego-frame convention -- a driver's-eye top-down view.
    if white_xy.size:
        ax[1, 0].scatter(-white_xy[:, 1], white_xy[:, 0], s=3, c="dimgray", label="white")
    if yellow_xy.size:
        ax[1, 0].scatter(-yellow_xy[:, 1], yellow_xy[:, 0], s=3, c="goldenrod", label="yellow")
    ax[1, 0].scatter([0], [0], marker="^", c="red", s=80, zorder=5, label="ego (base_link)")
    ax[1, 0].set_xlabel("-y  (left of car -> plotted left)")
    ax[1, 0].set_ylabel("x  (forward, m)")
    ax[1, 0].set_aspect("equal")
    ax[1, 0].legend(loc="upper right", fontsize=8)
    ax[1, 0].grid(alpha=0.3)

    fig.tight_layout()
    out_path = os.path.join(dump_dir, f"{seq:05d}_debug.png")
    fig.savefig(out_path, dpi=120)
    print(f"wrote {out_path}")
    plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump_dir")
    ap.add_argument("--seq", type=int, default=None, help="frame index to visualize; default is the latest dumped")
    ap.add_argument("--list", action="store_true", help="print available frame indices and exit")
    args = ap.parse_args()

    seqs = available_seqs(args.dump_dir)
    if not seqs:
        sys.exit(f"no dumped frames found in {args.dump_dir} "
                 "(run lane_node with -p dump_dir:=... -p dump_every_n:=... first)")

    if args.list:
        print(f"{len(seqs)} frame(s) in {args.dump_dir}: {seqs}")
        return

    seq = args.seq if args.seq is not None else seqs[-1]
    if seq not in seqs:
        sys.exit(f"frame {seq} not found; available: {seqs}")

    plot_frame(args.dump_dir, seq)


if __name__ == "__main__":
    main()
