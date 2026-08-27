"""generate_sign_textures.py - self-contained sign board assets for urban_gazebo.

The original v2 plan reused the v1 sign meshes/textures from robotcar_gazebo
(model://sign_<kind>/meshes/sign.obj). Building from an empty workspace there
is no v1 package, so this script generates equivalent assets from scratch.
For each kind in {left, right, winding, stop} it writes:

    src/urban_gazebo/models/sign_<kind>_urban/meshes/sign.png   (512x512 face)
    src/urban_gazebo/models/sign_<kind>_urban/meshes/sign.obj   (1x1 m quad)
    src/urban_gazebo/models/sign_<kind>_urban/meshes/sign.mtl

The quad lies in the Y-Z plane with its normal along +X; the model yaw set by
generate_course_world.py turns +X toward the approaching car, and the mesh
<scale> there blows the 1 m quad up to the 1.8 m urban board.
Faces are drawn to match the classifier prompt (vlm_prompts.SIGN_PROMPT):
black bent arrow (left/right), black S-shaped double-bend arrow (winding),
white STOP on a red octagon (stop). Verify Qwen reads your rendering with the
Day-12 offline probe before trusting it in the loop.

Usage:
  python3 scripts/generate_sign_textures.py
"""
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

WS = Path(__file__).resolve().parents[1]
SIZE = 512          # texture resolution
LINE_W = 46         # arrow stroke width, px


def _board():
    """White board with a black frame (the arrow signs)."""
    img = Image.new("RGB", (SIZE, SIZE), "white")
    d = ImageDraw.Draw(img)
    d.rectangle([8, 8, SIZE - 9, SIZE - 9], outline="black", width=14)
    return img, d


def _arrow_head(d, tip, direction, size=95):
    """Filled triangular head at `tip` pointing along unit vector `direction`."""
    dx, dy = direction
    px, py = -dy, dx                       # perpendicular
    base = (tip[0] - dx * size, tip[1] - dy * size)
    d.polygon([tip,
               (base[0] + px * size * 0.62, base[1] + py * size * 0.62),
               (base[0] - px * size * 0.62, base[1] - py * size * 0.62)],
              fill="black")


def draw_turn(kind):
    """90-degree bent arrow: up from the bottom, bend out left/right."""
    img, d = _board()
    s = 1 if kind == "right" else -1       # image x grows to the RIGHT
    cx = SIZE // 2
    x_out = cx + s * 120
    d.line([(cx, 430), (cx, 185), (x_out, 185)],
           fill="black", width=LINE_W, joint="curve")
    _arrow_head(d, (x_out + s * 55, 185), (s, 0))
    return img


def draw_winding():
    """S-shaped (double-bend) wavy arrow, tip pointing up."""
    img, d = _board()
    cx = SIZE // 2
    pts = []
    for i in range(25):
        t = i / 24.0
        y = 435 - 330 * t
        x = cx + 85 * math.sin(2 * math.pi * 0.75 * t)   # 1.5 bends = S
        pts.append((x, y))
    d.line(pts, fill="black", width=LINE_W, joint="curve")
    _arrow_head(d, (pts[-1][0], pts[-1][1] - 45), (0, -1))
    return img


def draw_stop():
    """White STOP on a red octagon with a white rim."""
    img = Image.new("RGB", (SIZE, SIZE), "white")
    d = ImageDraw.Draw(img)
    c, r = SIZE / 2, SIZE / 2 - 14
    octagon = [(c + r * math.cos(math.radians(22.5 + 45 * k)),
                c + r * math.sin(math.radians(22.5 + 45 * k))) for k in range(8)]
    d.polygon(octagon, fill=(200, 20, 25), outline="white", width=16)
    try:
        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 150)
    except OSError:
        font = ImageFont.load_default()
    bbox = d.textbbox((0, 0), "STOP", font=font)
    d.text((c - (bbox[2] - bbox[0]) / 2, c - (bbox[3] - bbox[1]) / 2 - bbox[1]),
           "STOP", fill="white", font=font)
    return img


# 1x1 m quad in the Y-Z plane, normal +X. UVs chosen so a viewer standing on
# +X looking back at the face (the approaching car) sees the texture upright
# and un-mirrored: image-u grows along +Y (the viewer's right), image-v top
# maps to +Z.
#
# Material name is per-kind (sign_<kind>, not a shared "sign"): Gazebo's
# renderer caches loaded materials by name, so if every model declared the
# same material name, whichever sign model loaded first would "win" and
# every other board would silently render its texture instead of its own.
def obj_text(kind):
    return f"""mtllib sign.mtl
v 0 -0.5 -0.5
v 0  0.5 -0.5
v 0  0.5  0.5
v 0 -0.5  0.5
vt 0 0
vt 1 0
vt 1 1
vt 0 1
vn 1 0 0
usemtl sign_{kind}
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
"""


def mtl_text(kind):
    # texture filename is also per-kind (sign_<kind>.png, not shared "sign.png"):
    # some loaders key cached textures by filename alone, so a shared name can
    # collide the same way a shared material name does.
    return f"""newmtl sign_{kind}
Ka 1 1 1
Kd 1 1 1
map_Kd sign_{kind}.png
"""


def main():
    draws = {"left": lambda: draw_turn("left"),
             "right": lambda: draw_turn("right"),
             "winding": draw_winding,
             "stop": draw_stop}
    for kind, fn in draws.items():
        mdir = WS / "src/urban_gazebo/models" / f"sign_{kind}_urban" / "meshes"
        mdir.mkdir(parents=True, exist_ok=True)
        fn().save(mdir / f"sign_{kind}.png")
        (mdir / "sign.obj").write_text(obj_text(kind), encoding="utf-8")
        (mdir / "sign.mtl").write_text(mtl_text(kind), encoding="utf-8")
        print(f"wrote {mdir}/sign.obj, sign.mtl, sign_{kind}.png")


if __name__ == "__main__":
    main()