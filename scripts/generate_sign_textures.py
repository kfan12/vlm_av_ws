"""generate_sign_textures.py - self-contained sign board assets for urban_gazebo.

The original v2 plan reused the v1 sign meshes/textures from robotcar_gazebo
(model://sign_<kind>/meshes/sign.obj). Building from an empty workspace there
is no v1 package, so this script generates equivalent assets from scratch.

The sign faces are now real reference artwork dropped in under

    src/urban_gazebo/models/pngs/{left,right,winding,stop}.png

(proper MUTCD-style boards: yellow diamond with a black bent arrow for
left/right, yellow diamond with a black S-shaped double-bend arrow for winding,
white STOP on a red octagon for stop). This script normalises each one into a
square face texture and writes, for each kind in {left, right, winding, stop}:

    src/urban_gazebo/models/sign_<kind>_urban/meshes/sign_<kind>.png  (512x512 face)
    src/urban_gazebo/models/sign_<kind>_urban/meshes/sign.obj         (1x1 m quad)
    src/urban_gazebo/models/sign_<kind>_urban/meshes/sign.mtl

The source art has transparent corners; the mesh is a full square quad, so the
corners are composited onto white (matching how the reference boards are framed
and how the Day-12 classifier was validated).

The quad lies in the Y-Z plane with its normal along +X; the model yaw set by
generate_course_world.py turns +X toward the approaching car, and the mesh
<scale> there (SIGN_BOARD_M) blows the 1 m quad up to the urban board.

Faces must match the classifier prompt (vlm_prompts.SIGN_PROMPT). Verify Qwen
reads the rendering with the Day-12 offline probe before trusting it in the
loop:

  python3 scripts/probe_sign_texture.py \\
      src/urban_gazebo/models/sign_left_urban/meshes/sign_left.png

Usage:
  python3 scripts/generate_sign_textures.py
"""
from pathlib import Path

from PIL import Image

WS = Path(__file__).resolve().parents[1]
PNG_DIR = WS / "src/urban_gazebo/models/pngs"
SIZE = 512          # output texture resolution
KINDS = ("left", "right", "winding", "stop")


def face(kind):
    """Load pngs/<kind>.png, flatten its transparent corners onto white, and
    return a square SIZE x SIZE RGB face texture."""
    src = Image.open(PNG_DIR / f"{kind}.png").convert("RGBA")
    canvas = Image.new("RGBA", src.size, (255, 255, 255, 255))
    canvas.alpha_composite(src)
    return canvas.convert("RGB").resize((SIZE, SIZE), Image.LANCZOS)


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
    for kind in KINDS:
        mdir = WS / "src/urban_gazebo/models" / f"sign_{kind}_urban" / "meshes"
        mdir.mkdir(parents=True, exist_ok=True)
        face(kind).save(mdir / f"sign_{kind}.png")
        (mdir / "sign.obj").write_text(obj_text(kind), encoding="utf-8")
        (mdir / "sign.mtl").write_text(mtl_text(kind), encoding="utf-8")
        print(f"wrote {mdir}/sign.obj, sign.mtl, sign_{kind}.png")


if __name__ == "__main__":
    main()
