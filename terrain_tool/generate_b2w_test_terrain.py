"""Generate a B2W evaluation terrain: several uniform staircase blocks + several slope blocks.

Stairs:  one block per step height (5, 10, 15, 16, 18, 20 cm). Every step inside a block is the
         same height; each block is a separate flight in its own lane. Tread width is 0.275 m,
         matching `step_width` in the B2W staircase teacher
         (robot_lab .../unitree_b2w/staircaseup_teacher_env_cfg.py). After climbing up, each
         flight is joined at its top by a mirrored flight that descends back to the ground, so
         the robot drives up then down a symmetric pyramid in one pass.
Slopes:  one block per angle, spanning the training `slope_range=(0.0, 0.50)` rad of the B2W
         slopeup teacher (.../unitree_b2w/slopeup_teacher_env_cfg.py). Each block is an up ramp
         joined at its peak by a mirrored down ramp, forming a symmetric A-frame ridge.

Layout: staircase lanes occupy +Y, slope lanes occupy -Y, all climbing in +X from near x=1.5,
        then descending back to the ground further along +X, so the robot can drive forward down
        any lane to test one height/grade up-and-down in isolation.

Reuses the existing TerrainGenerator.AddStairs / AddBox primitives.

Run from the terrain_tool directory:
    cd terrain_tool && python3 generate_b2w_test_terrain.py
Then start the C++ simulator (simulate/config.yaml already points at scene_terrain.xml).
"""

import sys
import types

import numpy as np

# terrain_generator imports `noise` at module top for its Perlin helper, which we do not use
# here. Stub it so this script runs without the (compiler-dependent) `noise` package installed.
if "noise" not in sys.modules:
    try:
        import noise  # noqa: F401
    except ImportError:
        sys.modules["noise"] = types.ModuleType("noise")

import terrain_generator as tgen

# build the b2w scene (include b2w.xml), not the default go2 template scene
tgen.ROBOT = "b2w"
tgen.INPUT_SCENE_PATH = "../unitree_robots/b2w/scene.xml"
tgen.OUTPUT_SCENE_PATH = "../unitree_robots/b2w/scene_terrain.xml"


def add_slope(tg, init_x, y, slope_rad, length=3.0, width=1.0, thickness=0.1):
    """A single constant-grade ramp climbing in +X, low edge resting on the ground (reuses AddBox).

    init_x: x of the ramp's low (ground) edge; y: lane center; slope_rad: grade in radians.
    Returns the x of the ramp's high (peak) edge so a descending ramp can be joined onto it.
    """
    # lift the center so the low edge sits at z=0, and place the center past init_x by half the run
    center_x = init_x + 0.5 * length * np.cos(slope_rad)
    center_z = 0.5 * length * np.sin(slope_rad) + 0.5 * thickness * np.cos(slope_rad)
    tg.AddBox(position=[center_x, y, center_z],
              euler=[0.0, -slope_rad, 0.0],
              size=[length, width, thickness])
    return init_x + length * np.cos(slope_rad) + 0.5 * thickness * np.sin(slope_rad)


def add_slope_down(tg, peak_x, y, slope_rad, length=3.0, width=1.0, thickness=0.1):
    """A mirror of add_slope that descends in +X, its high edge butting onto peak_x (reuses AddBox).

    peak_x: x of the high edge to join (the value returned by add_slope); the ramp then drops back
    to z=0 over one run length. Same center_z as the up ramp, but pitched +slope_rad so +X is down.
    """
    center_x = peak_x + 0.5 * length * np.cos(slope_rad) + 0.5 * thickness * np.sin(slope_rad)
    center_z = 0.5 * length * np.sin(slope_rad) + 0.5 * thickness * np.cos(slope_rad)
    tg.AddBox(position=[center_x, y, center_z],
              euler=[0.0, slope_rad, 0.0],
              size=[length, width, thickness])


if __name__ == "__main__":
    tg = tgen.TerrainGenerator()

    init_x = 1.5        # x where every lane starts climbing
    lane_dy = 1.3       # spacing between lanes (m)

    # --- Staircase blocks: one uniform-height flight per step height, in +Y lanes ---
    step_heights = [0.05, 0.10, 0.15, 0.16, 0.18, 0.20]
    stair_nums = 4      # steps per block
    tread = 0.275       # tread depth (matches training step_width)
    stair_len = 1.0     # step length across the climb direction
    for i, h in enumerate(step_heights):
        y = 0.75 + i * lane_dy
        tg.AddStairs(init_pos=[init_x, y, 0.0], yaw=0.0,
                     width=tread, height=h, length=stair_len, stair_nums=stair_nums)
        # Mirror flight descending in +X (yaw=pi flips the climb direction), its top step butted
        # right against the up flight's top step so the robot crests one pyramid and walks down.
        down_x0 = init_x + (2 * stair_nums + 1) * tread
        tg.AddStairs(init_pos=[down_x0, y, 0.0], yaw=np.pi,
                     width=tread, height=h, length=stair_len, stair_nums=stair_nums)

    # --- Slope blocks: one constant grade per block, in -Y lanes, spanning 0..0.50 rad ---
    slope_rads = [0.10, 0.20, 0.30, 0.40, 0.50]
    for j, s in enumerate(slope_rads):
        y = -0.75 - j * lane_dy
        peak_x = add_slope(tg, init_x=init_x, y=y, slope_rad=s, length=3.0, width=1.0)
        add_slope_down(tg, peak_x=peak_x, y=y, slope_rad=s, length=3.0, width=1.0)

    tg.Save()
    print(f"Wrote {tgen.OUTPUT_SCENE_PATH}")
    print(f"  staircase blocks ({stair_nums} steps up + {stair_nums} steps down, tread {tread} m): "
          f"{['%.0fcm' % (h * 100) for h in step_heights]}")
    print(f"  slope blocks (up ramp + down ramp): "
          f"{['%.0f deg (%.2f rad)' % (np.degrees(s), s) for s in slope_rads]}")