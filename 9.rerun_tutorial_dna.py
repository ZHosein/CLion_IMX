import rerun as rr

from math import tau
import numpy as np
from rerun.utilities import build_color_spiral
from rerun.utilities import bounce_lerp


def line():
    rr.init("rerun line example", spawn=True)
    positions = np.zeros((10, 3))
    positions[:, 0] = np.linspace(-10, 10, 10)

    colors = np.zeros((10, 3), dtype=np.uint8)
    colors[:, 0] = np.linspace(0, 255, 10)

    rr.log(
        "line_points",
        rr.Points3D(positions, colors=colors, radii=0.5)
    )


def cube():
    rr.init("rerun cube example", spawn=True)

    SIZE = 10

    pos_grid = np.meshgrid(*[np.linspace(-10, 10, SIZE)] * 3)
    positions = np.vstack([d.reshape(-1) for d in pos_grid]).T

    col_grid = np.meshgrid(*[np.linspace(0, 255, SIZE)] * 3)
    colors = np.vstack([c.reshape(-1) for c in col_grid]).astype(np.uint8).T

    rr.log(
        "cube_points",
        rr.Points3D(positions, colors=colors, radii=0.5)
    )


def dna_example():
    rr.init("rerun_dna_example")
    rr.spawn()
    rr.set_time_seconds("stable_time", 0)

    NUM_P0INTS = 100

    # Points and colors are both np.array((NUM_POINTS, 3))
    points1, colors1 = build_color_spiral(NUM_P0INTS)
    points2, colors2 = build_color_spiral(NUM_P0INTS, angular_offset=tau * 0.5)

    rr.log("dna/structure/left", rr.Points3D(points1, colors=colors1, radii=0.08))
    rr.log("dna/structure/right", rr.Points3D(points2, colors=colors2, radii=0.08))

    rr.log(
        "dna/structure/scaffolding",
        rr.LineStrips3D(np.stack((points1, points2), axis=1), colors=[128, 128, 128])
    )

    time_offsets = np.random.rand(NUM_P0INTS)
    for i in range(400):
        time = i * 0.01
        rr.set_time_seconds("stable_time", time)

        times = np.repeat(time, NUM_P0INTS) + time_offsets
        beads = [bounce_lerp(points1[n], points2[n], times[n]) for n in range(NUM_P0INTS)]
        colors = [[int(bounce_lerp(80, 230, times[n] * 2))] for n in range(NUM_P0INTS)]
        rr.log(
            "dna/structure/scaffolding/beads",
            rr.Points3D(beads, radii=0.06, colors=np.repeat(colors, 3, axis=1))
        )

    # rotating the spiral; could have also been done within the above loop
    for i in range(400):
        time = i * 0.01
        rr.set_time_seconds("stable_time", time)
        rr.log(
            "dna/structure",
            rr.Transform3D(rotation=rr.RotationAxisAngle(axis=[0, 0, 1], radians=time / 4.0 * tau))
        )


if __name__ == "__main__":
    dna_example()
