"""Bottom case of the left half.

A build123d rewrite of the v0.1.0 prototype (``v0.1.0/bottom.FCStd``, Body001).
The right half is the mirror image of this part.

Coordinates are in mm and match the FreeCAD model: XY is the layout plane
(+Y points to the back, away from the typist) and +Z points up. Before the
tilt cut the case is a block from z = 0 to z = HEIGHT, and the top case sits
on z = HEIGHT.

Run ``uv run python bottom.py`` to export STEP / STL into ``out/``. Add
``--show`` to also send the part to the OCP CAD Viewer.
"""

import sys
from math import hypot, radians, sqrt, tan
from pathlib import Path

from build123d import (
    Axis,
    Circle,
    GeomType,
    Line,
    Plane,
    Polygon,
    Pos,
    RadiusArc,
    Rectangle,
    Sketch,
    export_step,
    export_stl,
    extrude,
    make_face,
    revolve,
)

HEIGHT = 20.0

# --- Outline ---------------------------------------------------------------
# The outline goes counter-clockwise from the bottom of the right (inner)
# side. The screw bosses (see SCREWS) bulge out of it. For RadiusArc, a
# negative radius puts the centre on the left of the direction of travel.
SIDE_X = 67.825  # the left and right sides are at x = -SIDE_X and +SIDE_X
BACK_R = 308.896  # back edge, centre (0, -266.243)
FRONT_LEFT_R = 203.5  # concave, centre (7.3122, 159.531)
THUMB_R = 179.745  # convex, centre (-6.2286, -222.398)

RIGHT_BOTTOM = (SIDE_X, -27.7716)
FRONT_1 = (-15.7901, -42.6534)  # front-left arc -> straight part
FRONT_2 = (-5.8494, -42.6534)  # straight part -> thumb arc
THUMB_END = (48.6409, -51.2326)  # thumb arc -> straight part
# The top-left, bottom-left and thumb corners lie on the screw bosses there,
# so they are computed in corners().
THUMB_CORNER_APPROX = (59.0981, -54.6303)

# Inside of the walls (the pocket down to the floor): the outline offset
# inwards by WALL along the back and front edges. The left / right sides and
# the slanted side next to the thumb keys are left open (no wall there).
WALL = 2.8
INNER_TOP_Y = 32.2441  # where the inner back arc meets the sides
INNER_LEFT_BOTTOM = (-SIDE_X, -26.5735)
INNER_FRONT_1 = (-15.6301, -39.8534)
INNER_FRONT_2 = (-5.8464, -39.8534)
INNER_THUMB_END = (49.5146, -48.5723)

FLOOR_Z = 14.5

# --- Screws ----------------------------------------------------------------
# The screws go in from the bottom. Each one sits in a boss that bulges out of
# the outline, and the head pocket removes the boss up to SCREW_HEAD_DEPTH.
SCREWS = [
    (45.2441, 38.5128),
    (-10.6426, 41.6691),
    (-65.8622, 34.731),
    (-65.8264, -29.514),
    (-10.5735, -41.8534),
    (57.602, -53.303),
]
SCREW_BOSS_R = 2.0
SCREW_HEAD_D = 4.0
SCREW_HEAD_DEPTH = 14.5
SCREW_HOLE_D = 2.4

# --- Pockets for the top case ----------------------------------------------
# The top of the walls is cut away except at x = -51.65..-26.65 and
# x = 5.5..30.5. Each entry is (x_min, y_min, x_max, y_max).
TOP_CASE_POCKET_DEPTH = 2.5
TOP_CASE_POCKETS = [
    (-70.0, -40.0, -51.6485, 40.0),
    (-26.65, -54.4901, 5.5, 47.1597),
    (30.5015, -56.1449, 70.0, 41.1434),
]

# --- Tilt ------------------------------------------------------------------
# The bottom is cut along a plane so that the back is higher than the front.
# The plane is at z = TILT_FRONT_Z at y = TILT_FRONT_Y and rises towards the
# front by TILT_ANGLE.
TILT_ANGLE = 6.0
TILT_FRONT_Y = -53.5
TILT_FRONT_Z = 12.0

# --- Back chamfer ----------------------------------------------------------
# A triangular cut along the bottom of the back edge, BACK_CHAMFER_W deep and
# BACK_CHAMFER_H high. The arc runs just outside the back wall.
BACK_CHAMFER_CENTER = (0.0, -266.24)
BACK_CHAMFER_R = 308.9
BACK_CHAMFER_W = 6.0
BACK_CHAMFER_H = 12.0

# --- Fillets ---------------------------------------------------------------
CHAMFER_SIDE_FILLET_R = 1.0  # where the chamfer meets the left / right side
THUMB_CORNER_FILLET_R = 1.0  # vertical corner at RIGHT_BOTTOM
BOTTOM_FILLET_R = 0.5  # bottom edge of the chamfer and the sides
CHAMFER_SCREW_FILLET_R = 0.5  # where the chamfer cuts into the screw pockets

TOL = 1e-3


def line_circle(point, direction, center, radius):
    """Points where a line meets a circle, in the order along direction."""
    px, py = point[0] - center[0], point[1] - center[1]
    dx, dy = direction
    a = dx * dx + dy * dy
    b = px * dx + py * dy
    d = sqrt(b * b - a * (px * px + py * py - radius * radius))
    return [
        (point[0] + t * dx, point[1] + t * dy) for t in ((-b - d) / a, (-b + d) / a)
    ]


def line_line(p0, d0, p1, d1):
    """Point where two lines meet."""
    t = ((p1[0] - p0[0]) * d1[1] - (p1[1] - p0[1]) * d1[0]) / (
        d0[0] * d1[1] - d0[1] * d1[0]
    )
    return (p0[0] + t * d0[0], p0[1] + t * d0[1])


def corners():
    """The corners that lie on the screw bosses."""
    left_top = line_circle((-SIDE_X, 0), (0, 1), SCREWS[2], SCREW_BOSS_R)[1]
    left_bottom = line_circle((-SIDE_X, 0), (0, 1), SCREWS[3], SCREW_BOSS_R)[0]
    thumb_dir = (
        THUMB_CORNER_APPROX[0] - THUMB_END[0],
        THUMB_CORNER_APPROX[1] - THUMB_END[1],
    )
    thumb_corner = line_circle(THUMB_END, thumb_dir, SCREWS[5], SCREW_BOSS_R)[1]
    return left_top, left_bottom, thumb_corner


def side_clip():
    """The area between the left side and the right / slanted sides, extended
    far to the back and the front. The screw bosses are cut off at the sides.
    """
    far = 100.0
    _, _, thumb_corner = corners()
    (x0, y0), (x1, y1) = thumb_corner, RIGHT_BOTTOM
    x_far = x0 + (x1 - x0) * (-far - y0) / (y1 - y0)
    return Polygon(
        (-SIDE_X, far),
        (-SIDE_X, -far),
        (x_far, -far),
        RIGHT_BOTTOM,
        (SIDE_X, far),
        align=None,
    )


def screw_circles(radius):
    return [Pos(x, y) * Circle(radius) for x, y in SCREWS]


def outer_outline():
    left_top, left_bottom, thumb_corner = corners()
    right_top = (SIDE_X, left_top[1])
    base = make_face(
        [
            Line(RIGHT_BOTTOM, right_top),
            RadiusArc(right_top, left_top, -BACK_R),
            Line(left_top, left_bottom),
            RadiusArc(left_bottom, FRONT_1, -FRONT_LEFT_R),
            Line(FRONT_1, FRONT_2),
            RadiusArc(FRONT_2, THUMB_END, THUMB_R),
            Line(THUMB_END, thumb_corner),
            Line(thumb_corner, RIGHT_BOTTOM),
        ]
    )
    bosses = Sketch() + screw_circles(SCREW_BOSS_R)
    return base + (bosses & side_clip())


def inner_outline():
    _, _, thumb_corner = corners()
    right_top = (SIDE_X, INNER_TOP_Y)
    left_top = (-SIDE_X, INNER_TOP_Y)
    # The offset of the thumb straight part runs into the slanted side.
    corner = line_line(
        INNER_THUMB_END,
        (thumb_corner[0] - THUMB_END[0], thumb_corner[1] - THUMB_END[1]),
        thumb_corner,
        (RIGHT_BOTTOM[0] - thumb_corner[0], RIGHT_BOTTOM[1] - thumb_corner[1]),
    )
    return make_face(
        [
            Line(RIGHT_BOTTOM, right_top),
            RadiusArc(right_top, left_top, -(BACK_R - WALL)),
            Line(left_top, INNER_LEFT_BOTTOM),
            RadiusArc(INNER_LEFT_BOTTOM, INNER_FRONT_1, -(FRONT_LEFT_R - WALL)),
            Line(INNER_FRONT_1, INNER_FRONT_2),
            RadiusArc(INNER_FRONT_2, INNER_THUMB_END, THUMB_R + WALL),
            Line(INNER_THUMB_END, corner),
            Line(corner, RIGHT_BOTTOM),
        ]
    )


def top_case_pockets():
    return Sketch() + [
        Pos((x0 + x1) / 2, (y0 + y1) / 2) * Rectangle(x1 - x0, y1 - y0)
        for x0, y0, x1, y1 in TOP_CASE_POCKETS
    ]


def tilt_cutter():
    back_y = TILT_FRONT_Y + TILT_FRONT_Z / tan(radians(TILT_ANGLE))
    section = Polygon(
        (TILT_FRONT_Y, 0),
        (TILT_FRONT_Y, TILT_FRONT_Z),
        (back_y, 0),
        align=None,
    )
    return extrude(Plane.YZ * section, amount=200, both=True)


def back_chamfer_cutter():
    # Revolve the triangular section around the centre of the back arc.
    cx, cy = BACK_CHAMFER_CENTER
    section = Polygon(
        (BACK_CHAMFER_R, 0),
        (BACK_CHAMFER_R, BACK_CHAMFER_H),
        (BACK_CHAMFER_R - BACK_CHAMFER_W, 0),
        align=None,
    )
    plane = Plane(origin=(cx, cy, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
    return revolve(plane * section, Axis((cx, cy, 0), (0, 0, 1)))


def on_plane_x(edge, x):
    return all(abs(p.X - x) < TOL for p in (edge @ 0, edge @ 0.5, edge @ 1))


def on_screw_head(edge):
    r = SCREW_HEAD_D / 2
    return any(
        all(
            abs(hypot(p.X - x, p.Y - y) - r) < TOL
            for p in (edge @ 0, edge @ 0.5, edge @ 1)
        )
        for x, y in SCREWS
    )


def chamfer_edges(part):
    return [e for f in part.faces().filter_by(GeomType.CONE) for e in f.edges()]


def build_bottom():
    top = Plane.XY.offset(HEIGHT)

    bottom = extrude(outer_outline(), amount=HEIGHT)
    # Inside down to the floor, and the pockets for the top case.
    bottom -= extrude(top * inner_outline(), amount=-(HEIGHT - FLOOR_Z))
    bottom -= extrude(top * top_case_pockets(), amount=-TOP_CASE_POCKET_DEPTH)
    bottom -= tilt_cutter()
    bottom -= back_chamfer_cutter()

    # Where the chamfer meets the left / right sides, and the vertical corner
    # next to the thumb keys. This is done before the screw head pockets are
    # cut: OCC fails to fillet the left side once the pocket at the top-left
    # corner meets it at a shallow angle.
    side_edges = [
        e
        for e in chamfer_edges(bottom)
        if on_plane_x(e, -SIDE_X) or on_plane_x(e, SIDE_X)
    ]
    corner_x, corner_y = RIGHT_BOTTOM
    corner_edges = [
        e
        for e in bottom.edges().filter_by(Axis.Z)
        if abs((e @ 0).X - corner_x) < TOL and abs((e @ 0).Y - corner_y) < TOL
    ]
    bottom = bottom.fillet(CHAMFER_SIDE_FILLET_R, side_edges)
    bottom = bottom.fillet(THUMB_CORNER_FILLET_R, corner_edges)

    # Screw head pockets from the bottom, then the screw holes.
    bottom -= extrude(Sketch() + screw_circles(SCREW_HEAD_D / 2), SCREW_HEAD_DEPTH)
    bottom -= extrude(Sketch() + screw_circles(SCREW_HOLE_D / 2), HEIGHT)

    # Where the chamfer meets the tilted bottom. The fillet carries on along
    # the edges that are now tangent to it: the bottom of the left side, the
    # right side and the slanted side.
    tilted = bottom.faces().filter_by(GeomType.PLANE).sort_by(Axis.Z)[0]
    bottom_edges = [e for e in chamfer_edges(bottom) if e in tilted.edges()]
    bottom = bottom.fillet(BOTTOM_FILLET_R, bottom_edges)

    # Where the chamfer cuts into the screw head pockets.
    screw_edges = [e for e in chamfer_edges(bottom) if on_screw_head(e)]
    bottom = bottom.fillet(CHAMFER_SCREW_FILLET_R, screw_edges)

    return bottom


bottom = build_bottom()

if __name__ == "__main__":
    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    export_step(bottom, out / "bottom-left.step")
    export_stl(bottom, out / "bottom-left.stl")

    if "--show" in sys.argv:
        from ocp_vscode import show

        show(bottom)
