# トップケース（左手）。v0.2.0: plate/plate_left.dxf のプレートに合わせたもの。
# - 座標はプレート DXF と同じ（y は奥が正）。z = 0 が下端、z = 23 が上面
# - 平面形はすべてプレートの外形からのオフセット（layout.py）。作り方は v0.1.0（FreeCAD 版）を踏襲
# - 完全無線化に合わせて、v0.1.0 にあった Pro Micro の切り欠き・コネクタのポケット・TRRS の穴はない
# - 実行: uv run python top.py [--show]   （out/ に STEP / STL / 3MF を書き出す。--show で OCP CAD Viewer に表示）
import math
import sys
from pathlib import Path

from build123d import (
    Axis, Circle, Face, Kind, Mesher, Plane, Polyline, Pos, RectangleRounded, Rot, Solid, Vector, Wire,
    export_step, export_stl, extrude, offset,
)

import bottom as B
import layout as L
import plate as P
from outline import on_curve, path_face, plan_corners, plan_curves

HEIGHT = 23.0
CAVITY_TOP = 19.0       # 内側の空間（ボトムケースの床の上）の高さ。ここから上面までがベゼル
INNER_TOP = 11.0        # 外壁の内面（ボトムケースを囲む部分）の高さ。ここが段差になり、ボトムケースの下げた壁の上端が当たる
SLOT_TOP = 20.0         # ガスケットのポケットの高さ
INNER_BOSS_R = 2.4      # ボトムケースのボスを逃がす円
CHAMFER = 3.0           # 上面の外周（奥、手前左、右下の直線）の面取り

NUT_Z = (15.0, 16.8)    # ナットのポケットの高さ（段差 z = 11 から 4 mm 上。ナットが抜け落ちないよう、上下に肉を残す）
NUT_AF = 4.2            # ナットのポケットの二面幅（M2 ナット 4.0 mm + 0.2）
HOLE_TOP = 18.0         # ネジ穴の深さ（ナットの上に 1.2 mm）

BEZEL_MARGIN = 0.6      # キーの開口部: 1U の枠から外へ（18 mm のキーキャップとは 1.125 mm）。角は R0.6
BEZEL_ROUND = 0.5       # キーの開口部の内角の丸め

# 下端の切り欠き: 手前はボトムケースの傾斜した底面に合わせ、奥は円弧で上げる（ボトムケースの奥の面取りを逃がす）
WALL_CUT_BACK_Z = B.BEVEL_H - L.Z_OFFSET + 0.5   # 奥の端の高さ（6.0）

FAR = 200.0  # 外形より十分外側


def bezel():
    """キーの開口部: 各キーの 1U の枠を BEZEL_MARGIN だけオフセットした形（直線と R）の和集合の外周。
    親指キーの下辺は、プレートの親指の下の円弧から BEZEL_MARGIN 外の円弧（各キーの下辺の中点に接する）。
    内角は丸め、キーの間に囲まれたすき間（行と親指の列の間など）は開口部に含める（上面に浮いた島が残らないように）。"""
    m = BEZEL_MARGIN
    keys = P.keys()
    opening = None
    for k in keys:
        sq = Pos(k.center.X, k.center.Y) * Rot(0, 0, k.angle) * RectangleRounded(19.05 + 2 * m, 19.05 + 2 * m, m)
        opening = sq if opening is None else opening + sq
    opening += thumb_band(keys)
    opening = offset(opening, BEZEL_ROUND, kind=Kind.ARC)
    opening = offset(opening, -BEZEL_ROUND, kind=Kind.ARC)
    return Face(opening.faces()[0].outer_wire())


def thumb_band(keys):
    """親指キーの下辺の円弧と、キーの中心の円弧の間の帯（隣り合うキーの下の角の間の V 字のすき間を埋める）。
    親指キーはプレートの親指の下の円弧の中心のまわりに並んでいて、下辺の中点がその円弧の上にある。"""
    (cx, cy), r = P.THUMB[1:]
    c = Vector(cx, cy)
    thumbs = [k for k in keys if abs((k.center - c).length - (r + 9.525)) < 0.05]
    angles = []
    for k in thumbs:
        rot = math.radians(k.angle if k.angle < 45 else k.angle - 90)
        mid = k.center - Vector(-math.sin(rot), math.cos(rot)) * 9.525   # 下辺の中点
        angles.append(math.atan2(mid.Y - cy, mid.X - cx))
    a0, a1 = min(angles), max(angles)
    r_out, r_in = r - BEZEL_MARGIN, r + 9.525

    def at(rad, a):
        return (cx + rad * math.cos(a), cy + rad * math.sin(a))

    arc_out, arc_in = ("arc", (cx, cy), r_out), ("arc", (cx, cy), r_in)
    return path_face([(at(r_out, a0), arc_out), (at(r_out, a1), None), (at(r_in, a1), arc_in), (at(r_in, a0), None)])


def nut_pocket(screw):
    """ナットのポケット。平らな面を外（壁側）へ向けた六角形の外側半分と、内側へナットを入れるための溝。"""
    r = NUT_AF / math.sqrt(3)
    n = screw.normal
    t = Vector(-n.Y, n.X)
    local = [(r, 0), (r / 2, NUT_AF / 2), (-r / 2, NUT_AF / 2), (-r, 0), (-r, -NUT_AF / 2), (r, -NUT_AF / 2)]
    pts = [screw.pos + t * a + n * b for a, b in local]
    return Face(Wire(Polyline(*[(p.X, p.Y) for p in pts], close=True).edges()))


def wall_cut():
    """下端の切り欠きの断面（YZ 平面上 (y, z)）。"""
    y_front = L.face(L.BOTTOM_OUTER).bounding_box().min.Y   # ボトムケースの最も手前
    y0 = y_front + (B.TILT_FRONT_Z - L.Z_OFFSET) / B.TILT_SLOPE   # ボトムケースの底面がトップケースの z = 0 を横切る位置
    bb = L.face(L.TOP_OUTER).bounding_box()
    y_top, h = bb.max.Y, WALL_CUT_BACK_Z
    radius = ((y_top - y0) ** 2 + h * h) / (2 * h)  # (y0, 0) を通り、(y_top, h) で水平になる円弧
    y_ff = bb.min.Y - 20
    z_ff = B.tilt_z(y_ff, y_front) - L.Z_OFFSET
    return path_face([
        ((y_ff, z_ff), None),
        ((y0, 0.0), ("arc", (y_top, h - radius), radius)),
        ((y_top, h), None),
        ((y_top + 20, h), None),
        ((y_top + 20, -1.0), None),
        ((y_ff, -1.0), None),
    ])


def build():
    screws = L.screws()
    case = extrude(L.face(L.TOP_OUTER), HEIGHT)

    # 内側: ボトムケースの床の上の空間、外壁の内面（ボスの逃げつき）、ガスケットのポケット
    case -= Pos(0, 0, -1) * extrude(L.face(L.TOP_CAVITY), CAVITY_TOP + 1)
    inner = L.face(L.TOP_INNER) + [Pos(s.pos.X, s.pos.Y) * Circle(INNER_BOSS_R) for s in screws]
    case -= Pos(0, 0, -1) * extrude(inner, INNER_TOP + 1)
    ring = L.face(L.TOP_INNER) - L.face(L.TOP_CAVITY)
    for tab in P.tabs():
        case -= Pos(0, 0, -1) * extrude(ring & L.tab_strip(tab, L.slot_half_width(tab)), SLOT_TOP + 1)

    # キーの開口部。上面の板（CAVITY_TOP から上）だけを抜く。キーに対するオフセットで描いているので、
    # プレートの外形がキーの角を切る所（R3C3 の左下、R0C3 の上）では内側の空間よりわずかに外へ出る
    case -= Pos(0, 0, CAVITY_TOP) * extrude(bezel(), HEIGHT - CAVITY_TOP + 1)

    # ナットのポケットとネジ穴
    for s in screws:
        case -= Pos(0, 0, NUT_Z[0]) * extrude(nut_pocket(s), NUT_Z[1] - NUT_Z[0])
        case -= Pos(s.pos.X, s.pos.Y, -1) * Solid.make_cylinder(L.HOLE_R, HOLE_TOP + 1)

    # 下端の切り欠き。x 方向に貫通させる
    case -= extrude(Plane.YZ * wall_cut(), FAR, both=True)

    # 上面の外周の面取り（奥、手前左、右下の直線）
    curves = plan_curves(P.PLAN, L.TOP_OUTER)
    targets = [curves[i] for i in (P.I_BACK, P.I_FRONT_LEFT, P.I_THUMB_LINE)]
    top = case.faces().filter_by(Axis.Z).sort_by(Axis.Z)[-1]
    edges = [e for e in top.edges() if any(on_curve(e.center(), c) for c in targets)]
    case = case.chamfer(CHAMFER, None, edges)

    # 外周の縦の角と、そこにつながる面取りの端を R1
    corners = plan_corners(P.PLAN, L.TOP_OUTER)

    def at(p):
        return any((Vector(p.X, p.Y) - c).length < 1e-3 for c in corners)

    edges = []
    for e in case.edges():
        vs = sorted(e.vertices(), key=lambda v: v.Z)
        low, high = vs[0], vs[-1]
        if high.Z - low.Z < 1 or not at(low):
            continue
        vertical = (Vector(low.X, low.Y) - Vector(high.X, high.Y)).length < 1e-6
        chamfer_end = abs(low.Z - (HEIGHT - CHAMFER)) < 1e-3 and abs(high.Z - HEIGHT) < 1e-3
        if vertical or chamfer_end:
            edges.append(e)
    case = case.fillet(1.0, edges)
    return case


if __name__ == "__main__":
    case = build()
    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    export_step(case, out / "top-l.step")
    export_stl(case, out / "top-l.stl")
    mesher = Mesher()  # 3D プリンタのスライサー用。単位は mm
    mesher.add_shape(case)
    mesher.write(out / "top-l.3mf")
    print(f"volume = {case.volume:.2f} mm^3")
    if "--show" in sys.argv:
        from ocp_vscode import show
        show(case)
