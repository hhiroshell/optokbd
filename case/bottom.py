# ボトムケース（左手）。v0.2.0: plate/plate_left.dxf のプレートに合わせたもの。
# - 座標はプレート DXF と同じ（y は奥が正）。z = 0 が底面の基準、z = 20 がトップケースとの合わせ面
# - 平面形はすべてプレートの外形からのオフセット（layout.py）。作り方は v0.1.0（FreeCAD 版）を踏襲
# - 実行: uv run python bottom.py [--show]   （out/ に STEP / STL / 3MF を書き出す。--show で OCP CAD Viewer に表示）
import math
import sys
from pathlib import Path

from build123d import (
    Axis, Circle, Face, GeomType, Mesher, Plane, Polyline, Pos, Rectangle, Solid, Vector, Wire,
    export_step, export_stl, extrude, revolve,
)

import layout as L
import plate as P
from outline import plan_corners

HEIGHT = 20.0           # ケースの高さ（傾斜カット前）
FLOOR_Z = 14.5          # 内側の床の高さ
RIM_Z = 17.5            # 壁の上端を下げた高さ。タブの下の柱だけ HEIGHT まで残す（トップケースの段差がここに当たる）

# 底面の傾斜（奥が高くなる）: 最も手前の点で TILT_FRONT_Z だけ削り、奥へ TILT_SLOPE で浅くなる
TILT_FRONT_Z = 12.0     # 最も手前で床の厚さ FLOOR_Z - 12 = 2.5 mm を残す
TILT_SLOPE = 12.0 / 114.1721  # 約 6.0°

# 奥の下側の面取り: 奥の辺に沿って、外周で高さ 12、内側へ 6 の三角形を回す
BEVEL_H = 12.0
BEVEL_W = 6.0

FAR = 200.0  # 外形より十分外側


def tilt_z(y, y_front):
    """傾斜カットの高さ（底面からどこまで削るか）。"""
    return TILT_FRONT_Z - TILT_SLOPE * (y - y_front)


def build():
    outer = L.face(L.BOTTOM_OUTER)
    screws = L.screws()

    # 床より下はボスなし（ボスはネジ頭の穴と同じ円なので空になる）、床より上はボスつきの外形
    case = extrude(outer, FLOOR_Z)
    upper = outer + [Pos(s.pos.X, s.pos.Y) * Circle(L.BOSS_R) for s in screws]
    case += Pos(0, 0, FLOOR_Z) * extrude(upper, HEIGHT - FLOOR_Z)
    case = case.clean()

    # 床。左右には壁がない
    case -= Pos(0, 0, FLOOR_Z) * extrude(L.face(L.BOTTOM_FLOOR), HEIGHT)

    # 壁の上端を RIM_Z まで下げる。ガスケットタブの下（柱）だけ残す
    rim = Rectangle(2 * FAR, 2 * FAR)
    for tab in P.tabs():
        rim -= L.tab_strip(tab, L.pillar_half_width(tab))
    case -= Pos(0, 0, RIM_Z) * extrude(rim, HEIGHT)

    # 底面の傾斜。YZ 平面に描いて x 方向に貫通させる
    y_front = outer.bounding_box().min.Y
    y_zero = y_front + TILT_FRONT_Z / TILT_SLOPE
    y_far = y_front - 20
    wedge = Plane.YZ * Polyline(
        (y_far, tilt_z(y_far, y_front)), (y_zero, 0), (y_zero, -FAR), (y_far, -FAR), close=True
    )
    case -= extrude(Face(Wire(wedge.edges())), FAR, both=True)

    # 奥の下側の面取り。三角形を奥の辺の中心軸まわりに回転させて削る。
    # 斜面は外周より外まで延長して、奥の壁と稜線が重ならないようにする
    _, (cx, cy), r = P.BACK
    r += L.BOTTOM_OUTER["edge"]
    slope = BEVEL_H / BEVEL_W
    tri = Plane.XZ * Polyline(
        (r - BEVEL_W, 0), (r + 1, slope * (BEVEL_W + 1)), (r + 1, -FAR), (r - BEVEL_W, -FAR), close=True
    )
    case -= Pos(cx, cy) * revolve(Face(Wire(tri.edges())), Axis.Z)

    # 面取りと左右の側面の境目、手前の角の縦の稜線を R1
    left, right = P.LEFT[1][0], P.RIGHT[1][0]
    cone = case.faces().filter_by(GeomType.CONE)[0]
    sides = [e for e in cone.edges() if min(abs(e.center().X - left), abs(e.center().X - right)) < 1e-6]
    front_corners = [p for p in plan_corners(P.PLAN, L.BOTTOM_OUTER) if p.Y < -30]
    corners = case.edges().filter_by(Axis.Z).filter_by(
        lambda e: any((Vector(e.center().X, e.center().Y) - p).length < 1e-6 for p in front_corners)
    )
    case = case.fillet(1.0, sides + list(corners))

    # 面取りと底面の境目を R0.5
    cone = case.faces().filter_by(GeomType.CONE)[0]
    bottom = case.faces().sort_by(Axis.Z)[0]
    case = case.fillet(0.5, [e for e in cone.edges() if any(e.is_same(b) for b in bottom.edges())])

    # ネジ頭の穴と貫通穴。R1 のフィレットが穴にぶつかると OCC では作れないので、穴はフィレットの後で開ける
    for s in screws:
        case -= Pos(s.pos.X, s.pos.Y, -1) * Solid.make_cylinder(L.BOSS_R, FLOOR_Z + 1)
        case -= Pos(s.pos.X, s.pos.Y, -1) * Solid.make_cylinder(L.HOLE_R, HEIGHT + 2)

    # 奥のネジ頭の穴が面取りに抜けたところを R0.5
    def on_hole(f, p):
        return all(abs(math.hypot(v.X - p.X, v.Y - p.Y) - L.BOSS_R) < 1e-4 for v in f.vertices())

    hole_faces = [
        f for f in case.faces().filter_by(GeomType.CYLINDER)
        if any(on_hole(f, s.pos) for s in screws if s.back)
    ]
    edges = [e for f in hole_faces for e in f.edges() if e.geom_type not in (GeomType.LINE, GeomType.CIRCLE)]
    if edges:
        case = case.fillet(0.5, edges)
    return case


if __name__ == "__main__":
    case = build()
    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    export_step(case, out / "bottom-l.step")
    export_stl(case, out / "bottom-l.stl")
    mesher = Mesher()  # 3D プリンタのスライサー用。単位は mm
    mesher.add_shape(case)
    mesher.write(out / "bottom-l.3mf")
    print(f"volume = {case.volume:.2f} mm^3")
    if "--show" in sys.argv:
        from ocp_vscode import show
        show(case)
