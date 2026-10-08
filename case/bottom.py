# ボトムケース（左手）。v0.1.0/bottom.FCStd の Body001 を build123d で書き直したもの。
# - 座標: XY がケースの平面（y は奥が正）、z は上向き。z = 0 が底面の基準、z = 20 がトップケースとの合わせ面
# - FreeCAD 版はミラーした右手用も同じファイルに置いていたが、ここでは左手だけを作る
# - 実行: uv run python bottom.py [--show]   （out/ に STEP / STL / 3MF を書き出す。--show で OCP CAD Viewer に表示）
import math
import sys
from pathlib import Path

from build123d import (
    Axis, Face, GeomType, Plane, Polyline, Pos, Rectangle, Solid, Vector, Wire,
    Mesher, export_step, export_stl, extrude, revolve,
)

from outline import (
    BACK_ARC, FRONT, HALF_W, LEFT, LEFT_ARC, RIGHT, SCREWS, SLANT, THUMB_ARC, THUMB_LINE, chain_face, plan,
)

HEIGHT = 20.0           # ケースの高さ（傾斜カット前）
FLOOR_Z = 14.5          # 内側の床の高さ（= 上から 5.5 mm 掘り下げ）
TOP_POCKET_DEPTH = 2.5  # トップケースを受ける切り欠きの深さ
WALL = 2.8              # 奥と手前の壁の厚さ（左右の側面には壁がない）

# ネジのボスは外形から半円状に張り出していて、半径はナット穴と同じ
NUT_R = 2.0    # 底面から床の高さまで掘るナット（ネジ頭）の穴。ボスの半径も同じ
HOLE_R = 1.2   # 貫通穴
BOSS = [("arc", c, NUT_R) for c in SCREWS]

# ボスつきの外形: [(辺, 始点の目安), ...]。始点は前の辺との交点のうち、目安に近い方
OUTLINE = [
    (RIGHT, (67.825, -27.7716)),
    (BACK_ARC, (67.825, 35.1148)),
    (BOSS[0], (47.1763, 39.0292)),
    (BACK_ARC, (43.5453, 39.5683)),
    (BOSS[1], (-8.8358, 42.5266)),
    (BACK_ARC, (-12.5043, 42.3998)),
    (BOSS[2], (-64.2390, 35.8995)),
    (LEFT, (-67.825, 35.1148)),
    (BOSS[3], (-67.825, -29.5897)),
    (LEFT_ARC, (-64.3992, -30.9151)),
    (FRONT, (-15.7901, -42.6534)),
    (BOSS[4], (-12.4065, -42.6534)),
    (FRONT, (-8.7405, -42.6534)),
    (THUMB_ARC, (-5.8494, -42.6534)),
    (THUMB_LINE, (48.6409, -51.2326)),
    (BOSS[5], (55.6115, -53.4975)),
    (SLANT, (59.0981, -54.6303)),
]

# 底面の傾斜（奥が高くなる）: YZ 平面で (y, z) = (-53.5, 12) と (60.6721, 0) を結ぶ線より下を落とす
TILT_FRONT = (-53.5, 12.0)
TILT_BACK_Y = 60.6721

# 奥の下側の面取り: 奥の辺に沿って、外周で高さ 12、内側へ 6 の三角形を回す
BEVEL_H = 12.0
BEVEL_W = 6.0

# トップケースを受ける切り欠き（x0, x1, y0, y1）
TOP_POCKETS = [
    (-67.83, -51.6485, -40.0, 40.0),
    (-26.65, 5.5, -54.4901, 47.1597),
    (30.5015, 70.0, -56.1449, 41.1434),
]

FAR = 100.0  # 外形より十分外側


def build():
    # 床より下はボスなし、床より上はボスつきの外形
    case = extrude(plan(), FLOOR_Z)
    case += Pos(0, 0, FLOOR_Z) * extrude(chain_face(OUTLINE), HEIGHT - FLOOR_Z)
    case = case.clean()

    # 床。奥と手前の辺を WALL だけ内側へずらしたもの。
    # 左右には壁がないので、左右と斜めの辺は外形と面が重ならないように外へずらす
    case -= Pos(0, 0, FLOOR_Z) * extrude(plan(-WALL, 1.0), HEIGHT)

    # トップケースを受ける切り欠き
    for x0, x1, y0, y1 in TOP_POCKETS:
        rect = Pos((x0 + x1) / 2, (y0 + y1) / 2) * Rectangle(x1 - x0, y1 - y0)
        case -= Pos(0, 0, HEIGHT - TOP_POCKET_DEPTH) * extrude(rect, TOP_POCKET_DEPTH + 1)

    # 底面の傾斜。YZ 平面に三角形を描いて x 方向に貫通させる
    fy, fz = TILT_FRONT
    wedge = Plane.YZ.offset(-FAR) * Polyline((fy - FAR, -FAR), (TILT_BACK_Y, 0), (fy, fz), (fy - FAR, fz), close=True)
    case -= extrude(Face(Wire(wedge.edges())), 2 * FAR)

    # 奥の下側の面取り。三角形を奥の辺の中心軸まわりに回転させて削る。
    # 斜面は外周より外まで延長して、奥の壁と稜線が重ならないようにする
    _, (cx, cy), r = BACK_ARC
    slope = BEVEL_H / BEVEL_W
    tri = Plane.XZ * Polyline(
        (r - BEVEL_W, 0), (r + 1, slope * (BEVEL_W + 1)), (r + 1, -FAR), (r - BEVEL_W, -FAR), close=True
    )
    case -= Pos(cx, cy) * revolve(Face(Wire(tri.edges())), Axis.Z)

    # 面取りと左右の側面の境目、右下の角（傾斜した底面から床まで）を R1
    cone = case.faces().filter_by(GeomType.CONE)[0]
    sides = [e for e in cone.edges() if abs(abs(e.center().X) - HALF_W) < 1e-6]
    corner = case.edges().filter_by(Axis.Z).filter_by(
        lambda e: (Vector(e.center().X, e.center().Y) - Vector(*SLANT[1])).length < 1e-6
    )
    case = case.fillet(1.0, sides + list(corner))

    # 面取りと底面の境目を R0.5
    cone = case.faces().filter_by(GeomType.CONE)[0]
    bottom = case.faces().sort_by(Axis.Z)[0]
    case = case.fillet(0.5, [e for e in cone.edges() if any(e.is_same(b) for b in bottom.edges())])

    # ナット穴と貫通穴。R1 のフィレットがナット穴にぶつかると OCC では作れないので、穴はフィレットの後で開ける
    for x, y in SCREWS:
        case -= Pos(x, y, -1) * Solid.make_cylinder(NUT_R, FLOOR_Z + 1)
        case -= Pos(x, y, -1) * Solid.make_cylinder(HOLE_R, HEIGHT + 2)
    # 左下と右下のボスは外形の角をちょうど通る設計だが、座標の丸めで角が 1e-5 mm ほど円の外に出て
    # 微小な切れ端が残るので、いちばん大きい塊だけを残す
    case = max(case.solids(), key=lambda s: s.volume)

    # 奥のナット穴が面取りに抜けたところを R0.5
    def on_nut(f, x, y):
        return all(abs(math.hypot(v.X - x, v.Y - y) - NUT_R) < 1e-4 for v in f.vertices())

    nut_faces = [
        f for f in case.faces().filter_by(GeomType.CYLINDER)
        if any(on_nut(f, x, y) for x, y in SCREWS if y > 0)
    ]
    edges = [e for f in nut_faces for e in f.edges() if e.geom_type not in (GeomType.LINE, GeomType.CIRCLE)]
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
