# トップケース（左手）。v0.1.0/top.FCStd の Body を build123d で書き直したもの。
# - 座標は bottom.py と同じ。z = 0 が下端、z = 23 が上面
# - FreeCAD 版はミラーした右手用も同じファイルに置いていたが、ここでは左手だけを作る
# - 実行: uv run python top.py [--show]   （out/ に STEP / STL / 3MF を書き出す。--show で OCP CAD Viewer に表示）
import sys
from pathlib import Path

from build123d import (
    Axis, Face, Plane, Polyline, Pos, Rectangle, Solid, Vector, Wire,
    Mesher, export_step, export_stl, extrude,
)

from outline import (
    BACK_ARC, FRONT, LEFT_ARC, SCREWS, THUMB_ARC, on_curve, path_face, plan, plan_curves, shifted, with_bosses,
)

HEIGHT = 23.0
OUTER_GROW = 2.5        # 外壁: ボトムケースの外形から外へ（左右と斜めの辺は OUTER_SIDE_GROW）
OUTER_SIDE_GROW = 2.7
INNER_GROW = 0.5        # 外壁の内面: ボトムケースの外形から外へ（左右と斜めの辺は INNER_SIDE_GROW）
INNER_SIDE_GROW = 0.7
INNER_TOP = 11.0        # 外壁の内面の高さ
INNER_BOSS_R = 2.4      # ボトムケースのボスを逃がす円
BUMP_GROW = -2.8        # ボトムケースの床の上の空間（ボトムケースの床と同じ輪郭。左右だけ INNER_SIDE_GROW）
BUMP_TOP = 19.0
GASKET_TOP = 20.0       # ガスケットのポケットの高さ
GASKET_X = [(-51.6485, -26.6485), (5.5015, 30.5015)]  # ガスケットのポケットの x の範囲（それぞれ手前と奥）
CHAMFER = 3.0           # 上面の外周（奥、手前左、右下の直線）の面取り

NUT_Z = (13.0, 14.8)    # ナットを入れるポケットの高さ
HOLE_R = 1.2            # ネジ穴
HOLE_TOP = 16.0

# キーの開口部: [(点, 次の点までの辺), ...]。辺は None（直線）か円弧
BEZEL = [
    ((57.1311, -29.5446), None),
    ((66.7595, -32.6730), None),
    ((60.5020, -51.9316), None),
    ((50.8727, -48.8028), ("arc", (-6.2273, -222.3994), 182.7462)),
    ((-6.4906, -39.6534), None),
    ((-16.0883, -39.6534), None),
    ((-16.0883, -20.6034), None),
    ((-29.0237, -20.6034), None),
    ((-29.0237, -26.3184), None),
    ((-68.3233, -26.3184), None),
    ((-68.3233, 32.0312), None),
    ((-30.2233, 32.0312), None),
    ((-30.2233, 37.7462), None),
    ((-11.1733, 37.7462), None),
    ((-11.1733, 39.6512), None),
    ((9.0763, 39.6512), None),
    ((9.0763, 37.7462), None),
    ((28.1263, 37.7462), None),
    ((28.1263, 35.8412), None),
    ((47.1763, 35.8412), None),
    ((47.1763, -22.5084), None),
    ((36.3761, -22.5084), ("arc", (36.3761, -23.2084), 0.7)),
    ((36.2297, -23.8929), ("arc", (-6.2244, -222.3736), 202.9703)),
]

# ナットを入れるポケット。ネジごとの六角形（内側へ逃がしてある）
NUT_POCKETS = [
    [(44.3532, 40.7681), (42.8645, 38.8928), (42.5668, 36.7786), (47.3021, 36.0756), (47.6315, 38.1851), (46.7518, 40.4120)],
    [(-8.2353, 41.7788), (-9.5034, 43.8098), (-11.9268, 43.7260), (-13.0517, 41.6123), (-12.9619, 39.4791), (-8.1776, 39.6445)],
    [(-9.3611, -43.9534), (-8.1641, -41.8802), (-8.1641, -39.7532), (-12.9829, -39.7533), (-12.9829, -41.8802), (-11.7859, -43.9534)],
    [(-67.4955, 36.5232), (-68.2217, 34.2417), (-67.7497, 32.1594), (-63.0779, 33.1818), (-63.5169, 35.2622), (-65.1264, 37.0416)],
    [(55.9596, -50.5611), (55.3024, -52.5838), (55.8000, -54.9255), (58.1062, -55.6749), (59.8653, -54.0909), (60.5424, -52.0501)],
    [(-67.7149, -31.0351), (-65.4534, -31.9100), (-63.5882, -30.4077), (-62.8572, -28.3974), (-67.3028, -26.6770), (-68.0791, -28.6961)],
]

# Pro Micro の切り欠き（奥の右）: x の範囲と、下側の円弧
PROMICRO_X = (49.6763, 65.6763)
PROMICRO_ARC = ("arc", (57.6763, 59.9137), 30.0)
# Pro Micro のコネクタのポケット: (x0, x1, y0, y1)、高さ
CONNECTOR = (51.1763, 67.025, 26.0, 34.0)
CONNECTOR_TOP = 20.0

# TRRS ジャックの穴（右の側面）: y の範囲、直線部の上端、上の円弧
TRRS_Y = (-19.5, -11.5)
TRRS_TOP = 18.5
TRRS_ARC = ("arc", (-15.5, 11.0), 8.5)  # YZ 平面上 (y, z)

# 下端の切り欠き（YZ 平面上 (y, z)）。手前は直線、奥は円弧で、両端が高さ 6 まで上がる
WALL_CUT_FRONT = (-58.0, 6.0)
WALL_CUT_LOW_Y = -0.9138
WALL_CUT_BACK = (45.4999, 6.0)
WALL_CUT_ARC = ("arc", (44.8770, -171.7010), 177.7021)

FAR = 100.0  # 外形より十分外側


def gasket_pockets():
    """ガスケットのタブが入るポケット。外壁の内面とボトムケースの床の輪郭の間を、x の範囲で切り出す。"""
    ring = plan(INNER_GROW, INNER_SIDE_GROW) - plan(BUMP_GROW, INNER_SIDE_GROW)
    strips = [Pos((x0 + x1) / 2, 0) * Rectangle(x1 - x0, 2 * FAR) for x0, x1 in GASKET_X]
    return ring & (strips[0] + strips[1])


def corner_points():
    """外周の角（R1 のフィレットをかける縦の稜線の位置）。"""
    outer = plan(OUTER_GROW, OUTER_SIDE_GROW)
    back = shifted(BACK_ARC, OUTER_GROW)
    # 手前中央の水平な辺と親指の下の円弧の両端は、ほぼ接線つながりなので除く
    smooth = [shifted(FRONT, -OUTER_GROW), shifted(THUMB_ARC, -OUTER_GROW)]
    pts = [Vector(v.X, v.Y) for v in outer.vertices() if not any(on_curve(v, c) for c in smooth)]
    # Pro Micro の切り欠きの両側と、TRRS の穴の両側
    _, (cx, cy), r = back
    pts += [Vector(x, cy + (r * r - (x - cx) ** 2) ** 0.5) for x in PROMICRO_X]
    pts += [Vector(plan_curves(OUTER_GROW, OUTER_SIDE_GROW)[0][1][0], y) for y in TRRS_Y]
    return pts


def reflex_corners(path):
    """開口部の輪郭のうち、直線どうしが内側へへこんで交わる角（時計回りの輪郭で左に曲がる角）。"""
    pts = [Vector(*p) for p, _ in path]
    out = []
    n = len(path)
    for i in range(n):
        if path[i - 1][1] is not None or path[i][1] is not None:
            continue
        a, b, c = pts[i - 1], pts[i], pts[(i + 1) % n]
        if (b - a).cross(c - b).Z > 1e-9:
            out.append(b)
    return out


def build():
    case = extrude(plan(OUTER_GROW, OUTER_SIDE_GROW), HEIGHT)

    # 内側: ボトムケースの床の上の空間、外壁の内面、ガスケットのポケット
    case -= Pos(0, 0, -1) * extrude(plan(BUMP_GROW, INNER_SIDE_GROW), BUMP_TOP + 1)
    case -= Pos(0, 0, -1) * extrude(with_bosses(plan(INNER_GROW, INNER_SIDE_GROW), INNER_BOSS_R), INNER_TOP + 1)
    case -= Pos(0, 0, -1) * extrude(gasket_pockets(), GASKET_TOP + 1)

    # キーの開口部
    case -= Pos(0, 0, -1) * extrude(path_face(BEZEL), HEIGHT + 2)

    # ナットのポケットとネジ穴
    for pts in NUT_POCKETS:
        case -= Pos(0, 0, NUT_Z[0]) * extrude(Face(Wire(Polyline(*pts, close=True).edges())), NUT_Z[1] - NUT_Z[0])
    for x, y in SCREWS:
        case -= Pos(x, y, -1) * Solid.make_cylinder(HOLE_R, HOLE_TOP + 1)

    # Pro Micro の切り欠き（上から貫通）とコネクタのポケット
    x0, x1 = PROMICRO_X
    ya = PROMICRO_ARC[1][1] - (PROMICRO_ARC[2] ** 2 - (x0 - PROMICRO_ARC[1][0]) ** 2) ** 0.5
    notch = path_face([((x0, ya), PROMICRO_ARC), ((x1, ya), None), ((x1, FAR), None), ((x0, FAR), None)])
    case -= Pos(0, 0, -1) * extrude(notch, HEIGHT + 2)
    x0, x1, y0, y1 = CONNECTOR
    case -= Pos((x0 + x1) / 2, (y0 + y1) / 2, -1) * extrude(Rectangle(x1 - x0, y1 - y0), CONNECTOR_TOP + 1)

    # TRRS ジャックの穴。外壁の内面（x = 右の辺 + INNER_SIDE_GROW）から外へ抜く
    y0, y1 = TRRS_Y
    inner_x = plan_curves(INNER_GROW, INNER_SIDE_GROW)[0][1][0]
    trrs = path_face([((y0, -1), None), ((y1, -1), None), ((y1, TRRS_TOP), TRRS_ARC), ((y0, TRRS_TOP), None)])
    case -= Plane.YZ.offset(inner_x) * extrude(trrs, OUTER_SIDE_GROW - INNER_SIDE_GROW + 1)

    # 下端の切り欠き。x 方向に貫通させる
    cut = path_face([
        (WALL_CUT_FRONT, None),
        ((WALL_CUT_LOW_Y, 0.0), WALL_CUT_ARC),
        (WALL_CUT_BACK, None),
        ((WALL_CUT_BACK[0], -1.0), None),
        ((WALL_CUT_FRONT[0], -1.0), None),
    ])
    case -= Plane.YZ.offset(-FAR) * extrude(cut, 2 * FAR)

    # 上面の外周の面取り（奥、手前左、右下の直線）
    targets = [shifted(BACK_ARC, OUTER_GROW), shifted(LEFT_ARC, OUTER_GROW), plan_curves(OUTER_GROW, OUTER_SIDE_GROW)[6]]
    top = case.faces().filter_by(Axis.Z).sort_by(Axis.Z)[-1]
    edges = [e for e in top.edges() if any(on_curve(e.center(), c) for c in targets)]
    case = case.chamfer(CHAMFER, None, edges)

    # 外周の縦の角と、そこにつながる面取りの端を R1
    def at(p, corners):
        return any((Vector(p.X, p.Y) - c).length < 1e-3 for c in corners)

    corners = corner_points()
    edges = []
    for e in case.edges():
        vs = sorted(e.vertices(), key=lambda v: v.Z)
        low, high = vs[0], vs[-1]
        if high.Z - low.Z < 1:
            continue
        vertical = at(low, corners) and (Vector(low.X, low.Y) - Vector(high.X, high.Y)).length < 1e-6
        chamfer_end = at(low, corners) and abs(low.Z - (HEIGHT - CHAMFER)) < 1e-3 and abs(high.Z - HEIGHT) < 1e-3
        if vertical or chamfer_end:
            edges.append(e)
    case = case.fillet(1.0, edges)

    # 開口部のへこんだ角（ボトムケースの床の上の空間より上の部分）を R0.5
    corners = reflex_corners(BEZEL)
    edges = [
        e for e in case.edges().filter_by(Axis.Z)
        if any((Vector(e.center().X, e.center().Y) - c).length < 1e-3 for c in corners)
    ]
    case = case.fillet(0.5, edges)
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
