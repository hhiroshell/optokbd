# トップケース（左手）。v0.2.0: plate/plate_left.dxf のプレートに合わせたもの。
# - 座標はプレート DXF と同じ（y は奥が正）。z = 0 が下端、z = 22.6 が上面
# - 平面形はすべてプレートの外形からのオフセット（layout.py）。作り方は v0.1.0（FreeCAD 版）を踏襲
# - 完全無線化に合わせて、v0.1.0 にあった Pro Micro の切り欠き・コネクタのポケット・TRRS の穴はない
# - 実行: uv run python top.py [--show]   （out/ に STEP / STL / 3MF を書き出す。--show で OCP CAD Viewer に表示）
import math
import sys
from pathlib import Path

from build123d import (
    Axis, Box, Circle, Compound, Face, Kind, Mesher, Plane, Polyline, Pos, Rectangle, RectangleRounded, Rot, Solid, Vector,
    Wire, export_step, export_stl, extrude, offset,
)

import bottom as B
import layout as L
import plate as P
from outline import on_curve, path_face, plan_corners, plan_curves

HEIGHT = 22.6           # 上面。ベゼルの厚さ 5.0。プレートを 0.4 下げた分だけ 23.0 から下げて、プレートの上面からの高さを
                        # その前（5.5）に戻した（ガスケットのポケットの天井の上に 3.0、上面の面取りの斜面の下に 2.0 残る）
CAVITY_TOP = 17.6       # 内側の空間（ボトムケースの床の上）の高さ。ここから上面までがベゼル（厚さ 5.0）。
                        # プレートの上面（17.1、gasket.py）から PLATE_TOP_CLEAR 上
PLATE_TOP_CLEAR = 0.5   # ベゼルの下面とプレートの上面のすき間（gasket.py で確かめる）
INNER_TOP = B.RIM_Z - L.Z_OFFSET   # 外壁の内面（ボトムケースを囲む部分）の高さ（13.1）。ここが段差になり、ボトムケースの壁の上端が
                                   # 当たる
SLOT_TOP = 19.6         # ガスケットのポケットの高さ（ボトムケースの壁の上端と同じだけ下げて、ガスケットを挟む
                        # 高さ 6.5 を保つ）
INNER_BOSS_R = 2.4      # ボトムケースのボスを逃がす円
CHAMFER = 3.0           # 上面の外周（奥、手前左、親指の下の円弧と直線）の面取り
BED_CHAMFER = 0.3       # 上面（上面を下にして刷るので、印刷でベッドに当たる面）の縁の 45° の面取り: 外周の左右と右下の斜めの辺、
                        # キーの開口部の縁。1 層目（0.2）がつぶれて外や内へはみ出す分（エレファントフット）を、1 層目だけ約 0.2
                        # 引っ込めて打ち消す。上面の左右の辺の平らな幅は 2.1 → 1.5

NUT_Z = (15.6, 17.4)    # ナットのポケットの高さ（段差 INNER_TOP から 2.5 mm 上。ナットが抜け落ちないよう、上下に肉を残す）
NUT_AF = 4.2            # ナットのポケットの二面幅（M2 ナット 4.0 mm + 0.2）
HOLE_TOP = 18.6         # ネジ穴の深さ（ナットの上に 1.2 mm）
CASE_SCREW_LEN = 10.0   # ケースを留めるネジ（M2 × 10）
# 奥のネジは座面がボトムケースの面取りより上にあり（bottom.BACK_SEAT_Z）、ネジの先端も高くなるので、穴を深くする
HOLE_TOP_BACK = B.BACK_SEAT_Z - L.Z_OFFSET + CASE_SCREW_LEN + 0.2
assert NUT_Z[1] <= B.FLOOR_Z - L.Z_OFFSET + CASE_SCREW_LEN <= HOLE_TOP   # 手前のネジ: 先端がナットの上に出て、穴に収まる
assert NUT_Z[0] - INNER_TOP >= 2.5 - 1e-6                                # ナットの下の肉
# ナットのポケットの天井（上面を下にして刷るので、ナットが当たる z = NUT_Z[0] の面が印刷の天井になる）の作り方。サポートを
# 使わず、幅 4.2 の天井をブリッジで刷るための形（ネジ穴があると、ブリッジの線が穴の縁で途切れて垂れる）:
#   "round"    : ネジ穴をそのまま通す（今まで。サポートが要る）
#   "stepped"  : 段付きブリッジ。天井の 1 層目は穴の幅の帯（ブリッジの向き＝溝の幅の向きに沿う）を空け、2 層目は帯を直角に
#                渡して穴の位置に四角を残し、3 層目から丸い穴にする
#   "membrane" : 捨て層。ネジ穴を天井の 1 層手前で止め、穴のない膜をブリッジで渡す。印刷後に膜を Ø2 のドリルで抜く
NUT_CEILING = "stepped"
NUT_LAYER = 0.2         # 印刷の層の厚さ（NUT_Z[0] は上面から層の厚さの整数倍の位置にあること）


def _on_layer(d):
    return abs(d / NUT_LAYER - round(d / NUT_LAYER)) < 1e-6


assert _on_layer(HEIGHT - NUT_Z[0])

BEZEL_MARGIN = 0.6      # キーの開口部: 1U の枠から外へ（18 mm のキーキャップとは 1.125 mm）。角は R0.6。内側の空間の壁
                        # （プレートから PLATE_GAP）より 0.1 内側なので、キーの辺とプレートの辺が重なる所（左右、斜め、親指の下）
                        # では、ベゼルの下面に 0.1 の段ができる。プレートの外形がキーの角を切る所（R3C3 の左下、R0C3 の奥）では、
                        # 内側の空間よりわずかに外へ出る。一時 PLATE_GAP + 0.05 まで広げて壁で切っていたが、上面の左右の辺の
                        # 平らな幅を残すため（BED_CHAMFER を両側に入れる）戻した
BEZEL_ROUND = 0.5       # キーの開口部の内角の丸め

# 下端の切り欠き: 手前はボトムケースの傾斜した底面に合わせ、WALL_CUT_LAND_Y から奥は直線で上げ、奥の面では水平（ボトムケースの
# 奥の面取りを逃がす）
WALL_CUT_BACK_Z = B.BEVEL_H - L.Z_OFFSET        # 奥の面の高さ（7.5）。ボトムケースの面取りの上端（奥の外面の曲がった稜線）にそろえる
WALL_CUT_LAND_Y = -48.0                          # 直線がボトムケースの底面の傾斜から離れる位置（y。側面の中央より 12 mm 手前）

FAR = 200.0  # 外形より十分外側


def bezel():
    """キーの開口部: 各キーの 1U の枠を BEZEL_MARGIN だけオフセットした形（直線と R）の和集合の外周。
    親指キーの下辺は、プレートの親指の下の円弧から BEZEL_MARGIN 外の円弧（各キーの下辺の中点に接する）。
    内角は丸め、キーの間に囲まれたすき間（行と親指の列の間など）は開口部に含める（上面に浮いた島が残らないように）。"""
    keys = P.keys()
    opening = None
    for k in keys:
        sq = key_square(k, BEZEL_MARGIN)
        opening = sq if opening is None else opening + sq
    opening += thumb_band(keys)
    opening = offset(opening, BEZEL_ROUND, kind=Kind.ARC)
    opening = offset(opening, -BEZEL_ROUND, kind=Kind.ARC)
    return Face(opening.faces()[0].outer_wire())


def key_square(k, m):
    """キーの 1U の枠を m だけオフセットした角 R m の形。"""
    return Pos(k.center.X, k.center.Y) * Rot(0, 0, k.angle) * RectangleRounded(19.05 + 2 * m, 19.05 + 2 * m, m)


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


def screw_cut(screw, ceiling=NUT_CEILING):
    """ネジまわりの削る形: ナットのポケットとネジ穴。ceiling はポケットの天井の作り方（NUT_CEILING を参照）。"""
    z0, layer, rh = NUT_Z[0], NUT_LAYER, L.HOLE_R
    hole_top = HOLE_TOP_BACK if screw.back else HOLE_TOP
    pocket = nut_pocket(screw)
    # 輪郭が時計回りになり、面の法線が下を向くので、押し出す向きを固定する（固定しないと z0 から下に押し出される）
    cut = Pos(0, 0, z0) * extrude(pocket, NUT_Z[1] - z0, dir=(0, 0, 1))
    x, y = screw.pos.X, screw.pos.Y
    if ceiling == "membrane":
        cut += Pos(x, y, -1) * Solid.make_cylinder(rh, z0 - layer + 1)
        cut += Pos(x, y, z0) * Solid.make_cylinder(rh, hole_top - z0)
        return cut
    cut += Pos(x, y, -1) * Solid.make_cylinder(rh, hole_top + 1)
    if ceiling == "stepped":
        t = Vector(-screw.normal.Y, screw.normal.X)   # 溝の幅の向き（ブリッジの向き）
        at = Pos(x, y) * Rot(0, 0, math.degrees(math.atan2(t.Y, t.X)))
        strip = pocket & (at * Rectangle(4 * NUT_AF, 2 * rh))   # 1 層目: 穴の幅の帯（ポケットの中だけ）
        cut += Pos(0, 0, z0 - layer) * extrude(strip, layer, dir=(0, 0, 1))
        cut += Pos(0, 0, z0 - 2 * layer) * extrude(at * Rectangle(2 * rh, 2 * rh), layer, dir=(0, 0, 1))   # 2 層目: 穴の位置の四角
    else:
        assert ceiling == "round", ceiling
    return cut


NUT_TEST_SCREW = 1        # ナットのポケットのテストピースにするネジ（L.screws() の番号。奥の辺の中央。タブから離れている）
NUT_TEST_MARGIN = 1.5     # テストピース: ポケットの左右（溝の幅の向き）に残す肉
NUT_TEST_Z = (12.0, 19.0)  # テストピースの高さの範囲（トップケースの z）。上端は印刷のベッドの面になるので、NUT_Z[0] から
                           # 層の厚さの整数倍だけ上にする
assert _on_layer(NUT_TEST_Z[1] - NUT_Z[0])


def nut_test(ceiling):
    """ナットのポケットのはまり具合と、天井の刷れ方を試すテストピース: ケースのうち、奥の辺の中央のネジのまわりだけを、
    ケースの内面（ポケットの口）から外面まで切り出したもの。ケースと同じく上面を下にして刷る向き（z = 0 がベッド）で返す。"""
    s = L.screws()[NUT_TEST_SCREW]
    case = build(ceiling)
    t = Vector(-s.normal.Y, s.normal.X)
    half = NUT_AF / math.sqrt(3) + NUT_TEST_MARGIN
    z0, z1 = NUT_TEST_Z
    box = Pos(s.pos.X, s.pos.Y, (z0 + z1) / 2) * Rot(0, 0, math.degrees(math.atan2(t.Y, t.X))) * Box(2 * half, 20, z1 - z0)
    piece = case & box
    # 印刷の向き: ネジの位置を原点に、溝の幅を x に、z を反転して上端 z1 をベッドに
    flat = Rot(180, 0, 0) * Rot(0, 0, -math.degrees(math.atan2(t.Y, t.X))) * Pos(-s.pos.X, -s.pos.Y, -z1) * piece
    return flat


def wall_cut():
    """下端の切り欠きの断面（YZ 平面上 (y, z)）。手前はボトムケースの傾斜した底面に沿って切り、WALL_CUT_LAND_Y から奥へは
    直線で上げる（横から見ると、ボトムケースが奥へ太くなるくさび形の帯に見える）。奥の面（上から見ると円弧）の範囲では水平にして、
    奥の面の下端の高さを WALL_CUT_BACK_Z でそろえる（直線のままだと、円弧の中央ほど奥にあるぶん切り欠きが高くなる）。"""
    y_front = L.face(L.BOTTOM_OUTER).bounding_box().min.Y   # ボトムケースの最も手前
    y0 = y_front + (B.TILT_FRONT_Z - L.Z_OFFSET) / B.TILT_SLOPE   # ボトムケースの底面がトップケースの z = 0 を横切る位置
    assert WALL_CUT_LAND_Y < y0   # 着地点はボトムケースの底面の傾斜の上（そこより奥は z = 0 より下）
    y_land = WALL_CUT_LAND_Y
    z_land = B.tilt_z(y_land, y_front) - L.Z_OFFSET
    # 水平にし始める所: 奥の面の両端（左右の側面との角。R1 の丸めの分だけ手前）
    y_flat = min(c.Y for c in plan_corners(P.PLAN, L.TOP_OUTER) if c.Y > y0) - 1.0
    bb = L.face(L.TOP_OUTER).bounding_box()
    h = WALL_CUT_BACK_Z
    y_ff = bb.min.Y - 20
    z_ff = B.tilt_z(y_ff, y_front) - L.Z_OFFSET
    return path_face([
        ((y_ff, z_ff), None),
        ((y_land, z_land), None),
        ((y_flat, h), None),
        ((bb.max.Y + 20, h), None),
        ((bb.max.Y + 20, -1.0), None),
        ((y_ff, -1.0), None),
    ])


def build(ceiling=NUT_CEILING):
    screws = L.screws()
    case = extrude(L.face(L.TOP_OUTER), HEIGHT)

    # 内側: ボトムケースの床の上の空間、外壁の内面（ボスの逃げつき）、ガスケットのポケット
    case -= Pos(0, 0, -1) * extrude(L.face(L.TOP_CAVITY), CAVITY_TOP + 1)
    inner = L.face(L.TOP_INNER) + [Pos(s.pos.X, s.pos.Y) * Circle(INNER_BOSS_R) for s in screws]
    case -= Pos(0, 0, -1) * extrude(inner, INNER_TOP + 1)
    ring = L.face(L.TOP_INNER) - L.face(L.TOP_CAVITY)
    for tab in P.tabs():
        case -= Pos(0, 0, -1) * extrude(ring & L.tab_strip(tab, L.slot_half_width(tab)), SLOT_TOP + 1)

    # キーの開口部。上面の板（CAVITY_TOP から上）だけを抜く
    case -= Pos(0, 0, CAVITY_TOP) * extrude(bezel(), HEIGHT - CAVITY_TOP + 1)

    # ナットのポケットとネジ穴
    for s in screws:
        case -= screw_cut(s, ceiling)

    # 下端の切り欠き。x 方向に貫通させる
    case -= extrude(Plane.YZ * wall_cut(), FAR, both=True)

    # 上面の外周の面取り（奥、手前左、親指の下の円弧と直線）
    curves = plan_curves(P.PLAN, L.TOP_OUTER)
    # 親指の下の円弧は、両隣と接線でつながっているので指定しなくても面取りが回るが、明示しておく
    targets = [curves[i] for i in (P.I_BACK, P.I_FRONT_LEFT, P.I_THUMB, P.I_THUMB_LINE)]
    top = case.faces().filter_by(Axis.Z).sort_by(Axis.Z)[-1]
    edges = [e for e in top.edges() if any(on_curve(e.center(), c) for c in targets)]
    case = case.chamfer(CHAMFER, None, edges)

    # 上面の外周の残りの辺（左右と右下の斜めの辺）の BED_CHAMFER の面取り（エレファントフット対策）。角の R1 の前にかける
    # （R1 の後では、R1 と CHAMFER の面取りの端の R1 を伝って、CHAMFER の面取りとの境目にまで面取りが回ってしまう）
    sides = [curves[i] for i in (P.I_LEFT, P.I_RIGHT, P.I_SLANT)]
    top = case.faces().filter_by(Axis.Z).sort_by(Axis.Z)[-1]
    edges = [e for e in top.outer_wire().edges() if any(on_curve(e.center(), c) for c in sides)]
    assert len(edges) == len(sides)
    case = case.chamfer(BED_CHAMFER, None, edges)

    # 外周の縦の角と、そこにつながる面取りの端を R1（CHAMFER の面取りの端は、上端が BED_CHAMFER の面取りで切れている）
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
        chamfer_end = abs(low.Z - (HEIGHT - CHAMFER)) < 1e-3 and high.Z > HEIGHT - BED_CHAMFER - 1e-3
        if vertical or chamfer_end:
            edges.append(e)
    case = case.fillet(1.0, edges)

    # キーの開口部の縁の BED_CHAMFER の面取り（エレファントフット対策）
    top = case.faces().filter_by(Axis.Z).sort_by(Axis.Z)[-1]
    (opening,) = top.inner_wires()
    return case.chamfer(BED_CHAMFER, None, opening.edges())


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
    # ナットのポケットのテストピース: 段付きブリッジと捨て層を並べる（3MF では部品名で区別する）
    tests = [(f"top-nut-test-{c}", Pos(14 * i, 0, 0) * nut_test(c)) for i, c in enumerate(("stepped", "membrane"))]
    export_step(Compound([p for _, p in tests]), out / "top-nut-test-l.step")
    export_stl(Compound([p for _, p in tests]), out / "top-nut-test-l.stl")
    mesher = Mesher()
    for name, p in tests:
        mesher.add_shape(p, part_number=name)
    mesher.write(out / "top-nut-test-l.3mf")
    if "--show" in sys.argv:
        from ocp_vscode import show
        show(case)
