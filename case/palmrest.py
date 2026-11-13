# パームレスト（左手）。キーボードの手前に置く、断面を左右へ押し出した形（「波」の案）。
# - 座標は机の座標: キーボード（ゴム足込み）を机に置いた状態で、z = 0 が机の面。x と y はプレート DXF と同じ向き
#   （y は奥が正）で、キーボードの底面を水平に置くための傾き（bottom.TILT_ANGLE、x 軸まわり）だけ回した座標
# - 寸法はケースのコード（layout.py, plate.py, top.py, bottom.py）から読む。ケースを変えるとそろって変わる
# - 天面: 奥は高さ FLAT_Z の水平、途中からトップケースの上面と平行（6.25°）に手前へ下がる。手前は垂直に切る
# - 平面形: 左はトップケースの左の外面にそろえる。右はトップケースの右下の斜めの辺（の天面の稜線）を延長した直線で切る。
#   手前の辺は、トップケースの奥の円弧（外面）と同じ半径で向きを反対にした円弧
# - 奥の面は、トップケースの外面を GAP だけ広げた面。キーボードの手前の面（机の上では 6.25° 手前へ傾く）と平行になり、
#   すき間が高さによらずそろう
# - 天面の稜線: 奥・左・右は EDGE_CHAMFER の面取り、手前は FRONT_ROUND の丸め。左右の端の面の稜線も EDGE_CHAMFER の
#   面取り（奥の縦の稜線は、奥の面と鋭い角度で交わるので除く）
# - 印刷: 左の端の面を下にして立てて刷る。天面の曲がり（水平から傾斜への折れ目）と手前の丸めが壁になるので、積層の段差が
#   天面に出ない。A1 はベッドが前後に動くので、断面の長い向き（奥行き）を前後にそろえる。ブリムを付ける
# - 実行: uv run python palmrest.py [--show]   （out/print/ に STEP / STL / 3MF を書き出す。向きは置いた状態で、
#   底面が z = 0。--show で OCP CAD Viewer にキーボードと並べて表示）
import math
import sys
from pathlib import Path

from build123d import (
    Align, Axis, Circle, GeomType, Line, Plane, Polygon, Pos, Rectangle, Rot, ThreePointArc, Vector, chamfer, extrude, fillet,
    make_face,
)

import bottom as B
import layout as L
import plate as P
import top as T
from outline import plan_corners, plan_face

GAP = 1.5               # キーボードの外面（トップケースの外形）とのすき間
FLAT_Z = 17.6           # 天面の水平部分の高さ（机から）。ケースの手前の上面（19.7〜22.1）より少し低い
SLOPE_SHIFT = 25.0      # 傾斜の始まり: 水平部分とトップケースの上面の延長面の交線から、手前へずらす量（延長面に乗せると
                        # 手前が薄くなりすぎた）。傾斜はトップケースの上面と平行
BLEND_R = 150.0         # 水平部分と傾斜の折れ目をつなぐ円弧（折れ角 6.25° なので、元の折れ目からのずれは 0.2 mm）
FRONT_Y = -185.0        # 手前の辺（円弧）の、トップケースの奥の円弧の中心の x での y
FRONT_ROUND = 3.0       # 天面の手前の稜線の丸め
EDGE_CHAMFER = 0.6      # 天面の奥・左・右の稜線と、左右の端の面の稜線の面取り
BACK_Y = -50.0          # 平面形の奥の端（キーボードの手前の面で切るので、それより奥ならよい）
PROFILE_Y = (-55.0, -205.0)   # 断面を描く範囲（奥、手前）。平面形より広く

# ---- 机の座標 ----
TILT = math.radians(B.TILT_ANGLE)
_Y_FRONT = L.face(L.BOTTOM_OUTER).bounding_box().min.Y
_Z0 = _Y_FRONT * math.sin(TILT) + B.TILT_FRONT_Z * math.cos(TILT)   # 底面の傾斜カットの面を回したあとの高さ
TOP_Z = T.HEIGHT + L.Z_OFFSET   # トップケースの上面（ボトムケースの座標）
TOP_TAN = math.tan(TILT)        # トップケースの上面は、机の座標で z = TOP_TAN * y + TOP_B
TOP_B = TOP_Z / math.cos(TILT) + B.FOOT_H - _Z0


def to_table(s):
    """ボトムケースの座標 → 机の座標（底面を水平に、ゴム足の高さだけ浮かせる）。"""
    return Pos(0, 0, B.FOOT_H - _Z0) * (Rot(math.degrees(TILT), 0, 0) * s)


def table_xy(p, z=TOP_Z):
    """ボトムケースの座標の点（高さ z）の、机の座標での平面の位置。"""
    return Vector(p.X, p.Y * math.cos(TILT) - z * math.sin(TILT))


def keepout():
    """キーボードの外面を GAP だけ広げた柱（机の座標）。手前の面はケースと同じく手前へ傾く。"""
    grow = {"edge": L.TOP_OUTER["edge"] + GAP, "side": L.TOP_OUTER["side"] + GAP}
    return to_table(Pos(0, 0, -60) * extrude(plan_face(P.PLAN, grow), 200))


def slant_line():
    """トップケースの右下の斜めの辺の、天面の稜線（机の座標）: 点（手前の端）と向き。"""
    corners = plan_corners(P.PLAN, L.TOP_OUTER)
    a = min(corners, key=lambda c: c.Y)                                  # 外形の手前の右の角
    b = min((c for c in corners if c.X > a.X - 1 and c.Y > a.Y + 1), key=lambda c: c.Y)   # 斜めの辺の奥の端
    a, b = table_xy(a), table_xy(b)
    return a, (b - a).normalized()


def plan():
    """平面形（机の座標）。奥はキーボードと重なっている（keepout() で切る）。"""
    p, d = slant_line()
    far = 400.0
    left_of = Polygon(p - d * far, p + d * far, p + d * far + Vector(-far, 0), p - d * far + Vector(-far, 0), align=None)
    if left_of.faces()[0].normal_at().Z < 0:
        left_of = Polygon(*reversed(left_of.vertices()), align=None)
    outer = L.face(L.TOP_OUTER).bounding_box()
    side = Pos(outer.min.X, BACK_Y) * Rectangle(outer.size.X, far, align=(Align.MIN, Align.MAX))   # 左右はトップケースの幅の中
    r = L.TOP_OUTER["edge"] + P.BACK[2]
    front = Pos(P.BACK[1][0], FRONT_Y + r) * Circle(r)
    return max((front & left_of & side).faces(), key=lambda f: f.area)


def slope_start():
    return (FLAT_Z - TOP_B) / TOP_TAN - SLOPE_SHIFT


def profile():
    """断面（YZ 平面）: 水平部分、折れ目の円弧、トップケースの上面と平行な傾斜。"""
    y_back, y_front = PROFILE_Y
    yi = slope_start()
    t = BLEND_R * math.tan(TILT / 2)
    p1 = Vector(yi + t, FLAT_Z)
    p2 = Vector(yi - t * math.cos(TILT), FLAT_Z - t * math.sin(TILT))
    c = Vector(p1.X, FLAT_Z - BLEND_R)
    m = c + (Vector(yi, FLAT_Z) - c).normalized() * BLEND_R
    pf = Vector(y_front, FLAT_Z + TOP_TAN * (y_front - yi))
    z0 = B.FOOT_H
    edges = [Line((y_back, FLAT_Z), p1), ThreePointArc(p1, m, p2), Line(p2, pf),
             Line(pf, (y_front, z0)), Line((y_front, z0), (y_back, z0)), Line((y_back, z0), (y_back, FLAT_Z))]
    return Plane.YZ * make_face(edges)


def back_faces(part):
    """奥の面（キーボードに向いた、上ほど手前に出る面）。"""
    return [f for f in part.faces() if f.normal_at(f.center()).Y > 0.5 and 0.01 < f.normal_at(f.center()).Z < 0.3]


def end_edges(part, face):
    """左右の端の面の稜線のうち、奥の面との境（鋭い角度で交わる）を除いたもの。"""
    backs = [e for f in back_faces(part) for e in f.edges()]
    return [e for e in face.edges() if not any(e.is_same(b) for b in backs)]


def build():
    f = plan()
    bb = f.bounding_box()
    part = Pos(bb.min.X - 1, 0, 0) * extrude(profile(), bb.size.X + 2)
    part = (part & extrude(f, 50)) - keepout()
    # 天面の手前の稜線: 手前の面（手前の円弧の円柱面）の上端
    front = min(part.faces(), key=lambda fa: fa.center().Y)
    part = fillet([max(front.edges(), key=lambda e: e.center().Z)], FRONT_ROUND)
    # 天面の奥の稜線と、右の端（斜めの辺の延長の面）・左の端の面の稜線。1 回の面取りにまとめる（分けると、奥の左の角に
    # 細い面ができて 3MF のメッシュが作れなかった）
    _, d = slant_line()
    n = Vector(d.Y, -d.X)
    right = max((fa for fa in part.faces() if fa.geom_type == GeomType.PLANE and abs(fa.normal_at().dot(n)) > 0.999),
                key=lambda fa: fa.area)
    left = part.faces().sort_by(Axis.X)[0]
    back_top = [e for fa in back_faces(part) for e in fa.edges() if e.center().Z > FLAT_Z - 0.1]
    part = chamfer(back_top + end_edges(part, right) + end_edges(part, left), EDGE_CHAMFER)
    return part


def keyboard():
    """机の座標に置いたキーボード（トップケースとボトムケース）。"""
    return to_table(Pos(0, 0, L.Z_OFFSET) * T.build()), to_table(B.build())


def check(part, kb):
    assert len(part.solids()) == 1
    gap = min(part.distance_to(s) for s in kb)
    assert gap > GAP - 0.05, gap
    return gap


if __name__ == "__main__":
    part = build()
    kb = keyboard()
    gap = check(part, kb)
    bb = part.bounding_box()
    out = Path(__file__).parent / "out" / "print"
    out.mkdir(parents=True, exist_ok=True)
    placed = Pos(0, 0, -bb.min.Z) * part
    L.export_hands("palmrest", placed, out)
    print(f"size = {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm (top {bb.max.Z:.2f} from the desk), "
          f"volume = {part.volume:.2f} mm^3, solids = {len(part.solids())}, gap = {gap:.2f}, "
          f"slope starts at y = {slope_start():.1f}")
    if "--show" in sys.argv:
        from ocp_vscode import show
        show(part, *kb, names=["palmrest-l", "top-l", "bottom-l"], colors=["#70929e", "#d0d0cb", "#b2b2ad"])
