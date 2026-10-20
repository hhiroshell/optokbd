# ボトムケース（左手）。v0.2.0: plate/plate_left.dxf のプレートに合わせたもの。
# - 座標はプレート DXF と同じ（y は奥が正）。z = 0 が底面の基準、z = 20 がトップケースとの合わせ面
# - 平面形はすべてプレートの外形からのオフセット（layout.py）。作り方は v0.1.0（FreeCAD 版）を踏襲
# - 実行: uv run python bottom.py [--show]   （out/ に STEP / STL / 3MF を書き出す。--show で OCP CAD Viewer に表示）
import math
import sys
from pathlib import Path

from OCP.Standard import Standard_ConstructionError
from build123d import (
    Align, Axis, Box, Circle, Face, GeomType, Mesher, Plane, Polyline, Pos, Rectangle, Rot, Solid, Vector, Vertex, Wire,
    export_step, export_stl, extrude, revolve,
)

import layout as L
import plate as P
from outline import plan_corners

HEIGHT = 19.6           # ケースの高さ（傾斜カット前）= ガスケットを受ける柱の上端。マトリクス基板と床の間を
                        # 2.5 mm にして 2 mm のフォームを入れるため、20.0 から 0.4 下げた（トップケースの
                        # ガスケットのポケットの天井も同じだけ下げる）
FLOOR_Z = 14.5          # 内側の床の高さ
RIM_Z = 17.5            # 壁の上端を下げた高さ。タブの下の柱だけ HEIGHT まで残す（トップケースの段差がここに当たる）

# 底面の傾斜（奥が高くなる）: 最も手前の点で TILT_FRONT_Z だけ削り、奥へ TILT_SLOPE で浅くなる
TILT_FRONT_Z = 12.5     # 最も手前で床の厚さ FLOOR_Z - 12.5 = 2.0 mm を残す（全体の厚みを抑えるため 2.5 から薄くした）
TILT_ANGLE = 6.25       # 底面（底蓋の面）の傾き [度]。v0.2.0 の初めは 6°（12 / 114.1721）、一時 7° にしたが、電池を薄くして
                        # 全体の厚みを抑えるため下げた（6° では収納部の天井と、底蓋のネジのボスの高さが足りない）
TILT_SLOPE = math.tan(math.radians(TILT_ANGLE))

# 奥の下側の面取り: 奥の辺に沿って、外周で高さ BEVEL_H、内側へ BEVEL_W の三角形を回す
BEVEL_H = 14.0          # USB-C のケーブルのくぼみが面取りの斜面の中に収まる高さ（12 では上の縦の面にはみ出した）。
                        # 奥のネジ頭の穴の座面（FLOOR_Z）を削らないよう、FLOOR_Z - BEVEL_SEAT_CLEAR まで
BEVEL_SEAT_CLEAR = 0.5
# 奥のネジ（トップケースを留める M2 の六角穴付きボルト）のネジ頭は、外面より外へはみ出す。面取りの上端より上に沈めて、
# トップケースの壁（下端は面取りの上端にそろう）の裏に隠す。座面を床より上げると、ネジ頭の穴が壁の厚さを突き抜けて中の空間に
# 開くので、ネジのまわりだけ壁を内側へ厚くした台（BACK_BOSS_R、上端 BACK_BOSS_TOP）で囲む
SCREW_HEAD_H = 2.0      # M2 の六角穴付きボルトの頭の高さ（Ø3.8 × 2.0）
BACK_SEAT_Z = BEVEL_H + SCREW_HEAD_H + 0.3   # 奥のネジの座面（ネジ頭の下面を面取りの上端より 0.3 上に）
BACK_BOSS_R = L.BOSS_R + 1.2              # 台の半径（ネジ頭の穴のまわりに 1.2）。外形の外へは出さない
BACK_BOSS_TOP = BACK_SEAT_Z + 0.2         # 台の上端。マトリクス基板の下面（17.0）より下
BEVEL_OVERRUN = 0.1     # 面取りを削る三角形を、外面より外へ延ばす量（奥の壁と稜線が重ならないように）。床の上面より上で外面の外へ
                        # 張り出したネジのボス（L.BOSS_R）を斜面の延長が削らないよう、延長の先は床の上面より低くする
BEVEL_W = 6.0
assert BEVEL_H <= FLOOR_Z - BEVEL_SEAT_CLEAR
assert BEVEL_H / BEVEL_W * (BEVEL_W + BEVEL_OVERRUN) < FLOOR_Z
assert BACK_SEAT_Z - SCREW_HEAD_H >= BEVEL_H + 0.3

# ゴム足（直径 FOOT_D のクッションゴム）の座ぐり。底面（底蓋の外面と同じ傾いた面）に垂直に掘る。位置は底面の座標
# （bay.FRAME の x と、底面に沿って奥へ v）で、四隅に最も近く、座ぐりの縁が底面の縁・底蓋の開口・ネジ頭の穴の切り欠きから
# FOOT_EDGE_CLEAR 以上離れる点（0.5 mm 刻みで探した）
FOOT_D, FOOT_H = 7.9, 2.2   # ゴム足の直径と高さ
FOOT_CLEAR = 0.25       # ゴム足と座ぐりの縁のすき間（片側）
FOOT_DEPTH = 0.6        # 座ぐりの深さ（層の厚さ 0.2 の 3 層）。ゴム足は底面から FOOT_H - FOOT_DEPTH（1.6）出る
FOOT_EDGE_CLEAR = 4.0   # 縁に寄せすぎない（1.5 から内側へ寄せた）
FOOT_POS = [(9.0, -16.0), (124.5, -16.0), (9.0, -61.0), (120.0, -81.5)]   # 奥の左、奥の右、手前の左、手前の右

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

    # 奥のネジのまわりの台（床の上に、ネジ頭の穴を囲む）
    inside = Pos(0, 0, FLOOR_Z - 0.1) * extrude(outer, BACK_BOSS_TOP - FLOOR_Z + 0.1)
    for s in screws:
        if s.back:
            case += inside & (Pos(s.pos.X, s.pos.Y, FLOOR_Z - 0.1) * Solid.make_cylinder(BACK_BOSS_R, BACK_BOSS_TOP - FLOOR_Z + 0.1))

    # 底面の傾斜。YZ 平面に描いて x 方向に貫通させる
    y_front = outer.bounding_box().min.Y
    y_zero = y_front + TILT_FRONT_Z / TILT_SLOPE
    y_far = y_front - 20
    wedge = Plane.YZ * Polyline(
        (y_far, tilt_z(y_far, y_front)), (y_zero, 0), (y_zero, -FAR), (y_far, -FAR), close=True
    )
    case -= extrude(Face(Wire(wedge.edges())), FAR, both=True)

    # 奥の下側の面取り。三角形を奥の辺の中心軸まわりに回転させて削る。
    # 斜面は外周より外まで BEVEL_OVERRUN だけ延長して、奥の壁と稜線が重ならないようにする
    _, (cx, cy), r = P.BACK
    r += L.BOTTOM_OUTER["edge"]
    slope = BEVEL_H / BEVEL_W
    tri = Plane.XZ * Polyline(
        (r - BEVEL_W, 0), (r + BEVEL_OVERRUN, slope * (BEVEL_W + BEVEL_OVERRUN)), (r + BEVEL_OVERRUN, -FAR),
        (r - BEVEL_W, -FAR), close=True,
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
    # 奥のネジ頭の穴は外面より外へはみ出して面取りに抜けるので、穴の中心から外へ穴の直径の幅でまっすぐ延ばして U 字にする
    # （円のままだと外面の位置で幅が狭まり、抜けの縁がすぼまって見える）
    for s in screws:
        seat = BACK_SEAT_Z if s.back else FLOOR_Z
        case -= Pos(s.pos.X, s.pos.Y, -1) * Solid.make_cylinder(L.BOSS_R, seat + 1)
        if s.back:
            ang = math.degrees(math.atan2(s.normal.Y, s.normal.X))
            case -= Pos(s.pos.X, s.pos.Y, -1) * Rot(0, 0, ang) * Box(
                5.0, 2 * L.BOSS_R, seat + 1, align=(Align.MIN, Align.CENTER, Align.MIN))
        case -= Pos(s.pos.X, s.pos.Y, -1) * Solid.make_cylinder(L.HOLE_R, HEIGHT + 2)

    # 奥のネジ頭の穴が面取りに抜けたところを R0.5（面取りの高さによっては OCC で作れないので、そのときは丸めない）
    def on_hole(f, p):
        return all(abs(math.hypot(v.X - p.X, v.Y - p.Y) - L.BOSS_R) < 1e-4 for v in f.vertices())

    hole_faces = [
        f for f in case.faces().filter_by(GeomType.CYLINDER)
        if any(on_hole(f, s.pos) for s in screws if s.back)
    ]
    edges = [e for f in hole_faces for e in f.edges() if e.geom_type not in (GeomType.LINE, GeomType.CIRCLE)]
    if edges:
        try:
            case = case.fillet(0.5, edges)
        except (ValueError, Standard_ConstructionError):
            pass

    # コントローラーの収納部（底蓋の開口、USB-C のプラグの穴とケーブルのくぼみ、底蓋の四隅のネジのボス）
    import bay  # bay は bottom の寸法を使うので、ここで読み込む
    for cut in bay.case_cuts():
        case -= cut
    case = bay.add_screw_bosses(case)

    # ゴム足の座ぐり
    for x, v in FOOT_POS:
        case -= bay.local_cylinder(x, v, -1, FOOT_DEPTH, FOOT_D + 2 * FOOT_CLEAR)
    return case


def check_feet(case):
    """ゴム足の座ぐりの縁と、底面の縁（座ぐりを掘る前の底面の輪郭と穴）の最小距離。"""
    import bay
    local = bay.FRAME.to_local_coords(case)
    bottom = [f for f in local.faces()
              if f.geom_type == GeomType.PLANE and abs(f.normal_at().Z + 1) < 1e-6 and abs(f.center().Z) < 1e-4]
    assert len(bottom) == 1
    r = FOOT_D / 2 + FOOT_CLEAR
    wires = [w for w in bottom[0].inner_wires()
             if all(abs(abs(e.radius) - r) > 1e-3 for e in w.edges() if e.geom_type == GeomType.CIRCLE)]
    wires.append(bottom[0].outer_wire())
    return min(Vertex(x, v, 0).distance_to(w) for x, v in FOOT_POS for w in wires) - r


if __name__ == "__main__":
    case = build()
    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    export_step(case, out / "bottom-l.step")
    export_stl(case, out / "bottom-l.stl")
    mesher = Mesher()  # 3D プリンタのスライサー用。単位は mm
    mesher.add_shape(case)
    mesher.write(out / "bottom-l.3mf")
    print(f"volume = {case.volume:.2f} mm^3, foot recess edge clearance = {check_feet(case):.2f} mm")
    assert check_feet(case) >= FOOT_EDGE_CLEAR - 0.3   # 0.5 mm 刻みで探した位置なので、少しの差は許す
    if "--show" in sys.argv:
        from ocp_vscode import show
        show(case)
