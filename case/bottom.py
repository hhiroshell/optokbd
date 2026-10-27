# ボトムケース（左手）。v0.2.0: plate/plate_left.dxf のプレートに合わせたもの。
# - 座標はプレート DXF と同じ（y は奥が正）。z = 0 が底面の基準、z = 20 がトップケースとの合わせ面
# - 平面形はすべてプレートの外形からのオフセット（layout.py）。作り方は v0.1.0（FreeCAD 版）を踏襲
# - 実行: uv run python bottom.py [--show]   （out/ に STEP / STL / 3MF を書き出す。--show で OCP CAD Viewer に表示）
import math
import sys
from pathlib import Path

from OCP.Standard import Standard_ConstructionError
from build123d import (
    Align, Axis, Box, Circle, Face, GeomType, Line, Mesher, Plane, Polyline, Pos, Rectangle, Rot, Solid, ThreePointArc, Vector,
    Vertex, Wire, export_step, export_stl, extrude, revolve,
)

import layout as L
import plate as P
from outline import plan_corners

HEIGHT = 19.6           # ケースの高さ（傾斜カット前）= 壁の上端（ガスケットを受ける面）。マトリクス基板と床の間を
                        # 2.5 mm にして 2 mm のフォームを入れるため、20.0 から 0.4 下げた（トップケースの
                        # ガスケットのポケットの天井も同じだけ下げる）
FLOOR_Z = 14.5          # 内側の床の高さ
RIM_Z = HEIGHT          # 壁の上端（トップケースの段差がここに当たる）。ガスケットのタブの下も同じ高さの平らな面（以前は壁を
                        # 17.5 → 19.0 に下げて、タブの下だけ柱として残していた。奥のネジの座面を上げたので、ネジ頭とトップ
                        # ケースの間の肉を稼ぐため、柱と同じ高さまで上げて平らにした）

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
BEVEL_BULGE = 0.2       # 面取りの斜面の断面を、上下の端を通る円弧にして、中央で外へふくらませる量。印刷すると斜面が底から
                        # 奥へ向かって一度内側に凹むように反るので、それを打ち消す（0 で直線）
assert BEVEL_H <= FLOOR_Z - BEVEL_SEAT_CLEAR
assert BEVEL_H / BEVEL_W * (BEVEL_W + BEVEL_OVERRUN) < FLOOR_Z
assert BACK_SEAT_Z - SCREW_HEAD_H >= BEVEL_H + 0.3

# ゴム足（直径 FOOT_D のクッションゴム）を貼る位置の目印: 底面（底蓋の外面と同じ傾いた面）に彫る浅い円の溝。ゴム足を円が
# 真ん中に来るように貼ると、円はゴム足の下に隠れる（座ぐりはブリッジの天井がきれいに刷れなかったのでやめた）。位置は底面の
# 座標（bay.FRAME の x と、底面に沿って奥へ v）で、四隅に最も近く、ゴム足の縁が底面の縁・底蓋の開口・ネジ頭の穴の切り欠き
# から FOOT_EDGE_CLEAR 以上離れる点（0.5 mm 刻みで探した）
FOOT_D, FOOT_H = 7.9, 2.2   # ゴム足の直径と高さ
FOOT_MARK_D = 6.0       # 目印の円の直径（溝の中心）。ゴム足の縁より内側に隠れる
FOOT_MARK_W = 0.6       # 溝の幅（ノズル 0.4 より広く、1 層目のつぶれで埋まらないように）
FOOT_MARK_DEPTH = 0.4   # 溝の深さ（層の厚さ 0.2 の 2 層）
FOOT_EDGE_CLEAR = 4.25  # ゴム足の縁から底面の縁まで（座ぐりの縁から 4.0 だった位置のまま）
assert FOOT_MARK_D + FOOT_MARK_W <= FOOT_D - 1.0
FOOT_POS = [(9.0, -16.0), (124.5, -16.0), (9.0, -61.0), (120.0, -81.5)]   # 奥の左、奥の右、手前の左、手前の右

# 底面（底面を下にして刷る）の縁の面取り: 外周（奥の面取りとの境目を含む）、収納部の開口、手前のネジ頭の穴の口。1 層目が
# つぶれて外や内へはみ出す分（エレファントフット）で、外形が広がったり、底蓋とのすき間やネジ頭の穴（ネジ頭とのすき間は
# 片側 0.1）が狭まったりしないように。側面との境目では 45°
BED_CHAMFER = 0.4
PLUG_RECESS_CHAMFER = 0.4   # USB-C のケーブルのくぼみの口の縁の面取り

FAR = 200.0  # 外形より十分外側


def tilt_z(y, y_front):
    """傾斜カットの高さ（底面からどこまで削るか）。"""
    return TILT_FRONT_Z - TILT_SLOPE * (y - y_front)


def bevel_face(case):
    """奥の面取りの斜面（直線の断面なら円すい、円弧の断面ならトーラスの面）。"""
    return max((f for f in case.faces() if f.geom_type in (GeomType.CONE, GeomType.TORUS, GeomType.REVOLUTION)),
               key=lambda f: f.area)


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
    # 斜面は外周より外まで BEVEL_OVERRUN だけ延長して、奥の壁と稜線が重ならないようにする。斜面の断面は、下の端と延長の先を
    # 通り、中央で外（削る側）へ BEVEL_BULGE ふくらむ円弧
    _, (cx, cy), r = P.BACK
    r += L.BOTTOM_OUTER["edge"]
    slope = BEVEL_H / BEVEL_W
    p0, p1 = (r - BEVEL_W, 0), (r + BEVEL_OVERRUN, slope * (BEVEL_W + BEVEL_OVERRUN))
    rest = [Line(p1, (r + BEVEL_OVERRUN, -FAR)), Line((r + BEVEL_OVERRUN, -FAR), (r - BEVEL_W, -FAR)),
            Line((r - BEVEL_W, -FAR), p0)]
    if BEVEL_BULGE > 0:
        n = math.hypot(slope, 1)
        mid = ((p0[0] + p1[0]) / 2 + BEVEL_BULGE * slope / n, (p0[1] + p1[1]) / 2 - BEVEL_BULGE / n)
        slant = ThreePointArc(p0, mid, p1)
    else:
        slant = Line(p0, p1)
    profile = Plane.XZ * Face(Wire([slant] + rest))
    case -= Pos(cx, cy) * revolve(profile, Axis.Z)

    # 底面の外周（奥の面取りとの境目を含む）を BED_CHAMFER の面取り（R1 のフィレットの後では OCC で作れない）
    bottom = case.faces().sort_by(Axis.Z)[0]
    case = case.chamfer(BED_CHAMFER, None, bottom.outer_wire().edges())

    # 面取りと左右の側面の境目、手前の角の縦の稜線を R1
    left, right = P.LEFT[1][0], P.RIGHT[1][0]
    cone = bevel_face(case)
    sides = [e for e in cone.edges() if min(abs(e.center().X - left), abs(e.center().X - right)) < 1e-6]
    front_corners = [p for p in plan_corners(P.PLAN, L.BOTTOM_OUTER) if p.Y < -30]
    corners = case.edges().filter_by(Axis.Z).filter_by(
        lambda e: any((Vector(e.center().X, e.center().Y) - p).length < 1e-6 for p in front_corners)
    )
    case = case.fillet(1.0, sides + list(corners))

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

    # USB-C のケーブルのくぼみの口の縁（くぼみの側面と奥の面取りの斜面の境目）を PLUG_RECESS_CHAMFER の面取り。くぼみの側面は
    # FRAME の奥向き（v）に平行
    v_dir = bay.FRAME.y_dir
    bevel = bevel_face(case)
    recess = [f for f in case.faces() if abs(f.normal_at().dot(v_dir)) < 1e-6
              and abs(f.center().X - bay.BOARD_CX) < bay.PLUG_W and f.distance_to(bevel) < 1e-6]
    edges = [e for f in recess for e in f.edges() if any(e.is_same(b) for b in bevel.edges())]
    case = case.chamfer(PLUG_RECESS_CHAMFER, None, edges)

    # 収納部の開口の縁の面取り。奥の辺を除く（奥の面取りとの境目の面取りに近く、間の底面が 1 層目では線幅より細くなる。
    # 底蓋の外面の縁も面取りしてあるので、ここがはみ出しても底蓋には当たらない）
    bottom = [f for f in case.faces()
              if abs(bay.FRAME.to_local_coords(f.center()).Z) < 1e-6 and f.normal_at().dot(bay.FRAME.z_dir) < -0.999]
    face = max(bottom, key=lambda f: f.area)
    (opening,) = face.inner_wires()
    back = max(opening.edges(), key=lambda e: bay.FRAME.to_local_coords(e.center()).Y)
    case = case.chamfer(BED_CHAMFER, None, [e for e in opening.edges() if not e.is_same(back)])

    # 手前の 3 つのネジ頭の穴の口は、底面に垂直な軸の 45° の円すいで削る（右手前の穴は外周の角に近く、OCC の面取りでは
    # 作れない）。奥のネジ頭の穴は面取りの斜面の上にあり、底面には出ない
    for s in screws:
        if not s.back:
            case -= bay.FRAME * Pos(s.pos.X, bay.v_of(s.pos.Y), -1) * Solid.make_cone(
                L.BOSS_R + BED_CHAMFER + 1, L.BOSS_R - 0.5, BED_CHAMFER + 1.5)

    # ゴム足の目印
    for x, v in FOOT_POS:
        ring = bay.local_cylinder(x, v, -1, FOOT_MARK_DEPTH, FOOT_MARK_D + FOOT_MARK_W)
        ring -= bay.local_cylinder(x, v, -2, FOOT_MARK_DEPTH + 1, FOOT_MARK_D - FOOT_MARK_W)
        case -= ring
    return case


def check_feet(case):
    """ゴム足の縁と、底面の縁（目印を除いた底面の輪郭と穴）の最小距離。"""
    import bay
    local = bay.FRAME.to_local_coords(case)
    bottom = [f for f in local.faces()
              if f.geom_type == GeomType.PLANE and abs(f.normal_at().Z + 1) < 1e-6 and abs(f.center().Z) < 1e-4]
    face = max(bottom, key=lambda f: f.area)   # 目印の円の内側は、別の小さな面になる
    mark = (FOOT_MARK_D + FOOT_MARK_W) / 2
    wires = [w for w in face.inner_wires()
             if all(abs(abs(e.radius) - mark) > 1e-3 for e in w.edges() if e.geom_type == GeomType.CIRCLE)]
    wires.append(face.outer_wire())
    return min(Vertex(x, v, 0).distance_to(w) for x, v in FOOT_POS for w in wires) - FOOT_D / 2


if __name__ == "__main__":
    case = build()
    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    export_step(case, out / "bottom-l.step")
    export_stl(case, out / "bottom-l.stl")
    mesher = Mesher()  # 3D プリンタのスライサー用。単位は mm
    mesher.add_shape(case)
    mesher.write(out / "bottom-l.3mf")
    print(f"volume = {case.volume:.2f} mm^3, foot edge clearance = {check_feet(case):.2f} mm")
    # 0.5 mm 刻みで探した位置なので、少しの差は許す。底面の縁の面取りの分だけ、平らな所の縁は内へ入る
    assert check_feet(case) >= FOOT_EDGE_CLEAR - 0.3 - BED_CHAMFER
    if "--show" in sys.argv:
        from ocp_vscode import show
        show(case)
