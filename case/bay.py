# 底蓋（左手）と、ボトムケースのコントローラー収納部。電池蓋方式: 底蓋を奥から入れて手前を持ち上げて閉じ、四隅を M2 ネジ
# （M2 × 6 の六角穴付きボルト）でボトムケースのボスに埋めた M2 ナットに留める。
# - 底蓋はボトムケースの傾いた底面と面一の平らな板。コントローラー基板と電池は底蓋に平行に載る
# - 座標はプレート DXF と同じ（平面）とボトムケースの z。底蓋まわりの形状は、底蓋の外面を基準にした
#   座標系 FRAME（x はそのまま、v は底蓋に沿って奥へ、w は底蓋に垂直に上へ）でも作る
# - USB-C の口は収納部の奥の壁より奥（壁のケーブル用の穴の中）まで入る。底蓋は奥から入れる: 手前を下げて傾けた底蓋の
#   USB-C を壁の穴に差し込み、奥の端を支点にして手前を持ち上げて閉じる（まっすぐ押し込むと USB-C が壁に当たる）。
#   底蓋の外面より上の点は、底蓋の外面の高さの軸のまわりに手前を下げる向きに回すと、どれも手前へ動くので、閉じた位置で
#   奥の壁に当たらなければ途中でも当たらない。USB-C は穴の中で上下に動くので、穴の上下のすき間に収まることを確かめる
#   （usb_swing_clearance）。手前のツメを軸に回して閉める方式は、ツメが抜けかけた状態で回り始めて USB-C が奥の壁に
#   当たるのでやめた
# - 収納部の上には天井を作らず、床を抜いてボトムケースの内側の空間とつなげる（ボトムケースは傾いた底面を下にして
#   印刷するので、天井は印刷の向きで斜めの張り出しになり、サポートで支えるときれいに仕上がらない）
# - 実行: uv run python bay.py [--show]   （out/ に底蓋、ネジまわりのテストピース、合わせ確認用のボトムケースの切り出し、
#   基板の外形 DXF と、KiCad の Edge.Cuts に入れる外形の JSON を書き出す。仮のコントローラー基板と XIAO のモックは
#   controller_test_board.py）
import json
import math
import sys
from pathlib import Path

from build123d import (
    Box, Circle, Compound, Face, GeomType, Line, Mesher, Plane, Polyline, Pos, Rectangle, Solid, ThreePointArc, Wire,
    export_step, export_stl, extrude, fillet,
)

import bottom as B
import layout as L
import plate as P
from outline import path_face

A = math.atan(B.TILT_SLOPE)
_Y_FRONT = L.face(L.BOTTOM_OUTER).bounding_box().min.Y


def tilt(y):
    """ボトムケースの底面（傾いた面）の z。"""
    return B.tilt_z(y, _Y_FRONT)


# 底蓋の外面を基準にした座標系。原点は平面の y = 0 の底面上
FRAME = Plane(origin=(0, 0, tilt(0)), x_dir=(1, 0, 0), z_dir=(0, math.sin(A), math.cos(A)))


def v_of(y):
    """平面の y（底面上の点）→ FRAME の v。"""
    return y / math.cos(A)


# ボトムケースの奥の外形（円弧）
(BACK_CX, BACK_CY), BACK_R = P.BACK[1], P.BACK[2] + L.BOTTOM_OUTER["edge"]


def back_y(x, inset):
    """奥の外形から inset だけ内側の円弧の、x での y。"""
    r = BACK_R - inset
    return BACK_CY + math.sqrt(r * r - (x - BACK_CX) ** 2)


def back_arc(inset):
    return ("arc", (BACK_CX, BACK_CY), BACK_R - inset)


# ---- 寸法 ----
LID_T = 1.6                   # 底蓋の厚さ
LID_GAP = 0.2                 # 底蓋とボトムケースのすき間
LID_BACK = 5.8                # 底蓋の奥の端: 奥の外形から内側へ（面取りの斜面と、底面との境目の R0.5 から離す）
LID_FRONT_Y = -34.5           # 底蓋の手前の端（平面の y）

BOARD_W, BOARD_D, BOARD_T = 38.0, 29.0, 1.0   # コントローラー基板
BOARD_BACK = 6.5              # 基板の奥の端（XIAO の奥の端）: 奥の外形から内側へ。基板と XIAO の奥の縁と収納部の奥の壁の
                              # すき間が CONTROLLER_CLEAR 以上になる位置（基板は底蓋に平行なので上ほど奥へずれる）。
                              # USB-C の口は壁の穴に入る
CONTROLLER_CLEAR = 0.2        # 基板、XIAO の奥の縁と、収納部の奥の壁のすき間
# 基板は左右の手で同じものを使うので、外形と穴は基板の中心（x = BOARD_CX）に対して左右対称にする。
# 右手のケースは左手の鏡像なので、左右対称の基板が左手のケースに収まれば右手にも収まる
STANDOFF = 1.5                # 台座の高さ（スルーホール部品の足の逃げ）
BOARD_HOLES = [(7.0, 7.0), (31.0, 7.0), (8.0, 26.0), (30.0, 26.0)]  # 基板の左上（奥の端の中央の高さ）からの位置。
                              # 左右対称。四隅の切り欠きと底蓋のネジのボスを避け、奥の 2 つはネジ頭が XIAO に当たらない位置
BOARD_HOLE_D = 2.2            # 基板の M2 の穴
BOARD_CORNER_R = 1.0          # 基板の外形の角の丸め（出た角も入った角も）
BOSS_D, PILOT_D, PILOT_DEPTH = 5.0, 1.6, 2.4  # 台座、M2 タッピングネジ（M2 × 3）の下穴
# 電池のコネクタ（横型の PH）の足の逃げ: 足は基板を貫通して下へ出て、台座の高さだけでは底蓋に当たるので、底蓋の内面に
# 2 本の足をまとめて囲む長円の凹みを付ける。左手は J2 を使う（右手の J3 は基板の中心に対して J2 と左右対称の位置にあり、
# 鏡像の底蓋で同じく逃げられる）
BATTERY_PINS = [(-15.505, -11.495), (-13.505, -11.495)]  # J2 の足（基板の座標。KiCad の配置から）
PIN_RECESS_R = 1.3            # 凹みの半径: 足の中心から（ランド Ø1.2 のはんだと、位置のずれ）
PIN_RECESS_DEPTH = 1.0        # 凹みの深さ: 底蓋の内面から（足の出る量は要実測）

XIAO_W, XIAO_D, STACK_H = 17.8, 21.0, 5.5     # XIAO（基板の下面から XIAO と USB-C の上端まで STACK_H）
XIAO_PCB_T = 1.2              # XIAO の基板の厚さ
XIAO_EDGE_PARTS = 1.0         # XIAO の奥の端（USB-C の両脇）の部品の高さ（ボタンや LED）。USB-C より低い
USB_W, USB_H, USB_OVERHANG = 8.94, 3.26, 1.0  # USB-C のレセプタクル。XIAO の端から張り出す
USB_CLEAR = 0.3               # USB-C の通り道のすき間
PLUG_W, PLUG_H = 13.0, 7.5    # ケーブル用のくぼみ（プラグの根元が入る大きさ）
PLUG_GAP = 2.0                # くぼみの底: USB-C の口から奥へ。ケーブルを挿したとき、プラグの根元はレセプタクルに
                              # 触れずに少し離れるので、その分くぼみを浅くして収納部の奥の壁を残す（要実測）

# J1（基板の手前の端の近く、差し込み口は手前）に挿したプラグとケーブルの溝: 収納部の手前の壁から手前へ掘り込み、
# 上は床を抜いてボトムケースの内側の空間までつなげる（収納部と同じく天井を作らない）。平面形は J1 の中心（基板の中心）
# に合わせた長方形で、手前の角を丸める。手前の面は底蓋に垂直。底は基板の下面の高さで、それより下は収納部の手前の壁を
# そのまま残す（底蓋の開口の形は変えない）
J1_GROOVE_W = 20.0            # 溝の幅（x）。J1（幅約 14）+ 両側 3
J1_GROOVE_DEPTH = 8.0         # 溝の奥行き: 収納部の手前の壁から手前へ（プラグのはみ出しとケーブルの曲げ。要実測）
J1_GROOVE_R = 1.5             # 溝の手前の角の丸め

BATTERY = (31.0, 21.0, 6.0)   # 電池（幅 x、奥行き v、厚さ）。実測値
BATTERY_GAP = 2.7             # 電池の右端と基板の左端の間（リード線を通す）
BATTERY_FRONT_Y = -30.5       # 電池の手前の端（平面の y）
BATTERY_FRAME = (1.0, 2.0, 0.4)  # 電池の枠（厚さ、高さ、すき間）
BATTERY_LEAD_GAP = 8.0        # 電池の枠の基板側の辺の切り欠き: 奥の角から手前への長さ（電池のリード線を通す）
BATTERY_PRY_GAP = 6.0         # 電池の枠の奥の辺の中央の切り欠きの幅（電池を外すときにスパッジャーを差し込む）

# 底蓋のリブ: 左右の辺に沿った壁と、手前の辺の電池側の壁。底蓋に垂直に立ち、上端は底蓋に平行。
# 外側の面は底蓋の端と同じで、収納部の壁に沿って、底蓋を押し込むときのガイドになる。電池や基板を取り付けやすいよう
# 低めにする。四隅は、ボトムケースのボスが底蓋の厚肉の上面から上を占めるので、リブは角まで伸ばしたうえで、ボスに近づく所で
# 45° の斜面で厚肉の高さまで下げ、ネジ穴のある角の台（厚肉）とひとつながりにする。
# 手前の辺の基板側は、基板の手前の端が底蓋の手前の端に近いので壁を作らない（基板が台座で底蓋を補強する）。
# 電池の枠とリブの間は埋めて、枠の高さまでひとつながりにする
# 底蓋の手前の端と、収納部の手前の壁も、底蓋に垂直な面にする（左右の辺はもともと底蓋に垂直、奥の辺は鉛直のまま）
RIB_T = 1.2                   # 厚さ
RIB_H = 4.0                   # 高さ（底蓋の内面から）
RIB_CHAMFER = 0.4             # リブの上端の角の面取り
# 奥のリブ: 奥の辺（円弧）に沿った壁。基板の横幅（+ RIB_BOARD_CLEAR）の範囲は、基板の下面より低い高さ
# （底蓋の外面から RIB_LOW_W）にして、基板の奥の縁、XIAO、USB-C の口に当たらないようにする。高さの境目は 45° の斜面
RIB_BOARD_CLEAR = 0.5         # 低くする範囲: 基板の左右の端から外へ
RIB_LOW_W = LID_T + STANDOFF - 0.1  # 低くする所の上端（底蓋の外面から。基板の下面の 0.1 下）
RIB_BOSS_CLEAR = 0.3          # リブと電池の枠の斜面の下端と、ボトムケースのボスのすき間

# 四隅のネジ: 底蓋の各辺から LID_SCREW_INSET 内側。ボトムケースのボスは床の上面の高さから底蓋の内面まで下ろし、
# 隣り合う 2 つの壁につなげる。ボスの下面が底蓋の内面に当たる位置で、底蓋が底面と面一になる（4 か所とも）。
# 右の 2 か所は基板の角の切り欠きに入れ、左の 2 か所は電池の左横に置く（底蓋の左端は、左のボスの位置で決まる）
LID_SCREW_INSET = 3.2         # ネジの中心: 底蓋の端から内側へ（座ぐりの縁に 1 mm 残す）
LID_PAD = (6.4, 1.6)          # 底蓋の内側のネジまわりの厚肉（直径、底蓋の内面からの高さ）。ネジ頭の座ぐりのため。
                              # 平面形はボトムケースのボスと同じ（円と、隣り合う 2 つの辺までの四角）で、角を埋める
SCREW_HEAD = (4.4, 2.0)       # ネジ頭の座ぐり（直径、深さ）。M2 の六角穴付きボルト（Ø3.8 × 2.0）の頭が沈む
SCREW_LEN = 6.0               # ネジの長さ（M2 × 6）。先端は底蓋の外面から SCREW_HEAD[1] + SCREW_LEN。
                              # 手前のボスの高さ（床の上面まで）で決まる
SCREW_CLEAR_D = 2.4           # ネジの通し穴（底蓋とボス）
SCREW_BOSS_D = 6.4            # ボトムケースのボス
# ナットの溝: 収納部の内側（左のボスは右、右のボスは左）から横に差し込む。平らな面を手前と奥に向け、溝の幅は二面幅。
# 底蓋を外すと基板と電池も一緒に外れるので、収納部は空になって溝に手が届く
NUT_AF = 4.2                  # 二面幅（M2 ナット 4.0 mm + 0.2。トップケースのナットのポケットと同じ）
NUT_T = 1.8                   # 溝の厚さ（M2 ナット 1.6 mm + 0.2）
NUT_TIP = 0.8                 # ネジの先端がナットの上から出る長さ
BOSS_CLEAR = 0.5              # ボスと電池のすき間
BOARD_NOTCH_CLEAR = 0.8       # ボスと基板の切り欠きのすき間（基板は底蓋に平行で高い所ほど奥へずれる分も見込む）

# 基板の位置: USB-C とケーブルのくぼみの位置は、ケースの左右の中央から右へ 18.55（以前の電池と基板の配置で決めた位置）。
# 電池は基板の左に BATTERY_GAP 離して置く。底蓋の右端は基板から LID_MARGIN、左端は左のボスの位置で決まる
LID_CX = (P.LEFT[1][0] + P.RIGHT[1][0]) / 2   # ケースの左右の中央
BOARD_CX = LID_CX + 18.55     # 基板の中心の x（= USB-C の位置）
LID_MARGIN = 2.0
BATTERY_X0 = BOARD_CX - BOARD_W / 2 - BATTERY_GAP - BATTERY[0]  # 電池の左端
_left_screw_x = BATTERY_X0 - BOSS_CLEAR - SCREW_BOSS_D / 2
LID_X = (_left_screw_x - LID_SCREW_INSET, BOARD_CX + BOARD_W / 2 + LID_MARGIN)  # 底蓋の左右の端
# USB-C（XIAO）はケースの奥の中央のネジ（x = LID_CX）を避ける必要がある: XIAO の左端とネジの穴の間に 1 mm 以上
assert BOARD_CX - XIAO_W / 2 - (LID_CX + L.BOSS_R) >= 1.0

# 電池の上の手前の角が床の上面（マトリクス基板の側の空間の底）より下にあること。右のリブが基板から 0.5 mm 以上離れること
assert FRAME.from_local_coords((0, v_of(BATTERY_FRONT_Y), LID_T + BATTERY[2])).Z <= B.FLOOR_Z - 0.2
assert LID_X[1] - RIB_T - (BOARD_CX + BOARD_W / 2) >= 0.5

FAR = 60.0


# ---- 平面形 ----

def board_v_top():
    """基板の奥の辺のうち、XIAO が載る直線部分の v。XIAO の両端のうち、奥の外形の円弧から遠い側で BOARD_BACK になる位置。"""
    x_far = BOARD_CX + XIAO_W / 2 if BOARD_CX >= BACK_CX else BOARD_CX - XIAO_W / 2
    return v_of(back_y(x_far, BOARD_BACK))


def lid_screws_local():
    """四隅のネジの中心（FRAME の x, v）と、どの角か: [(x, v, 左右 -1/+1, 手前 -1 / 奥 +1)]。
    奥の 2 つは奥の外形の円弧から LID_SCREW_INSET 内側、手前の 2 つは手前の端から LID_SCREW_INSET 内側。"""
    xl, xr = LID_X[0] + LID_SCREW_INSET, LID_X[1] - LID_SCREW_INSET
    vf = v_of(LID_FRONT_Y) + LID_SCREW_INSET
    out = []
    for x, sx in ((xl, -1), (xr, 1)):
        out.append((x, vf, sx, -1))
        out.append((x, v_of(back_y(x, LID_BACK + LID_SCREW_INSET)), sx, 1))
    return out


def boss_center(x, v):
    """ボトムケースのボスの中心（平面）: ボスの下面（底蓋の厚肉の上面）の高さでのネジの軸の位置。
    ボスは底蓋に垂直な柱なので、上ほど奥へずれる。"""
    p = FRAME.from_local_coords((x, v, LID_T + LID_PAD[1]))
    return p.X, p.Y


def board_notch():
    """基板の右の角の切り欠き: (切り欠きの左端の x, 手前の切り欠きの奥の端の y, 奥の切り欠きの手前の端の y)。
    右の 2 つのボスから決め、左の角は鏡像にする。"""
    r = SCREW_BOSS_D / 2 + BOARD_NOTCH_CLEAR
    (xf, vf), (xb, vb) = [(x, v) for x, v, sx, _ in lid_screws_local() if sx > 0]
    (fx, fy), (bx, by) = boss_center(xf, vf), boss_center(xb, vb)
    return min(fx, bx) - r, fy + r, by - r


def board_path():
    """基板の外形（平面）: [(点, 次の点までの辺)]。基板の中心に対して左右対称。
    - 奥の辺は、XIAO の幅だけ直線（XIAO の端と USB-C を合わせる）、その両側の肩は円弧。右の肩（奥の外形の中心から遠く、
      奥の外形がより手前に来る側）は奥の外形に沿い、左の肩はその鏡像
    - 四隅は、底蓋のネジのボスを逃がす長方形の切り欠き（左手のケースでは右の 2 つ、右手では左の 2 つを使う）
    - 手前の辺は直線"""
    x0, x1 = BOARD_CX - BOARD_W / 2, BOARD_CX + BOARD_W / 2
    xl, xr = BOARD_CX - XIAO_W / 2, BOARD_CX + XIAO_W / 2
    assert BOARD_CX >= BACK_CX
    xn, yfn, ybn = board_notch()
    xnr, xnl = xn, 2 * BOARD_CX - xn
    assert xnl < xl and xr < xnr

    def shoulder_y(x):
        return back_y(x if x >= BOARD_CX else 2 * BOARD_CX - x, BOARD_BACK)

    arc_r = back_arc(BOARD_BACK)
    arc_l = ("arc", (2 * BOARD_CX - BACK_CX, BACK_CY), BACK_R - BOARD_BACK)
    y_top = board_v_top() * math.cos(A)
    y_front = y_top - BOARD_D * math.cos(A)
    assert y_front < yfn < ybn < shoulder_y(xnr)
    path = [
        ((xnl, y_front), None),
        ((xnr, y_front), None),
        ((xnr, yfn), None),
        ((x1, yfn), None),
        ((x1, ybn), None),
        ((xnr, ybn), None),
        ((xnr, shoulder_y(xnr)), arc_r),
        ((xr, shoulder_y(xr)), None),
        ((xr, y_top), None),
        ((xl, y_top), None),
        ((xl, shoulder_y(xl)), arc_l),
        ((xnl, shoulder_y(xnl)), None),
        ((xnl, ybn), None),
        ((x0, ybn), None),
        ((x0, yfn), None),
        ((xnl, yfn), None),
    ]
    out = []
    for p, c in path:  # 長さ 0 の辺（直線部分の端が円弧と一致する所）を除く
        if out and abs(p[0] - out[-1][0][0]) < 1e-6 and abs(p[1] - out[-1][0][1]) < 1e-6:
            out[-1] = (out[-1][0], c)
            continue
        out.append((p, c))
    return out


def board_outline():
    """コントローラー基板の外形（平面）。"""
    return path_face(board_path())


def lid_region(grow=0.0):
    """底蓋の範囲の立体: 左右と奥は鉛直（平面形を押し出したもの）、手前は底蓋に垂直な面。grow だけ外へ広げる。"""
    x0, x1 = LID_X[0] - grow, LID_X[1] + grow
    inset = LID_BACK - grow
    yf = LID_FRONT_Y - 10   # 手前は底蓋に垂直な面で切る
    face = path_face([
        ((x0, yf), None),
        ((x1, yf), None),
        ((x1, back_y(x1, inset)), back_arc(inset)),
        ((x0, back_y(x0, inset)), None),
    ])
    return prism(face) & local_box(-1000, 1000, v_of(LID_FRONT_Y) - grow, 1000, -100, 100)


def battery_outline(grow=0.0):
    l, w, _ = BATTERY
    yc = BATTERY_FRONT_Y + w / 2 * math.cos(A)
    return Pos(BATTERY_X0 + l / 2, yc) * Rectangle(l + 2 * grow, w * math.cos(A) + 2 * grow)


def rib_x_front():
    """手前のリブの右端: 電池の枠の右の外面にそろえる。"""
    return BATTERY_X0 + BATTERY[0] + BATTERY_FRAME[2] + BATTERY_FRAME[0]


# ---- 3D の部品 ----

def prism(face, z0=-30.0, z1=30.0):
    """平面形を z 方向に押し出した柱。"""
    return Pos(0, 0, z0) * extrude(face, z1 - z0)


def slab(w0, w1):
    """底蓋に平行な板状の領域（FRAME の w0..w1）。"""
    return extrude(FRAME.offset(w0) * Rectangle(400, 400), w1 - w0)


def local_box(x0, x1, v0, v1, w0, w1):
    """FRAME の座標で直方体。"""
    return FRAME * Pos((x0 + x1) / 2, (v0 + v1) / 2, w0) * extrude(Rectangle(x1 - x0, v1 - v0), w1 - w0)


def local_cylinder(x, v, w0, w1, d):
    return FRAME * Pos(x, v, w0) * Solid.make_cylinder(d / 2, w1 - w0)


def corner_footprint(x, v, sx, sv, r):
    """角のネジのまわりの平面形（FRAME の x, v）: 半径 r の円と、そこから隣り合う 2 つの壁（収納部の壁から 1 mm 外）までの
    長方形。ボトムケースのボス（r = SCREW_BOSS_D / 2）と、底蓋の角の厚肉に使う。"""
    if sv > 0:
        v_wall = v_of(back_y(x, LID_BACK - LID_GAP)) + 1.0   # 底蓋の外面の高さで、奥の壁から 1 mm 奥
    else:
        v_wall = v_of(LID_FRONT_Y) - LID_GAP - 1.0
    x_wall = (LID_X[1] + LID_GAP + 1.0) if sx > 0 else (LID_X[0] - LID_GAP - 1.0)
    footprint = Pos(x, v) * Circle(r)
    footprint += Pos(x, (v + v_wall) / 2) * Rectangle(2 * r, abs(v_wall - v))
    footprint += Pos((x + x_wall) / 2, v) * Rectangle(abs(x_wall - x), 2 * r)
    footprint += Pos((x + x_wall) / 2, (v + v_wall) / 2) * Rectangle(abs(x_wall - x), abs(v_wall - v))
    return footprint


def boss_flare(x, v, sx, sv, clear):
    """底蓋の側で、角のボスのまわりを避ける形（FRAME の座標）: ボスの平面形を clear だけ広げ、厚肉の上面から上へ、
    上ほど 45° で広がるように押し出したもの。リブや電池の枠はこれで削ると、ボスに近づく所で 45° の斜面になって
    厚肉の高さまで下がり、角の台とひとつながりになる。"""
    face = corner_footprint(x, v, sx, sv, SCREW_BOSS_D / 2 + clear)
    return FRAME * Pos(0, 0, LID_T + LID_PAD[1]) * extrude(face, 10, taper=-45)


def back_rib_band():
    """奥のリブの平面形の範囲（鉛直な柱）: 底蓋の奥の端から RIB_T 内側まで。基板の横幅の範囲は RIB_LOW_W より上を
    45° の斜面で削る。"""
    xa, xb = LID_X[0] - 2, LID_X[1] + 2
    inset = LID_BACK + RIB_T
    inner = path_face([
        ((xa, LID_FRONT_Y - 10), None),
        ((xb, LID_FRONT_Y - 10), None),
        ((xb, back_y(xb, inset)), back_arc(inset)),
        ((xa, back_y(xa, inset)), None),
    ])
    band = lid_region() - prism(inner)
    lo, hi = BOARD_CX - BOARD_W / 2 - RIB_BOARD_CLEAR, BOARD_CX + BOARD_W / 2 + RIB_BOARD_CLEAR
    low = FRAME * Pos((lo + hi) / 2, 0, RIB_LOW_W) * extrude(Rectangle(hi - lo, 400), 10, taper=-45)
    return band - low


def ribs():
    """底蓋のリブ（底蓋に垂直に立ち、上端は底蓋に平行。四隅のボスに近づく所は 45° の斜面で厚肉の高さまで下げ、
    上端の角を面取り）。左右、手前（電池の側）、奥の辺に沿う。"""
    x0, x1 = LID_X
    vf = v_of(LID_FRONT_Y)
    bands = local_box(x0 - 1, x0 + RIB_T, vf - 1, 1000, -100, 100)
    bands += local_box(x1 - RIB_T, x1 + 1, vf - 1, 1000, -100, 100)
    bands += local_box(x0 - 1, rib_x_front(), vf - 1, vf + RIB_T, -100, 100)
    bands = (bands & lid_region()) + back_rib_band()
    for s in lid_screws_local():
        bands -= boss_flare(*s, RIB_BOSS_CLEAR)
    top = LID_T + RIB_H
    rib = bands & slab(LID_T - 0.1, top)
    out = []
    for solid in rib.solids():
        # 上端の面（いちばん高い、底蓋に平行な面）の縁を面取りする
        tops = [f for f in solid.faces() if abs(FRAME.to_local_coords(f.center()).Z - top) < 1e-3]
        out.append(solid.chamfer(RIB_CHAMFER, None, [e for f in tops for e in f.edges()]))
    return out


def board_holes_local():
    x0, vt = BOARD_CX - BOARD_W / 2, board_v_top()
    return [(x0 + dx, vt - dv) for dx, dv in BOARD_HOLES]


def usb_axis_w():
    return LID_T + STANDOFF + BOARD_T + XIAO_PCB_T + USB_H / 2


def lid():
    """底蓋。"""
    body = lid_region() & slab(0, LID_T)

    # リブ
    for rib in ribs():
        body += rib

    # 台座（M2 タッピングネジの下穴つき）
    for x, v in board_holes_local():
        body += local_cylinder(x, v, LID_T - 0.1, LID_T + STANDOFF, BOSS_D)
        body -= local_cylinder(x, v, LID_T + STANDOFF - PILOT_DEPTH, LID_T + STANDOFF + 0.1, PILOT_D)

    # 電池の枠。左と手前は、リブとの間を埋めて底蓋の端までつなげる。左のボスに近づく所は、リブと同じく 45° の斜面で
    # 厚肉の高さまで下げ、角の台とつなげる。切り欠きは 2 つ:
    # - 基板側の辺の奥の角から BATTERY_LEAD_GAP（電池のリード線を奥の角から基板の側へ出す）
    # - 奥の辺の中央に幅 BATTERY_PRY_GAP（電池を外すときにスパッジャーを差し込む）
    t, h, c = BATTERY_FRAME
    outer = battery_outline(c + t).bounding_box()
    inner = battery_outline(c).bounding_box()
    x0, y0 = LID_X[0] - 1, LID_FRONT_Y - 1
    block = Pos((x0 + outer.max.X) / 2, (y0 + outer.max.Y) / 2) * Rectangle(outer.max.X - x0, outer.max.Y - y0)
    ring = block - battery_outline(c)
    ya, yb = inner.max.Y - BATTERY_LEAD_GAP, outer.max.Y + 1
    lead_gap = Pos((inner.max.X + outer.max.X + 1) / 2, (ya + yb) / 2) * Rectangle(outer.max.X + 1 - inner.max.X, yb - ya)
    xc = battery_outline().center().X
    pry_gap = Pos(xc, (inner.max.Y - 1 + yb) / 2) * Rectangle(BATTERY_PRY_GAP, yb - inner.max.Y + 1)
    frame = prism(ring - lead_gap - pry_gap) & slab(LID_T - 0.1, LID_T + h) & lid_region()
    for s in lid_screws_local():
        frame -= boss_flare(*s, RIB_BOSS_CLEAR)
    body += frame

    # 四隅のネジ: 角を埋める厚肉（ボスと同じ平面形を底蓋の外形で切る）、ネジ頭の座ぐり、通し穴
    for x, v, sx, sv in lid_screws_local():
        pad = FRAME * Pos(0, 0, LID_T - 0.1) * extrude(corner_footprint(x, v, sx, sv, LID_PAD[0] / 2), LID_PAD[1] + 0.1)
        body += pad & lid_region()
        body -= local_cylinder(x, v, -1, SCREW_HEAD[1], SCREW_HEAD[0])
        body -= local_cylinder(x, v, -1, LID_T + LID_PAD[1] + 1, SCREW_CLEAR_D)

    # 電池のコネクタの足の凹み
    body -= FRAME * Pos(0, 0, LID_T - PIN_RECESS_DEPTH) * extrude(pin_recess_face(), PIN_RECESS_DEPTH + 1)
    return body


def battery_pins_local():
    """電池のコネクタの足の中心（FRAME の x, v）。"""
    vt = board_v_top()
    return [(BOARD_CX + x, vt + y) for x, y in BATTERY_PINS]


def pin_recess_face():
    """電池のコネクタの足の凹みの平面形（FRAME の x, v）: 足の並びに沿った長円。"""
    (x0, v0), (x1, v1) = battery_pins_local()
    assert abs(v0 - v1) < 1e-9
    face = Pos(x0, v0) * Circle(PIN_RECESS_R) + Pos(x1, v1) * Circle(PIN_RECESS_R)
    return face + Pos((x0 + x1) / 2, v0) * Rectangle(abs(x1 - x0), 2 * PIN_RECESS_R)


# 凹みが基板の台座から離れていること、底蓋に厚さが残ること
assert min(math.hypot(px - x, pv - v) for px, pv in battery_pins_local() for x, v in board_holes_local()) \
    >= BOSS_D / 2 + PIN_RECESS_R + 0.5
assert LID_T - PIN_RECESS_DEPTH >= 0.6


def case_cuts():
    """ボトムケースから削る形状。"""
    cuts = []
    # 収納部（底蓋の開口から、床を抜いてボトムケースの内側の空間まで）
    cuts.append(lid_region(LID_GAP) & prism(Rectangle(1000, 1000), -30, B.HEIGHT + 1))
    # 奥の壁を抜けてケーブルのプラグが通る穴（USB-C の口の先が入る。収納部の中から、くぼみまで）と、面取りの斜面のくぼみ
    # （底は口から PLUG_GAP 奥）。くぼみは USB-C の口に合わせて決まるので、基板を奥へ寄せた分だけ浅くなり、収納部とくぼみの
    # 間の壁が厚くなる
    vt, wc = board_v_top(), usb_axis_w()
    mouth = vt + USB_OVERHANG
    hw, hh = usb_hole_half()
    cuts.append(local_box(BOARD_CX - hw, BOARD_CX + hw, vt - 1, mouth + PLUG_GAP + 0.5, wc - hh, wc + hh))
    cuts.append(local_box(BOARD_CX - PLUG_W / 2, BOARD_CX + PLUG_W / 2, mouth + PLUG_GAP, mouth + 20,
                          wc - PLUG_H / 2, wc + PLUG_H / 2))
    cuts.append(j1_groove())
    return cuts


def j1_groove():
    """J1 のプラグとケーブルの溝（FRAME の w の向きの柱）。奥は収納部の中まで伸ばして、収納部の手前の壁に重ねる。"""
    vf = v_of(LID_FRONT_Y) - LID_GAP
    v0, v1 = vf - J1_GROOVE_DEPTH, vf + 2 * J1_GROOVE_R
    rect = Pos(BOARD_CX, (v0 + v1) / 2) * Rectangle(J1_GROOVE_W, v1 - v0)
    face = fillet(rect.vertices(), J1_GROOVE_R).faces()[0]
    w0 = LID_T + STANDOFF
    return (FRAME * Pos(0, 0, w0) * extrude(face, 40)) & prism(Rectangle(1000, 1000), -30, B.HEIGHT + 1)


# J1 の溝が手前の右のボス（とナットの溝の口）から離れていること
assert BOARD_CX + J1_GROOVE_W / 2 <= LID_X[1] - LID_SCREW_INSET - SCREW_BOSS_D / 2 - 2.0 - 1.0


def boss_solid(x, v, sx, sv):
    """ボトムケースの角のボス: 円と、そこから隣り合う 2 つの壁へ入り込む長方形（FRAME の w の向きの柱）。
    下面は底蓋の厚肉の上面、上端は床の上面の高さ。奥の壁は鉛直なので、上ほど深く入る。"""
    w0 = LID_T + LID_PAD[1]
    footprint = corner_footprint(x, v, sx, sv, SCREW_BOSS_D / 2)
    return (FRAME * Pos(0, 0, w0) * extrude(footprint, 30)) & prism(Rectangle(1000, 1000), -30, B.FLOOR_Z)


def nut_pocket(x, v, sx):
    """ナットの溝: 六角形の外側（壁側）の半分と、そこから収納部の内側へ伸ばした溝を 1 つの多角形で描く
    （六角形と長方形の和にすると、重なった辺が残って 3MF のメッシュが壊れる）。平らな面は手前と奥。"""
    r = NUT_AF / math.sqrt(3)
    reach = SCREW_BOSS_D / 2 + 2.0   # 溝の口: ボスの内側の面より先まで
    s = sx   # 六角形の頂点を壁の側（左のボスは左、右のボスは右）に向ける
    pts = [(s * r, 0), (s * r / 2, NUT_AF / 2), (-s * reach, NUT_AF / 2), (-s * reach, -NUT_AF / 2), (s * r / 2, -NUT_AF / 2)]
    tip = SCREW_HEAD[1] + SCREW_LEN
    nut_top = tip - NUT_TIP
    # 左のボスでは輪郭が時計回りになり、面の法線が下を向くので、押し出す向きを固定する
    face = Face(Wire(Polyline(*pts, close=True).edges()))
    return FRAME * Pos(x, v, nut_top - NUT_T) * extrude(face, NUT_T, dir=(0, 0, 1))


def add_screw_bosses(case):
    """収納部を削った後のボトムケースに、四隅のボスを足して、ネジの通し穴とナットの溝を開ける。
    ボス、穴、溝はどれも底蓋に垂直（FRAME の w の向き）。"""
    for x, v, sx, sv in lid_screws_local():
        case += boss_solid(x, v, sx, sv)
    tip = SCREW_HEAD[1] + SCREW_LEN
    w0 = LID_T + LID_PAD[1]
    for x, v, sx, _ in lid_screws_local():
        case -= local_cylinder(x, v, w0 - 1, tip + 0.4, SCREW_CLEAR_D)
        case -= nut_pocket(x, v, sx)
    return case


def boss_top_w():
    """ボスの上端（床の上面）の、ネジの軸での高さ（底蓋の外面から垂直に）のうち、いちばん低いもの（手前のボス）。"""
    out = []
    for x, v, _, _ in lid_screws_local():
        base = FRAME.from_local_coords((x, v, 0))
        out.append((B.FLOOR_Z - base.Z) / math.cos(A))
    return min(out)


# ネジの通し穴（先端 + 0.4）がボスの上端（床の上面）より下に収まること
assert SCREW_HEAD[1] + SCREW_LEN + 0.4 <= boss_top_w() - 0.3


def usb_hole_half():
    """奥の壁の USB-C の穴の、幅と高さの半分（FRAME の x, w）。"""
    return USB_W / 2 + USB_CLEAR, USB_H / 2 + USB_CLEAR


def controller_back_points():
    """基板の奥側の縁（上下の面）と XIAO の奥の角の点（FRAME の x, v, w）。USB-C は壁の穴に入るので含めない。"""
    w_b = LID_T + STANDOFF
    pts = []
    path = board_path()
    for i, (p0, c) in enumerate(path):   # 基板の外形を細かくたどる（奥側だけ）
        p1 = path[(i + 1) % len(path)][0]
        for k in range(21):
            t = k / 20
            if c is None:
                x, y = p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t
            else:
                (cx, cy), r = c[1:]
                a0, a1 = math.atan2(p0[1] - cy, p0[0] - cx), math.atan2(p1[1] - cy, p1[0] - cx)
                x, y = cx + r * math.cos(a0 + (a1 - a0) * t), cy + r * math.sin(a0 + (a1 - a0) * t)
            if y > -15:
                pts += [(x, v_of(y), w_b), (x, v_of(y), w_b + BOARD_T)]
    w_x = w_b + BOARD_T + XIAO_PCB_T + XIAO_EDGE_PARTS
    pts += [(BOARD_CX + s * XIAO_W / 2, board_v_top(), w_x) for s in (-1, 1)]
    return pts


def swing(p, angle, pivot_v):
    """FRAME の点 (x, v, w) を、底蓋の外面の高さ（w = 0）で v = pivot_v の、x に平行な軸のまわりに、手前を下げる向きに
    angle（ラジアン）回した点。"""
    x, v, w = p
    dv, c, s = v - pivot_v, math.cos(angle), math.sin(angle)
    return x, pivot_v + dv * c - w * s, dv * s + w * c


def wall_gap(p):
    """FRAME の点と、収納部の奥の壁（鉛直）の y 方向の距離。正なら壁の手前。"""
    x = p[0]
    return back_y(x, LID_BACK - LID_GAP) - FRAME.from_local_coords(p).Y


def controller_wall_clearance():
    """基板の奥側の縁と XIAO の奥の角が、閉じた位置で収納部の奥の壁から離れている最小の距離（y 方向）。
    底蓋の外面より上の点は、底蓋を開ける向き（手前を下げる向き）に回すと手前へ動くので、閉じた位置だけを見ればよい。"""
    return min(wall_gap(p) for p in controller_back_points())


def usb_swing_clearance(steps=200):
    """底蓋を奥の端（底蓋の外面の高さ、USB-C の位置の奥の端）を支点にして開けていき、USB-C の口の先が奥の壁から抜けるまでの
    間の、USB-C と壁の穴の上下の面の最小のすき間と、抜けたときの角度（度）。USB-C は支点より奥にあるので、開けると
    手前へ動きながら上へ動く。"""
    vt, wc = board_v_top(), usb_axis_w()
    hw, hh = usb_hole_half()
    pivot_v = v_of(back_y(BOARD_CX, LID_BACK))
    # USB-C の上下の 4 つの稜線を、口から手前へ細かくたどる
    pts = [(BOARD_CX + sx * USB_W / 2, vt + USB_OVERHANG - k * 0.1, wc + sw * USB_H / 2)
           for sx in (-1, 1) for sw in (-1, 1) for k in range(31)]
    gap = hh - USB_H / 2
    for i in range(steps + 1):
        angle = math.radians(20) * i / steps
        inside = [q for q in (swing(p, angle, pivot_v) for p in pts) if wall_gap(q) < 0]
        if not inside:
            return gap, math.degrees(angle)
        gap = min([gap] + [min(q[2] - (wc - hh), (wc + hh) - q[2]) for q in inside])
    raise AssertionError("USB-C が壁の穴から抜けない")


def dummies():
    """プレビュー用: コントローラー基板、XIAO、USB-C、電池。"""
    w_b = LID_T + STANDOFF
    board = FRAME * Pos(BOARD_CX, board_v_top(), w_b) * extrude(board_face_local(), BOARD_T)
    for x, v in board_holes_local():
        board -= local_cylinder(x, v, w_b - 1, w_b + BOARD_T + 1, BOARD_HOLE_D)
    vt = board_v_top()
    # XIAO: 基板と、奥の端の小さな部品の高さまでの箱（USB-C は別）
    xiao = local_box(BOARD_CX - XIAO_W / 2, BOARD_CX + XIAO_W / 2, vt - XIAO_D, vt,
                     w_b + BOARD_T, w_b + BOARD_T + XIAO_PCB_T + XIAO_EDGE_PARTS)
    wc = usb_axis_w()
    usb = local_box(BOARD_CX - USB_W / 2, BOARD_CX + USB_W / 2, vt - 7, vt + USB_OVERHANG, wc - USB_H / 2, wc + USB_H / 2)
    battery = prism(battery_outline()) & slab(LID_T, LID_T + BATTERY[2])
    return board, xiao, usb, battery


# 基板と XIAO が収納部の奥の壁に当たらないこと。底蓋を開け閉めする途中で、USB-C が壁の穴の上下の面に当たらないこと
assert controller_wall_clearance() >= CONTROLLER_CLEAR - 1e-6
assert usb_swing_clearance()[0] >= 0.1


def screw_test(case, lid_part):
    """ネジまわりのテストピース: 右手前の角（いちばん低いボス、基板の切り欠き）のまわりだけを切り出したボトムケース
    （床の上面まで）と底蓋。"""
    x, v, _, _ = [s for s in lid_screws_local() if s[2] > 0 and s[3] < 0][0]
    cx, cy = boss_center(x, v)
    region = Pos(cx - 4, cy + 2) * Rectangle(20, 18)
    return case & prism(region, -30, B.FLOOR_Z), lid_part & prism(region)


FIT_TEST_MARGIN = 6.0         # 合わせ確認用の切り出し: 底蓋の範囲から手前と左右へ広げる量


def fit_test(case):
    """底蓋とボトムケースの合わせ確認用: ボトムケースのうち、底蓋のまわり（手前と左右に FIT_TEST_MARGIN、奥は外面まで。
    手前は J1 の溝の手前の端からも FIT_TEST_MARGIN / 2）を、底面から床の上面までの高さで切り出したもの。
    床の上面を下にして印刷できる。"""
    m = FIT_TEST_MARGIN
    x0, x1 = LID_X[0] - m, LID_X[1] + m
    y0, y1 = LID_FRONT_Y - max(m, J1_GROOVE_DEPTH + m / 2), 20.0
    region = Pos((x0 + x1) / 2, (y0 + y1) / 2) * Rectangle(x1 - x0, y1 - y0)
    return case & prism(region, -30, B.FLOOR_Z)


def board_local_segments():
    """コントローラー基板の外形を、基板の面の座標（mm、原点は奥の辺の直線部分の中央、y は奥が正）で表した辺の並び:
    [(始点, 円弧の中点 または None, 終点)]。円弧は平面の円弧の中点を写した 3 点で表す（基板の面に写すと y 方向に
    1 / cos(6°) 伸びて厳密には楕円になるが、差は 0.01 mm 未満）。"""
    vt = board_v_top()

    def local(p):  # 平面の点（底面上）→ 基板の面の座標
        return (p[0] - BOARD_CX, v_of(p[1]) - vt)

    outline = board_path()
    segs = []
    for i, (p0, c) in enumerate(outline):
        p1 = outline[(i + 1) % len(outline)][0]
        m = None
        if c is not None:
            (cx, cy), r0 = c[1:]
            am = (math.atan2(p0[1] - cy, p0[0] - cx) + math.atan2(p1[1] - cy, p1[0] - cx)) / 2
            m = local((cx + r0 * math.cos(am), cy + r0 * math.sin(am)))
        segs.append((local(p0), m, local(p1)))
    return segs


def board_face_local():
    """コントローラー基板の外形（基板の面の座標）。角を BOARD_CORNER_R で丸めたもの。"""
    edges = [Line(a, b) if m is None else ThreePointArc(a, m, b) for a, m, b in board_local_segments()]
    return fillet(Face(Wire(edges)).vertices(), BOARD_CORNER_R).faces()[0]


def board_outline_segments():
    """角を丸めたコントローラー基板の外形の辺の並び（基板の面の座標）: [(始点, 円弧の中点 または None, 終点)]。
    DXF と KiCad の Edge.Cuts に書き出す用。"""
    out = []
    for e in board_face_local().outer_wire().edges():
        a, b = ((e @ 0).X, (e @ 0).Y), ((e @ 1).X, (e @ 1).Y)
        m = None if e.geom_type == GeomType.LINE else ((e @ 0.5).X, (e @ 0.5).Y)
        out.append((a, m, b))
    return out


def board_holes_board():
    """コントローラー基板の M2 の穴の中心（基板の面の座標）。"""
    vt = board_v_top()
    return [(x - BOARD_CX, v - vt) for x, v in board_holes_local()]


def xiao_mock():
    """組み付け確認用の XIAO と USB-C のモック: XIAO の基板と奥の端の小さな部品の高さまでの箱に USB-C を載せたもの
    （USB-C は XIAO の奥の端から USB_OVERHANG 張り出す）。基板の面の座標で、下面を z = 0 に置く（印刷の向き）。
    仮のコントローラー基板は controller_test_board.py（KiCad の穴つき）で作る。"""
    h = XIAO_PCB_T + XIAO_EDGE_PARTS
    xiao = Pos(0, -XIAO_D / 2, h / 2) * Box(XIAO_W, XIAO_D, h)
    usb_len = 7.0 + USB_OVERHANG
    xiao += Pos(0, USB_OVERHANG - usb_len / 2, XIAO_PCB_T + USB_H / 2) * Box(USB_W, usb_len, USB_H)
    return xiao


def export_board_dxf(path, hole_d=BOARD_HOLE_D):
    """コントローラー基板の外形と M2 の穴を、基板の面の座標で DXF に書き出す。KiCad の Edge.Cuts に読み込む用。"""
    import ezdxf
    doc = ezdxf.new("R2010")
    doc.units = ezdxf.units.MM
    msp = doc.modelspace()
    for a, m, b in board_outline_segments():
        if m is None:
            msp.add_line(a, b)
            continue
        # 3 点を通る円弧
        (ax, ay), (bx, by), (mx, my) = a, b, m
        d = 2 * (ax * (my - by) + mx * (by - ay) + bx * (ay - my))
        ux = ((ax**2 + ay**2) * (my - by) + (mx**2 + my**2) * (by - ay) + (bx**2 + by**2) * (ay - my)) / d
        uy = ((ax**2 + ay**2) * (bx - mx) + (mx**2 + my**2) * (ax - bx) + (bx**2 + by**2) * (mx - ax)) / d
        r = math.hypot(ax - ux, ay - uy)
        s0, s1 = math.degrees(math.atan2(ay - uy, ax - ux)), math.degrees(math.atan2(by - uy, bx - ux))
        if (s1 - s0) % 360 > 180:  # 反時計回りに短い方を描く
            s0, s1 = s1, s0
        msp.add_arc((ux, uy), r, s0, s1)
    for x, y in board_holes_board():
        msp.add_circle((x, y), hole_d / 2)
    doc.saveas(path)


def export(shape_or_parts, path):
    parts = shape_or_parts if isinstance(shape_or_parts, list) else [("part", shape_or_parts)]
    export_step(Compound([s for _, s in parts]), path.with_suffix(".step"))
    export_stl(Compound([s for _, s in parts]), path.with_suffix(".stl"))
    m = Mesher()  # 3D プリンタのスライサー用。単位は mm
    for name, s in parts:
        m.add_shape(s, part_number=name)
    m.write(path.with_suffix(".3mf"))


if __name__ == "__main__":
    lid_part = lid()
    case = B.build()
    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    export(lid_part, out / "lid-l")
    case_piece, lid_piece = screw_test(case, lid_part)
    export([("screw-test-case", case_piece), ("screw-test-lid", Pos(0, -25, 0) * lid_piece)], out / "screw-test-l")
    export([("bay-fit-test-case", fit_test(case))], out / "bay-fit-test-l")
    export_board_dxf(out / "controller-board-l.dxf")
    # KiCad の Edge.Cuts に入れる外形（scripts/set_edge_cuts.py で読む）
    (out / "controller-board-outline.json").write_text(json.dumps(board_outline_segments()))
    print(f"lid volume = {lid_part.volume:.2f} mm^3, solids = {len(lid_part.solids())}")
    if "--show" in sys.argv:
        from ocp_vscode import show
        show(case, lid_part, *dummies(), names=["bottom-l", "lid", "board", "xiao", "usb-c", "battery"],
             colors=["#5b8fd6", "#7f77dd", "#3b6d11", "#1d9e75", "#ba7517", "#888780"])
