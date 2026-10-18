# 底蓋（左手）と、ボトムケースのコントローラー収納部。電池蓋方式: 底蓋を奥から入れて手前を持ち上げて閉じ、四隅を M2 ネジ
# （M2 × 5 の六角穴付きボルト）でボトムケースのボスに埋めた M2 ナットに留める。
# - 底蓋はボトムケースの傾いた底面と面一の平らな板。コントローラー基板と電池は底蓋に平行に載る
# - 座標はプレート DXF と同じ（平面）とボトムケースの z。底蓋まわりの形状は、底蓋の外面を基準にした
#   座標系 FRAME（x はそのまま、v は底蓋に沿って奥へ、w は底蓋に垂直に上へ）でも作る
# - USB-C の口は収納部の奥の壁より奥（壁のケーブル用の穴の中）まで入る。底蓋は奥から入れる: 手前を下げて傾けた底蓋の
#   USB-C を壁の穴に差し込み、奥の端を支点にして手前を持ち上げて閉じる（まっすぐ押し込むと USB-C が壁に当たる）。
#   底蓋の外面より上の点は、底蓋の外面の高さの軸のまわりに手前を下げる向きに回すと、どれも手前へ動くので、閉じた位置で
#   奥の壁に当たらなければ途中でも当たらない。USB-C は穴の中で上下に動くので、穴の上下のすき間に収まることを確かめる
#   （usb_swing_clearance）。手前のツメを軸に回して閉める方式は、ツメが抜けかけた状態で回り始めて USB-C が奥の壁に
#   当たるのでやめた
# - 収納部の上は天井でふさぐ。天井の下面は底蓋と平行（底蓋の外面から CEILING_W）で、天井の下の部品とのすき間はどこでも
#   同じ。床の上面は水平なので、天井は手前ほど薄い。ボトムケースは傾いた底面を下にして印刷するので、天井は印刷の向きでは
#   水平な張り出しになり、専用のサポート材で支える。天井には、J1 のケーブルの溝からマトリクス基板の J1 までの範囲と、
#   USB-C の上に開口を残す（floor_openings）
# - 実行: uv run python bay.py [--show]   （out/ に底蓋、ネジまわりのテストピース、ナットの溝だけのテストピース、
#   合わせ確認用のボトムケースの切り出し、
#   基板の外形 DXF と、KiCad の Edge.Cuts に入れる外形の JSON を書き出す。仮のコントローラー基板と XIAO のモックは
#   controller_test_board.py）
import json
import math
import sys
from pathlib import Path

from build123d import (
    Box, Circle, Compound, Face, GeomType, Line, Mesher, Plane, Polyline, Pos, Rectangle, RectangleRounded, SlotOverall, Solid,
    ThreePointArc, Wire,
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
LID_BACK = 6.6                # 底蓋の奥の端: 奥の外形から内側へ（面取りの斜面と、底面との境目の R0.5 から離す。
                              # 開口の奥の端から境目まで約 0.9。チルト角を大きくすると底面が低くなり、境目が内側へ寄る）
LID_FRONT_Y = -34.5           # 底蓋の手前の端（平面の y）

BOARD_W, BOARD_D, BOARD_T = 38.0, 29.0, 1.0   # コントローラー基板
BOARD_BACK = 7.4              # 基板の奥の端（XIAO の奥の端）: 奥の外形から内側へ。基板と XIAO の奥の縁と収納部の奥の壁の
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
# コネクタごとに 2 本の足をまとめて囲む長円の凹みを付ける。左手は J2、右手は J3 を使うが、基板は左右共通で両方に
# はんだ付けすることもあるので、J2 と J3 の両方の下に凹みを付ける（右手の底蓋は左手の鏡像なので、そのまま左右が入れ替わる）
BATTERY_PINS = [((-15.505, -11.495), (-13.505, -11.495)),   # J2 の足（基板の座標。KiCad の配置から）
                ((13.505, -11.495), (15.505, -11.495))]     # J3 の足
PIN_RECESS_R = 1.3            # 凹みの半径: 足の中心から（ランド Ø1.2 のはんだと、位置のずれ）
PIN_RECESS_DEPTH = 1.0        # 凹みの深さ: 底蓋の内面から（足の出る量は要実測）

XIAO_W, XIAO_D, STACK_H = 17.8, 21.0, 5.5     # XIAO（基板の下面から XIAO と USB-C の上端まで STACK_H）
XIAO_PCB_T = 1.2              # XIAO の基板の厚さ
XIAO_EDGE_PARTS = 1.0         # XIAO の奥の端（USB-C の両脇）の部品の高さ（ボタンや LED）。USB-C より低い
USB_W, USB_H, USB_OVERHANG = 8.94, 3.26, 1.0  # USB-C のレセプタクル（両端が半円の長円）。XIAO の端から張り出す
USB_CLEAR = 0.3               # USB-C の通り道のすき間（奥の壁の穴は、レセプタクルの長円をこれだけ広げた長円）
PLUG_W, PLUG_H = 13.0, 7.5    # ケーブル用のくぼみ（プラグの根元が入る大きさ）
PLUG_R = 2.5                  # くぼみの角の丸め（プラグの根元の角の丸みに合わせる。長円にすると角張ったプラグが入らない）
PLUG_GAP = 2.0                # くぼみの底: USB-C の口から奥へ。ケーブルを挿したとき、プラグの根元はレセプタクルに
                              # 触れずに少し離れるので、その分くぼみを浅くして収納部の奥の壁を残す（要実測）

# J1（基板の手前の端の近く、差し込み口は手前）に挿したプラグとケーブルの溝: 収納部の手前の壁から手前へ掘り込み、
# 上は床を抜いてボトムケースの内側の空間までつなげる（天井を作らない）。平面形は J1 の中心（基板の中心）
# に合わせた長方形で、手前の角を丸める。手前の面は底蓋に垂直。底は基板の下面の高さで、それより下は収納部の手前の壁を
# そのまま残す（底蓋の開口の形は変えない）
J1_GROOVE_W = 20.0            # 溝の幅（x）。J1（幅約 14）+ 両側 3
J1_GROOVE_DEPTH = 8.0         # 溝の奥行き: 収納部の手前の壁から手前へ（プラグのはみ出しとケーブルの曲げ。要実測）
J1_GROOVE_R = 1.5             # 溝の手前の角の丸め

# 底蓋を外すときに爪を掛ける切り欠き: 底蓋の手前の辺の中央に置く。収納部の手前の壁の下端（底蓋の手前の端の前）を、
# 底面から上へ 45° の斜面で削る。爪を斜面に沿って差し込み、底蓋の手前の端の内面に掛けて引き下ろす。斜面は、壁の面に
# 基板の下面の 0.1 下の高さで達する（印刷の向きでは下向きの 45° の斜面なので、サポートは要らない）。
# 中央は手前のリブと電池の枠（その間を埋めた所）の右の端にかかるので、底蓋の側にも同じ幅で、手前の端から NAIL_SLOT
# 奥まで、底蓋の内面の高さまで下がる溝を掘る（電池を押さえる枠の手前の壁は残す）
NAIL_W = 8.0                  # 切り欠きの幅（x）
NAIL_DEPTH = LID_T + STANDOFF - 0.1  # 底面での奥行き（収納部の手前の壁から手前へ）= 斜面が壁の面に達する高さ
NAIL_SLOT = 2.0               # 底蓋の側の溝の奥行き: 底蓋の手前の端から奥へ

BATTERY = (31.0, 21.0, 5.3)   # 電池（幅 x、奥行き v、厚さ）。EEMB LP502030（250 mAh）の仕様の最大値（公称 5.0）。要実測。
                              # 以前の 602030（300 mAh）は実測 31 × 21 × 6.0
BATTERY_GAP = 2.7             # 電池の右端と基板の左端の間（リード線を通す）
BATTERY_FRONT_Y = -30.5       # 電池の手前の端（平面の y）
BATTERY_FRAME = (1.0, 2.0, 0.4)  # 電池の枠（厚さ、高さ、すき間）
BATTERY_LEAD_GAP = 8.0        # 電池の枠の基板側の辺の切り欠き: 奥の角から手前への長さ（電池のリード線を通す）
BATTERY_PRY_GAP = 6.0         # 電池の枠の奥の辺の中央の切り欠きの幅（電池を外すときにスパッジャーを差し込む）

# 収納部の天井: 下面は底蓋と平行で、底蓋の外面から CEILING_W。天井の下に残る部品でいちばん高い電池の上面から
# CEILING_CLEAR 上。床の上面（水平）との間がいちばん薄いのは収納部の手前の端。天井の開口（floor_openings）は、
# コントローラーの基板の上の次の 2 つの長方形をつなげたもの（電池の上には開けない）:
# - J1 のケーブル、マトリクス基板の J1、J2: 左右は J2（高さ J2_H で天井より高い）の左端から J1 の溝の右端まで（マトリクスの
#   J1 の左右の端も含める）、奥はマトリクスの J1 と J2 の奥の端まで。どれも FLOOR_OPEN_MARGIN 外へ（角の丸めの中に
#   部品の角が入らない余裕）。手前は収納部の手前の壁まで。マトリクスの J1（R0C4 と R1C4 の境目。コントローラーの
#   J1 とほぼ同じ x）は下端が床の上面とほぼ同じ高さにあり、ケーブルは床の上面とソケットの下面の間（1.1 mm）を通れない
#   ので、溝から収納部に入って開口からマトリクスの J1 へ上がる。J2 に挿したプラグもこの範囲に入る
# - USB-C の上: USB-C は電池より高く、底蓋を奥から入れて回すときに少し上がるので、天井を抜く。手前は 1 つ目の長方形に
#   重ねてつなげる
CEILING_CLEAR = 0.5           # 天井の下に残る部品と天井のすき間
CEILING_W = LID_T + BATTERY[2] + CEILING_CLEAR   # 天井の下面: 底蓋の外面から
CEILING_MIN_T = 0.95          # 天井のいちばん薄い所（収納部の手前の端）の厚さの下限
FLOOR_OPEN_MARGIN = 1.0       # 天井の開口: マトリクスの J1 の本体、USB-C から外へ
FLOOR_OPEN_R = 2.0            # 天井の開口の角の丸め
J2_H = 4.85                   # J2（PH 横型）の高さ: 基板の上面から（KiCad の 3D モデル）
# 電池のケーブルのガイド: 底蓋から立ち上がる低い柱の上に、柱より広い円形の屋根を載せたもの（キノコ形）。ケーブルは屋根の
# 下に引っ掛け、柱に巻きつけて流す。屋根の下面は柱から縁へ 45° で広がる円すいにして、底蓋を外面を下にして刷るときに
# サポートなしで刷れるようにする（水平な下面だとサポートが外しにくかった）。ケーブルは電池の右上の角から横（基板の側）へ出て、電池の枠の右の外面に沿って奥から
# 手前へ流し、J2 に挿したプラグ（差し込み口は手前）へ手前から上がる。電池の枠の右の外面と J2 の足の間は基板の下
# （台座の高さ）を通る。柱は、基板の左の 2 つの角の切り欠き（左手ではボスがなく空いている）の中に置く:
# - 奥: 電池から横へ出たケーブルを巻きつけて、手前へ向ける
# - 手前: 下りてきたケーブルを、柱の左（西）から手前（南）を回して右（東）へ折り返し、J2 のプラグへ上げる。ケーブルを
#   通しやすいよう基板からなるべく離す。屋根の手前の縁は爪の溝の上に出るが、爪の先は屋根の縁の下面より低い所を通る
#   ので当たらない（底蓋に溝を掘った後でガイドを足すので、屋根は溝で削られない）
# 柱と屋根は、ケーブルがたるまず、きつすぎもしない太さにする。屋根は切り欠きの内側の縁と電池の枠の右の外面の間に収める
# （屋根の縁の下面は枠の上面と同じ高さなので、枠に接するところまでは寄せてよい）。2 本の x はそろえる
# 右手の底蓋は鏡像なので、基板の右の 2 つの切り欠きと J3 に合う
CABLE_POSTS = [(-17.3, -3.5), (-17.3, -25.4)]  # 柱の中心（基板の座標。奥、手前の順）
POST_D, POST_H = 3.0, 1.0     # 柱の直径、まっすぐな部分の高さ（底蓋の内面から。屋根の下の円すいはその上から始まる）
CAP_D, CAP_T = 5.0, 0.3       # 屋根の直径と、縁の厚さ（円すいの上の、まっすぐな部分）。屋根の上面は基板の上面（J2 の
                              # プラグの下面）より低い。屋根の下のすき間は、柱の根元で POST_H、縁で POST_H + 円すいの高さ
GUIDE_CLEAR = 0.5             # 屋根と、基板の切り欠きの縁・台座・爪の溝・電池の枠などのすき間
CABLE_PASS = 1.0              # 柱と電池の枠の右の外面の間（リード線 1 本が通る）

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
SCREW_LEN = 5.0               # ネジの長さ（M2 × 5）。先端は底蓋の外面から SCREW_HEAD[1] + SCREW_LEN。
                              # 手前のボスの高さ（床の上面まで）で決まる（チルトを 6.25° に下げてボスが低くなったので 6 から短くした）
SCREW_CLEAR_D = 2.4           # ネジの通し穴（底蓋とボス）
SCREW_BOSS_D = 6.4            # ボトムケースのボス
# ナットの溝: 収納部の内側（左のボスは右、右のボスは左）から横に差し込む。平らな面を手前と奥に向け、溝の幅は二面幅。
# 底蓋を外すと基板と電池も一緒に外れるので、収納部は空になって溝に手が届く。ボトムケースは底面を下にして刷るので、溝の
# 天井は水平な張り出しになり、中に入ったサポートが取れない。そこで天井を、溝の向き（x）に棟が通る NUT_ROOF_ANGLE の
# 切妻屋根にして、サポートなしで刷れるようにする（ナットは屋根の下の、高さ NUT_T の平らな部分に入る）
NUT_AF = 3.85                 # 溝の幅（M2 ナットの二面幅 4.0 mm より 0.15 狭い。印刷で溝が細る分と合わせて、押し込んで
                              # きつくはまり、重力で落ちない。4.2 では簡単に落ち、4.0 でちょうど良かったので少しだけ締めた。
                              # テストピースでは 3.95 でちょうど良かったが、ボトムケース全体ではかなり余裕があったので 3.85
                              # にした。高さ方向は層の厚さ単位でしか刷れないので、きつさは横方向（幅）で調整する）
NUT_T = 1.6                   # 溝の厚さ（M2 ナットの 1.6 mm と同じ。もとは 1.8）
NUT_TIP = 0.75                # ネジの先端がナットの上から出る長さ（溝を狭くして屋根が低くなった分、ナットを上げて、ネジの
                              # 先端と屋根のすき間を保つ。屋根の棟の高さは NUT_AF 3.95、NUT_TIP 0.8 のときと同じ）
NUT_ROOF_ANGLE = 45.0         # 溝の天井の屋根の傾き（水平から）[度]
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


def bevel_edge_gap():
    """収納部の開口の奥の端（平面）と、奥の面取りの斜面と底面の境目の、いちばん近い所の距離。境目は奥の外形から
    BEVEL_W - z × BEVEL_W / BEVEL_H 内側（z はそこでの底面の高さ）。"""
    out = []
    for i in range(41):
        x = LID_X[0] + (LID_X[1] - LID_X[0]) * i / 40
        inset = LID_BACK - LID_GAP
        out.append(inset - (B.BEVEL_W - tilt(back_y(x, inset)) * B.BEVEL_W / B.BEVEL_H))
    return min(out)


# 開口の奥の端が、面取りと底面の境目の R0.5 から離れていること
assert bevel_edge_gap() >= 0.8

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
    """電池の外形（FRAME の x, v の長方形）。手前の端は v_of(BATTERY_FRONT_Y)。grow だけ外へ広げる。
    電池と枠は底蓋に垂直に立つので、FRAME の面に描いて w の向きに押し出す。"""
    l, w, _ = BATTERY
    vc = v_of(BATTERY_FRONT_Y) + w / 2
    return Pos(BATTERY_X0 + l / 2, vc) * Rectangle(l + 2 * grow, w + 2 * grow)


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


def v_prism(section, x, w, v0, v1):
    """x-w 面の断面（中心が原点の Sketch）を、中心を FRAME の (x, w) に置いて v0 から v1 まで押し出した柱。"""
    plane = Plane(origin=(x, v0, w), x_dir=(1, 0, 0), z_dir=(0, 1, 0))
    return FRAME * extrude(plane * section, v1 - v0)


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

    # 電池の枠（底蓋に垂直に立つ。平面形は FRAME の x, v で描く）。左と手前は、リブとの間を埋めて底蓋の端までつなげる。
    # 左のボスに近づく所は、リブと同じく 45° の斜面で厚肉の高さまで下げ、角の台とつなげる。切り欠きは 2 つ:
    # - 基板側の辺の奥の角から BATTERY_LEAD_GAP（電池のリード線を奥の角から基板の側へ出す）
    # - 奥の辺の中央に幅 BATTERY_PRY_GAP（電池を外すときにスパッジャーを差し込む）
    t, h, c = BATTERY_FRAME
    outer = battery_outline(c + t).bounding_box()   # Y は FRAME の v
    inner = battery_outline(c).bounding_box()
    x0, v0 = LID_X[0] - 1, v_of(LID_FRONT_Y) - 1
    block = Pos((x0 + outer.max.X) / 2, (v0 + outer.max.Y) / 2) * Rectangle(outer.max.X - x0, outer.max.Y - v0)
    ring = block - battery_outline(c)
    va, vb = inner.max.Y - BATTERY_LEAD_GAP, outer.max.Y + 1
    lead_gap = Pos((inner.max.X + outer.max.X + 1) / 2, (va + vb) / 2) * Rectangle(outer.max.X + 1 - inner.max.X, vb - va)
    xc = battery_outline().center().X
    pry_gap = Pos(xc, (inner.max.Y - 1 + vb) / 2) * Rectangle(BATTERY_PRY_GAP, vb - inner.max.Y + 1)
    frame = (FRAME * Pos(0, 0, LID_T - 0.1) * extrude(ring - lead_gap - pry_gap, h + 0.1)) & lid_region()
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

    # 爪を掛ける溝（手前のリブと電池の枠の前を、底蓋の内面の高さまで削る）
    vf, xn = v_of(LID_FRONT_Y), nail_x()
    body -= local_box(xn - NAIL_W / 2, xn + NAIL_W / 2, vf - 1, vf + NAIL_SLOT, LID_T, LID_T + RIB_H + 1)

    # 電池のケーブルのガイド（手前の屋根は爪の溝の上に出るので、溝を掘った後で足す）
    body += cable_guides()
    return body


def battery_pins_local():
    """電池のコネクタの足の中心（FRAME の x, v）。J2 と J3 の全部の足。"""
    vt = board_v_top()
    return [(BOARD_CX + x, vt + y) for pair in BATTERY_PINS for x, y in pair]


def pin_recess_face():
    """電池のコネクタの足の凹みの平面形（FRAME の x, v）: コネクタごとに、足の並びに沿った長円。"""
    vt = board_v_top()
    face = None
    for (bx0, by0), (bx1, by1) in BATTERY_PINS:
        (x0, v0), (x1, v1) = (BOARD_CX + bx0, vt + by0), (BOARD_CX + bx1, vt + by1)
        assert abs(v0 - v1) < 1e-9
        f = Pos(x0, v0) * Circle(PIN_RECESS_R) + Pos(x1, v1) * Circle(PIN_RECESS_R)
        f += Pos((x0 + x1) / 2, v0) * Rectangle(abs(x1 - x0), 2 * PIN_RECESS_R)
        face = f if face is None else face + f
    return face


def cable_guides():
    """電池のケーブルのガイド（底蓋に足す立体）: 柱と屋根。FRAME の座標で、底蓋の内面から立ち上げる。"""
    vt = board_v_top()
    guides = None
    for bx, by in CABLE_POSTS:
        x, v = BOARD_CX + bx, vt + by
        post = local_cylinder(x, v, LID_T - 0.1, LID_T + POST_H, POST_D)
        w_cone = LID_T + POST_H
        cone = FRAME * Pos(x, v, w_cone) * Solid.make_cone(POST_D / 2, CAP_D / 2, cap_cone_h())   # 45° の円すい
        cap = local_cylinder(x, v, w_cone + cap_cone_h(), w_cone + cap_cone_h() + CAP_T, CAP_D)
        g = post + cone + cap
        guides = g if guides is None else guides + g
    return guides


# 凹みが基板の台座から離れていること、底蓋に厚さが残ること
assert min(math.hypot(px - x, pv - v) for px, pv in battery_pins_local() for x, v in board_holes_local()) \
    >= BOSS_D / 2 + PIN_RECESS_R + 0.5
assert LID_T - PIN_RECESS_DEPTH >= 0.6


def cap_cone_h():
    """ケーブルのガイドの屋根の下の円すいの高さ（45° で柱の直径から屋根の直径まで広がる）。"""
    return (CAP_D - POST_D) / 2


def check_cable_guides():
    """ケーブルのガイドの確かめ（基板の座標で）: 屋根の上面が基板の上面より低いこと。屋根が基板の左の切り欠きの中
    （基板がある範囲では縁から GUIDE_CLEAR）にあり、奥のリブ、爪の溝、台座、足の凹みから離れていること。柱と電池の枠の
    間にケーブルが通ること。"""
    t, h, c = BATTERY_FRAME
    vt = board_v_top()
    xn, yfn, ybn = board_notch()
    x_frame = battery_outline(c + t).bounding_box().max.X - BOARD_CX     # 電池の枠の右の外面
    x_notch = BOARD_CX - xn                                              # 左の切り欠きの内側の縁（負）
    x_board = -BOARD_W / 2                                               # 基板の左端
    y_back_notch, y_front_notch = v_of(ybn) - vt, v_of(yfn) - vt         # 奥の切り欠きの下端、手前の切り欠きの上端
    y_slot = v_of(LID_FRONT_Y) + NAIL_SLOT - vt                          # 爪の溝の奥の端
    y_rib = v_of(back_y(BOARD_CX + x_board, LID_BACK + RIB_T)) - vt      # 奥のリブの内面（基板の左端の x）
    assert LID_T + POST_H + cap_cone_h() + CAP_T < LID_T + STANDOFF + BOARD_T
    r = CAP_D / 2
    (xb, yb), (xf, yf) = CABLE_POSTS
    for x, y in CABLE_POSTS:
        assert x + r <= x_notch - GUIDE_CLEAR                    # 切り欠きの内側の縁
        assert x - POST_D / 2 - x_frame >= CABLE_PASS            # 電池の枠との間をケーブルが通る
        assert x - r >= x_frame - 1e-9                           # 屋根が電池の枠の上にかからない（接するのはよい）
        assert LID_T + POST_H + cap_cone_h() >= LID_T + h - 1e-9 # 屋根の縁の下面が枠の上面より低くない
        for hx, hy in [(hx - BOARD_CX, hv - vt) for hx, hv in board_holes_local()]:
            assert math.hypot(x - hx, y - hy) >= r + BOSS_D / 2 + GUIDE_CLEAR
        for pair in BATTERY_PINS:
            for px, py in pair:
                assert math.hypot(x - px, y - py) >= r + PIN_RECESS_R + GUIDE_CLEAR
    # 奥の柱: 奥の切り欠きの下端（その手前は基板の下）と、奥のリブの間
    assert yb - r >= y_back_notch + GUIDE_CLEAR and yb + r <= y_rib - GUIDE_CLEAR
    # 手前の柱: 屋根は手前の切り欠きの上端（その奥は基板の下）から離し、柱は爪の溝にかけない。屋根の手前の縁は溝の上に
    # 出るが、爪の先はボトムケースの斜面が壁の面に達する高さ（NAIL_DEPTH）より低い所を通るので、屋根の縁の下面が
    # それより高ければよい。屋根の縁は底蓋の手前の端からは離す
    assert yf + r <= y_front_notch - GUIDE_CLEAR and yf - POST_D / 2 >= y_slot
    assert LID_T + POST_H + cap_cone_h() > NAIL_DEPTH
    assert yf - r >= v_of(LID_FRONT_Y) - vt + GUIDE_CLEAR


check_cable_guides()


def case_cuts():
    """ボトムケースから削る形状。"""
    cuts = []
    # 収納部（底蓋の開口から、底蓋と平行な天井まで）と、天井の開口
    cuts.append(lid_region(LID_GAP) & slab(-50, CEILING_W))
    for face in floor_openings():
        cuts.append(lid_region(LID_GAP) & prism(face, -30, B.HEIGHT + 1))
    # 奥の壁を抜けてケーブルのプラグが通る穴（USB-C の口の先が入る。収納部の中から、くぼみまで）と、面取りの斜面のくぼみ
    # （底は口から PLUG_GAP 奥）。くぼみは USB-C の口に合わせて決まるので、基板を奥へ寄せた分だけ浅くなり、収納部とくぼみの
    # 間の壁が厚くなる
    vt, wc = board_v_top(), usb_axis_w()
    mouth = vt + USB_OVERHANG
    hw, hh = usb_hole_half()
    cuts.append(v_prism(SlotOverall(2 * hw, 2 * hh), BOARD_CX, wc, vt - 1, mouth + PLUG_GAP + 0.5))
    cuts.append(v_prism(RectangleRounded(PLUG_W, PLUG_H, PLUG_R), BOARD_CX, wc, mouth + PLUG_GAP, mouth + 20))
    cuts.append(j1_groove())
    cuts.append(nail_notch())
    return cuts


def floor_openings():
    """収納部の天井の開口の平面形（収納部の範囲で切る前の、角を丸めた長方形）のリスト。2 つは重なっていて、つながった
    1 つの開口になる: J1 のケーブル・マトリクス基板の J1・J2 の開口と、USB-C の上の開口。マトリクスの J1 の位置は
    matrix_mock（配置スクリプトの設定）から読む（matrix_mock は bay を使うので、ここで読み込む）。"""
    import matrix_mock as M
    vt = board_v_top()

    def rounded(x0, x1, y0, y1):
        rect = Pos((x0 + x1) / 2, (y0 + y1) / 2) * Rectangle(x1 - x0, y1 - y0)
        return fillet(rect.vertices(), FLOOR_OPEN_R).faces()[0]

    j2_left = BOARD_CX + min(x for x, _ in BATTERY_PINS[0]) - 2.5                      # J2 の左端（基板の x -18）
    j2_back = FRAME.from_local_coords((0, vt + BATTERY_PINS[0][0][1] + 1.9, 0)).Y      # J2 の奥の端（基板の y -9.6）
    j1_x0 = M.J1_AT[0] + M.J1_BODY[0] - FLOOR_OPEN_MARGIN
    j1_x1 = M.J1_AT[0] + M.J1_BODY[2] + FLOOR_OPEN_MARGIN
    y_back = max(M.J1_AT[1] + M.J1_BODY[3] + FLOOR_OPEN_MARGIN, j2_back + FLOOR_OPEN_MARGIN)
    cable = rounded(min(j2_left - FLOOR_OPEN_MARGIN, j1_x0), max(BOARD_CX + J1_GROOVE_W / 2, j1_x1),
                    LID_FRONT_Y - 5,   # 手前は収納部の手前の壁より先まで（収納部の範囲で切る）
                    y_back)
    hw = USB_W / 2 + FLOOR_OPEN_MARGIN
    usb = rounded(BOARD_CX - hw, BOARD_CX + hw,
                  y_back - 2 * FLOOR_OPEN_R,      # 手前は 1 つ目の長方形に重ねてつなげる
                  back_y(BOARD_CX, 0) + 5)        # 奥は奥の壁より先まで
    return [cable, usb]


def ceiling_clearance():
    """天井の下に残る部品（電池、XIAO、USB-C、J1、J2）の上面と天井の下面のすき間の最小（底蓋に垂直な向き）。上面を
    細かくたどり、天井の開口（角を丸めた長方形）の中にない点がある部品だけを数える。"""
    rects = [f.bounding_box() for f in floor_openings()]

    def in_rounded(p, bb, r):
        if not (bb.min.X <= p.X <= bb.max.X and bb.min.Y <= p.Y <= bb.max.Y):
            return False
        dx = max(bb.min.X + r - p.X, 0.0, p.X - (bb.max.X - r))
        dy = max(bb.min.Y + r - p.Y, 0.0, p.Y - (bb.max.Y - r))
        return math.hypot(dx, dy) <= r

    def in_opening(p):
        return any(in_rounded(p, bb, FLOOR_OPEN_R) for bb in rects)

    vt, wt = board_v_top(), LID_T + STANDOFF + BOARD_T
    bo = battery_outline().bounding_box()   # Y は FRAME の v
    jy = vt - 24.375   # J1 の原点（KiCad の配置）
    tops = [  # (x0, x1, v0, v1, 上面の w)
        (bo.min.X, bo.max.X, bo.min.Y, bo.max.Y, LID_T + BATTERY[2]),
        (BOARD_CX - XIAO_W / 2, BOARD_CX + XIAO_W / 2, vt - XIAO_D, vt, wt + XIAO_PCB_T + XIAO_EDGE_PARTS),
        (BOARD_CX - USB_W / 2, BOARD_CX + USB_W / 2, vt - 7, vt + USB_OVERHANG, usb_axis_w() + USB_H / 2),
        (BOARD_CX - 7.4, BOARD_CX + 7.4, jy - 3.3, jy + 3.3, wt + 2.95),      # J1（コートヤード）
        (BOARD_CX - 18, BOARD_CX - 11, vt - 18.3, vt - 9.6, wt + J2_H),     # J2（コートヤード）
    ]
    gap = 99
    for x0, x1, v0, v1, w in tops:
        pts = [FRAME.from_local_coords((x0 + (x1 - x0) * i / 10, v0 + (v1 - v0) * j / 10, 0))
               for i in range(11) for j in range(11)]
        if not all(in_opening(p) for p in pts):
            gap = min(gap, CEILING_W - w)
    return gap


def ceiling_min_thickness():
    """天井のいちばん薄い所（収納部の手前の端）の厚さ: 床の上面（水平）と天井の下面の z の差。"""
    return B.FLOOR_Z - FRAME.from_local_coords((0, v_of(LID_FRONT_Y) - LID_GAP, CEILING_W)).Z


# 天井の下に残る部品が天井に当たらないこと（マトリクスの J1 は、開口の範囲を J1 から決めているので開口の中にある）。
# 天井のいちばん薄い所に厚さが残ること
assert ceiling_clearance() >= CEILING_CLEAR - 1e-9
assert ceiling_min_thickness() >= CEILING_MIN_T
# 天井の開口が電池の枠の上にかからないこと
assert floor_openings()[0].bounding_box().min.X >= battery_outline(BATTERY_FRAME[2] + BATTERY_FRAME[0]).bounding_box().max.X


def nail_x():
    """爪を掛ける切り欠きの中心の x: 底蓋の手前の辺の中央。"""
    return (LID_X[0] + LID_X[1]) / 2


def nail_notch():
    """爪を掛ける切り欠き: 底面から上へ 45° の斜面で収納部の手前の壁を削る三角柱（FRAME の座標、x 方向に押し出す）。
    斜面は底面で壁から NAIL_DEPTH 手前、壁の面で w = NAIL_DEPTH。奥は収納部の中まで伸ばす。"""
    vf = v_of(LID_FRONT_Y) - LID_GAP
    x0, d = nail_x() - NAIL_W / 2, NAIL_DEPTH
    pts = [(x0, vf - d - 1, -1), (x0, vf + 1, -1), (x0, vf + 1, d + 1)]
    face = Face(Wire(Polyline(*pts, close=True).edges()))
    return FRAME * extrude(face, NAIL_W, dir=(1, 0, 0))


# 爪の切り欠きが J1 の溝から 1 mm 以上離れていること。底蓋の側の溝が、電池の枠の手前の壁を削らないこと
assert nail_x() + NAIL_W / 2 <= BOARD_CX - J1_GROOVE_W / 2 - 1.0
assert v_of(LID_FRONT_Y) + NAIL_SLOT <= battery_outline(BATTERY_FRAME[2] + BATTERY_FRAME[0]).bounding_box().min.Y


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
    pocket = Pos(x, v, nut_top - NUT_T) * extrude(face, NUT_T, dir=(0, 0, 1))
    # 天井の切妻屋根: 断面（v-w 面）が二面幅を底辺とする三角形の柱を x の向きに伸ばし、溝の平面形で切る
    h = nut_roof_h()
    tri = Face(Wire(Polyline((-20, -NUT_AF / 2, 0), (-20, NUT_AF / 2, 0), (-20, 0, h), close=True).edges()))
    roof = Pos(x, v, nut_top) * extrude(tri, 40, dir=(1, 0, 0))
    roof &= Pos(x, v, nut_top) * extrude(face, h, dir=(0, 0, 1))
    return FRAME * (pocket + roof)


def nut_roof_h():
    """ナットの溝の天井の屋根の高さ（ナットの上面から棟まで）。"""
    return NUT_AF / 2 * math.tan(math.radians(NUT_ROOF_ANGLE))


def add_screw_bosses(case):
    """収納部を削った後のボトムケースに、四隅のボスを足して、ネジの通し穴とナットの溝を開ける。
    ボス、穴、溝はどれも底蓋に垂直（FRAME の w の向き）。通し穴はナットの溝の床まで（その上はナットと、溝の天井の
    屋根の空間にネジの先端が入る。穴を屋根まで開けると、屋根の斜面に水平な天井の小さな張り出しができる）。"""
    for x, v, sx, sv in lid_screws_local():
        case += boss_solid(x, v, sx, sv)
    tip = SCREW_HEAD[1] + SCREW_LEN
    w0 = LID_T + LID_PAD[1]
    for x, v, sx, _ in lid_screws_local():
        case -= local_cylinder(x, v, w0 - 1, tip - NUT_TIP - NUT_T + 0.1, SCREW_CLEAR_D)
        case -= nut_pocket(x, v, sx)
    return case


def boss_top_w():
    """ボスの上端（床の上面）の、ネジの軸での高さ（底蓋の外面から垂直に）のうち、いちばん低いもの（手前のボス）。"""
    out = []
    for x, v, _, _ in lid_screws_local():
        base = FRAME.from_local_coords((x, v, 0))
        out.append((B.FLOOR_Z - base.Z) / math.cos(A))
    return min(out)


# ナットの溝の屋根の棟が、ボスの上端（床の上面）より下に収まること。ネジ（M2、太さ SCREW_D）の先端が、ネジの外周の
# 位置での屋根の高さより SCREW_TIP_CLEAR 以上低いこと（ナットの上に出たネジの先端は屋根の下の空間に入る）
SCREW_D = 2.0                 # ネジの太さ（M2）
SCREW_TIP_CLEAR = 0.15        # ネジの先端と屋根のすき間（ネジの外周の位置で。先端の面取りの分、実際はもっと空く）
assert SCREW_HEAD[1] + SCREW_LEN - NUT_TIP + nut_roof_h() <= boss_top_w() - 0.5
assert NUT_TIP + SCREW_TIP_CLEAR <= (NUT_AF / 2 - SCREW_D / 2) * math.tan(math.radians(NUT_ROOF_ANGLE)) + 1e-9


def usb_hole_half():
    """奥の壁の USB-C の穴（長円）の、幅と高さの半分（FRAME の x, w）。"""
    return USB_W / 2 + USB_CLEAR, USB_H / 2 + USB_CLEAR


def stadium_gap(dx, dw, half_w, r):
    """長円（中心が原点、幅 2 half_w、高さ 2 r、両端が半径 r の半円）の中の点 (dx, dw) と縁の距離。中なら正。"""
    return r - math.hypot(max(abs(dx) - (half_w - r), 0.0), dw)


def usb_outline(n=24):
    """USB-C のレセプタクルの断面の縁の点（中心が原点の (dx, dw)）: 両端の半円と、上下の直線。"""
    a, r = USB_W / 2 - USB_H / 2, USB_H / 2
    pts = []
    for i in range(n + 1):   # 右の半円と左の半円
        t = math.pi * (i / n - 0.5)
        pts += [(a + r * math.cos(t), r * math.sin(t)), (-a - r * math.cos(t), r * math.sin(t))]
    pts += [(a * (2 * k / 10 - 1), sw * r) for k in range(11) for sw in (-1, 1)]
    return pts


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
    間の、USB-C と壁の穴（どちらも長円）の縁の最小のすき間と、抜けたときの角度（度）。USB-C は支点より奥にあるので、
    開けると手前へ動きながら上へ動く。"""
    vt, wc = board_v_top(), usb_axis_w()
    hw, hh = usb_hole_half()
    pivot_v = v_of(back_y(BOARD_CX, LID_BACK))
    # USB-C の断面の縁を、口から手前へ細かくたどる
    pts = [(BOARD_CX + dx, vt + USB_OVERHANG - k * 0.1, wc + dw) for dx, dw in usb_outline() for k in range(31)]
    gap = USB_CLEAR
    for i in range(steps + 1):
        angle = math.radians(20) * i / steps
        inside = [q for q in (swing(p, angle, pivot_v) for p in pts) if wall_gap(q) < 0]
        if not inside:
            return gap, math.degrees(angle)
        gap = min([gap] + [stadium_gap(q[0] - BOARD_CX, q[2] - wc, hw, hh) for q in inside])
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
    usb = v_prism(SlotOverall(USB_W, USB_H), BOARD_CX, wc, vt - 7, vt + USB_OVERHANG)
    battery = FRAME * Pos(0, 0, LID_T) * extrude(battery_outline(), BATTERY[2])   # 底蓋に垂直に立つ
    return board, xiao, usb, battery


# 基板と XIAO が収納部の奥の壁に当たらないこと。底蓋を開け閉めする途中で、USB-C が壁の穴の縁に当たらないこと
assert controller_wall_clearance() >= CONTROLLER_CLEAR - 1e-6
assert usb_swing_clearance()[0] >= 0.1


def screw_test(case, lid_part):
    """ネジまわりのテストピース: 右手前の角（いちばん低いボス、基板の切り欠き）のまわりだけを切り出したボトムケース
    （床の上面まで）と底蓋。"""
    x, v, _, _ = [s for s in lid_screws_local() if s[2] > 0 and s[3] < 0][0]
    cx, cy = boss_center(x, v)
    region = Pos(cx - 4, cy + 2) * Rectangle(20, 18)
    return case & prism(region, -30, B.FLOOR_Z), lid_part & prism(region)


NUT_TEST_MARGIN = 1.2         # ナットの溝のテストピース: 溝とネジ穴のまわりに残す肉


def nut_test(case):
    """ナットの溝のはまり具合だけを試すテストピース: 右手前のボスの、ナットの溝（屋根を含む）とネジ穴のまわりだけを
    底蓋の座標の箱とボスの平面形で切り出したもの（ボスの外の収納部の天井は張り出しになるので含めない。溝の口はボスの
    丸い面に開く）。底面はボスの下面（底蓋の厚肉の上面に当たる面、底蓋と平行）。ボトムケースと同じく屋根が上になる
    向きで刷れるよう、底面を z = 0 に寝かせた形も返す: (ケースの中の位置の形, 印刷の向きの形)。"""
    x, v, sx, sv = [s for s in lid_screws_local() if s[2] > 0 and s[3] < 0][0]
    m = NUT_TEST_MARGIN
    reach = SCREW_BOSS_D / 2 + 2.0                 # 溝の口（nut_pocket と同じ）
    r = NUT_AF / math.sqrt(3)                      # 六角形の頂点（壁の側）
    x0, x1 = (x - reach, x + r + m) if sx > 0 else (x - r - m, x + reach)
    v0, v1 = v - NUT_AF / 2 - m, v + NUT_AF / 2 + m
    w0 = LID_T + LID_PAD[1]                        # ボスの下面
    apex = SCREW_HEAD[1] + SCREW_LEN - NUT_TIP + nut_roof_h()     # 屋根の棟
    w1 = min(apex + m, boss_top_w())               # 棟の上に m（ボスの上端まで）
    assert w1 - apex >= 0.5                        # ケースと同じ（屋根の棟の上に 0.5 以上）
    footprint = FRAME * Pos(0, 0, w0 - 1) * extrude(corner_footprint(x, v, sx, sv, SCREW_BOSS_D / 2), w1 - w0 + 2)
    piece = case & local_box(x0, x1, v0, v1, w0, w1) & footprint
    flat = Pos(-x, -v, -w0) * FRAME.to_local_coords(piece)
    return piece, flat


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
    1 / cos(チルト角) 伸びて厳密には楕円になるが、差は 0.01 mm 未満）。"""
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
    """組み付け確認用の XIAO と USB-C のモック: XIAO の基板と奥の端の小さな部品の高さまでの箱に USB-C（長円）を載せたもの
    （USB-C は XIAO の奥の端から USB_OVERHANG 張り出す）。基板の面の座標で、下面を z = 0 に置く（印刷の向き）。
    仮のコントローラー基板は controller_test_board.py（KiCad の穴つき）で作る。"""
    h = XIAO_PCB_T + XIAO_EDGE_PARTS
    xiao = Pos(0, -XIAO_D / 2, h / 2) * Box(XIAO_W, XIAO_D, h)
    usb_len = 7.0 + USB_OVERHANG
    usb_plane = Plane(origin=(0, USB_OVERHANG - usb_len, XIAO_PCB_T + USB_H / 2), x_dir=(1, 0, 0), z_dir=(0, 1, 0))
    xiao += extrude(usb_plane * SlotOverall(USB_W, USB_H), usb_len)   # USB-C は長円の断面
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
    export([("nut-test", nut_test(case)[1])], out / "nut-test-l")
    export_board_dxf(out / "controller-board-l.dxf")
    # KiCad の Edge.Cuts に入れる外形（scripts/set_edge_cuts.py で読む）
    (out / "controller-board-outline.json").write_text(json.dumps(board_outline_segments()))
    print(f"lid volume = {lid_part.volume:.2f} mm^3, solids = {len(lid_part.solids())}")
    if "--show" in sys.argv:
        from ocp_vscode import show
        show(case, lid_part, *dummies(), names=["bottom-l", "lid", "board", "xiao", "usb-c", "battery"],
             colors=["#5b8fd6", "#7f77dd", "#3b6d11", "#1d9e75", "#ba7517", "#888780"])
