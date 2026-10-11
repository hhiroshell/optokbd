# 底蓋（左手）と、ボトムケースのコントローラー収納部。電池蓋方式: 手前の辺に 2 本の固定ツメ、奥の辺にしなるラッチ。
# - 底蓋はボトムケースの傾いた底面と面一の平らな板。コントローラー基板と電池は底蓋に平行に載る
# - 座標はプレート DXF と同じ（平面）とボトムケースの z。底蓋まわりの形状は、底蓋の外面を基準にした
#   座標系 FRAME（x はそのまま、v は底蓋に沿って奥へ、w は底蓋に垂直に上へ）でも作る
# - コントローラー基板の真上は床を抜く（XIAO の上の高さのため）。電池の上は床を残す
# - 実行: uv run python bay.py [--show]   （out/ に底蓋、ラッチのテストピース、合わせ確認用のボトムケースの切り出し、
#   基板の外形 DXF を書き出す）
import math
import sys
from pathlib import Path

from build123d import (
    Compound, Face, Kind, Mesher, Plane, Polyline, Pos, Rectangle, Solid, Wire,
    export_step, export_stl, extrude, offset,
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
CEILING_Z = B.FLOOR_Z - 1.2   # 収納部の天井（床の上の板 1.2 mm を残す）。基板の真上は床を抜く
LID_T = 1.6                   # 底蓋の厚さ
LID_GAP = 0.2                 # 底蓋とボトムケースのすき間
LID_BACK = 5.8                # 底蓋の奥の端: 奥の外形から内側へ（面取りの斜面と、底面との境目の R0.5 から離す）
LID_FRONT_Y = -34.5           # 底蓋の手前の端（平面の y）

BOARD_W, BOARD_D, BOARD_T = 38.0, 29.0, 1.0   # コントローラー基板
BOARD_BACK = 7.2              # 基板の奥の端: 奥の外形から内側へ（XIAO は底蓋に垂直に立つので上ほど奥へ傾く分の余裕）
STANDOFF = 1.5                # 台座の高さ（スルーホール部品の足の逃げ）
BOARD_HOLES = [(5.0, 3.0), (33.0, 3.0), (4.5, 25.5), (33.5, 25.5)]  # 基板の左上（奥の端の中央の高さ）からの位置
BOSS_D, PILOT_D, PILOT_DEPTH = 5.0, 1.6, 2.4  # 台座、M2 タッピングネジの下穴
FLOOR_OPEN_MARGIN = 1.0       # 床を抜く範囲: 基板の外形から外へ

XIAO_W, XIAO_D, STACK_H = 17.8, 21.0, 5.5     # XIAO（基板の下面から XIAO と USB-C の上端まで STACK_H）
USB_W, USB_H, USB_OVERHANG = 8.94, 3.26, 1.0  # USB-C のレセプタクル。XIAO の端から張り出す
USB_CLEAR = 0.3               # USB-C の通り道のすき間
PLUG_W, PLUG_H = 13.0, 7.5    # ケーブル用のくぼみ（プラグの根元が入る大きさ）

BATTERY = (33.0, 20.0, 6.0)   # 電池（長さ x、幅 v、厚さ）。EEMB 602030、保護回路込みの長さを仮定
BATTERY_GAP = 2.7             # 電池の右端と基板の左端の間
BATTERY_BACK = 9.4            # 電池の奥の端: 奥の外形から内側へ（ラッチの柱より手前）
BATTERY_FRAME = (1.0, 2.0, 0.4)  # 電池の枠（厚さ、高さ、すき間）

# 固定ツメ: 断面はくさび形。下面（底蓋の外側）は底蓋に平行、上面は根元から先端へ下がる斜面。
# ラッチで底蓋の奥が押し上げられると、斜面が受けの溝の斜面に押し付けられて位置が決まる
TONGUE_W = 8.0                # 幅
TONGUE_LEN = 3.0              # 手前の辺からの突き出し
TONGUE_BOTTOM = 1.0           # 下面の高さ（底蓋の外面から）。この下に受けの溝の縁が残る
TONGUE_ROOT_T = 1.9           # 根元の厚さ（上面は底蓋の内面より上に出る。コントローラー基板の下面 3.1 より低く）
TONGUE_TIP_T = 1.0            # 先端の厚さ
TONGUE_BASE = 1.0             # 根元の内側の支え（手前の辺から内側への長さ。高さは根元の上面まで）
TONGUE_SLOT_CLEAR = 0.2       # 受けの溝とのすき間（下面、斜面、先端）

# 電池（左）と基板（右）をひとまとまりとして、その左右の中央をケースの中央に置く。
# 底蓋も同じ中央で、電池の枠と基板から LID_MARGIN 離れる幅にする
LID_CX = (P.LEFT[1][0] + P.RIGHT[1][0]) / 2
LID_MARGIN = 2.0
_battery_side = BATTERY_GAP + BATTERY[0] + BATTERY_FRAME[0] + BATTERY_FRAME[2]  # 基板の左端から電池の枠の外側まで
BOARD_CX = LID_CX + _battery_side / 2      # 基板の中心の x（= USB-C の位置）
BATTERY_X0 = BOARD_CX - BOARD_W / 2 - BATTERY_GAP - BATTERY[0]  # 電池の左端
_half = BOARD_W / 2 + _battery_side / 2 + LID_MARGIN
LID_X = (LID_CX - _half, LID_CX + _half)  # 底蓋の左右の端
# USB-C（XIAO）は奥の中央のネジ（x = LID_CX）を避ける必要がある: XIAO の左端とネジの穴の間に 1 mm 以上
assert BOARD_CX - XIAO_W / 2 - (LID_CX + L.BOSS_R) >= 1.0

TONGUES_X = (LID_CX - 20.0, LID_CX + 20.0)  # 固定ツメの中心の x（底蓋の中央に対して対称）

LATCH_X = (46.0, 60.0)        # ラッチの腕（付け根の x、先端の x）。先端はなるべく底蓋の中央に寄せる
                              # （すぐ右に奥の中央のネジの穴と基板があるので、その手前まで）
LATCH_ARM = 1.6               # 腕の幅（しなる方向の厚さ）
LATCH_SLOT = 1.0              # 腕の内側のすき間（押し込める量）
LATCH_POST = (4.5, 7.0)       # 腕の先端の柱（x の長さ、底蓋の外面からの高さ）
LATCH_NOSE = (0.7, 5.5, 6.3)  # フック（突き出し、下面の高さ、ななめ面の始まりの高さ）。上端は柱の高さ
LATCH_GROOVE = (0.4, 0.2)     # 受けの溝のすき間（奥行き方向、上下）
FINGER_NOTCH = (1.5, 3.0, 2.5)  # 指をかける切り欠き（腕の先端の左右に足す幅、奥行き、高さ）

# ラッチの指の切り欠きが奥の中央のネジの穴に、腕のすき間が基板に当たらないこと
assert LATCH_X[1] + FINGER_NOTCH[0] + 1.0 <= LID_CX - L.BOSS_R
assert LATCH_X[1] + LATCH_SLOT + 1.0 <= BOARD_CX - BOARD_W / 2

FAR = 60.0


# ---- 平面形 ----

def board_v_top():
    """基板の奥の辺のうち、XIAO が載る直線部分の v。XIAO の両端のうち、奥の外形の円弧から遠い側で BOARD_BACK になる位置。"""
    x_far = BOARD_CX + XIAO_W / 2 if BOARD_CX >= BACK_CX else BOARD_CX - XIAO_W / 2
    return v_of(back_y(x_far, BOARD_BACK))


def board_path():
    """基板の外形（平面）: [(点, 次の点までの辺)]。奥の辺は、XIAO の幅だけ直線（XIAO の端と USB-C を合わせる）、
    その両側の肩は奥の外形に沿った円弧。手前の辺は直線。"""
    x0, x1 = BOARD_CX - BOARD_W / 2, BOARD_CX + BOARD_W / 2
    xl, xr = BOARD_CX - XIAO_W / 2, BOARD_CX + XIAO_W / 2
    y_top = board_v_top() * math.cos(A)
    y_front = y_top - BOARD_D * math.cos(A)
    arc = back_arc(BOARD_BACK)
    path = [
        ((x0, y_front), None),
        ((x1, y_front), None),
        ((x1, back_y(x1, BOARD_BACK)), arc),
        ((xr, back_y(xr, BOARD_BACK)), None),
        ((xr, y_top), None),
        ((xl, y_top), None),
        ((xl, back_y(xl, BOARD_BACK)), arc),
        ((x0, back_y(x0, BOARD_BACK)), None),
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


def lid_outline(grow=0.0):
    """底蓋の外形（平面）。奥の辺は奥の外形に沿った円弧。grow だけ外へ広げる（ボトムケースの開口は LID_GAP）。"""
    x0, x1 = LID_X[0] - grow, LID_X[1] + grow
    yf = LID_FRONT_Y - grow
    inset = LID_BACK - grow
    return path_face([
        ((x0, yf), None),
        ((x1, yf), None),
        ((x1, back_y(x1, inset)), back_arc(inset)),
        ((x0, back_y(x0, inset)), None),
    ])


def battery_outline(grow=0.0):
    l, w, _ = BATTERY
    y1 = back_y(BATTERY_X0 + l / 2, BATTERY_BACK)
    return Pos(BATTERY_X0 + l / 2, y1 - w / 2 * math.cos(A)) * Rectangle(l + 2 * grow, w * math.cos(A) + 2 * grow)


def latch_band(r_in, r_out, x0, x1):
    """奥の外形と同心の帯（内側から r_in、r_out だけ内側の円弧の間）を x0..x1 で切ったもの。"""
    return path_face([
        ((x0, back_y(x0, r_out)), back_arc(r_out)),
        ((x1, back_y(x1, r_out)), None),
        ((x1, back_y(x1, r_in)), back_arc(r_in)),
        ((x0, back_y(x0, r_in)), None),
    ])


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


def board_holes_local():
    x0, vt = BOARD_CX - BOARD_W / 2, board_v_top()
    return [(x0 + dx, vt - dv) for dx, dv in BOARD_HOLES]


def usb_axis_w():
    return LID_T + STANDOFF + BOARD_T + 1.2 + USB_H / 2


def latch_nose():
    """フック: 柱の上部で奥へ突き出す。上側をななめにして、閉めるときにボトムケースの縁で押し込まれるようにする。
    底蓋の奥の端は鉛直な面なので、断面は YZ 平面で描き、高さは底蓋の外面から垂直に測る。"""
    x0, x1 = LATCH_X[1] - LATCH_POST[0], LATCH_X[1]
    y_e = back_y((x0 + x1) / 2, LID_BACK)
    depth, w_low, w_ramp = LATCH_NOSE
    c = math.cos(A)

    def at(dy, w):
        return (y_e + dy, tilt(y_e + dy) + w / c)

    pts = [at(-0.3, w_low), at(depth, w_low), at(depth, w_ramp), at(-0.3, LATCH_POST[1])]
    return extrude(Plane.YZ.offset(x0) * Face(Wire(Polyline(*pts, close=True).edges())), x1 - x0, dir=(1, 0, 0))


def tongue_wedge(x, clear):
    """固定ツメのくさび（clear > 0 なら、そのすき間だけ大きくした受けの溝）。
    底蓋の手前の辺は鉛直な面なので、断面は YZ 平面で描き、高さは底蓋の外面から垂直に測る。"""
    y_f = LID_FRONT_Y
    c = math.cos(A)

    def at(y, w):
        return (y, tilt(y) + w / c)

    w0 = TONGUE_BOTTOM - clear
    root, tip = TONGUE_BOTTOM + TONGUE_ROOT_T + clear, TONGUE_BOTTOM + TONGUE_TIP_T + clear
    y_tip = y_f - TONGUE_LEN - clear
    # 溝は収納部の開口（手前の辺 + LID_GAP）より内側まで伸ばして、収納部とつなげる
    y_in = y_f + (TONGUE_BASE if clear == 0 else LID_GAP + 0.5)
    pts = [at(y_in, w0), at(y_tip, w0), at(y_tip, tip), at(y_f, root), at(y_in, root)]
    width = TONGUE_W + 2 * clear
    return extrude(Plane.YZ.offset(x - width / 2) * Face(Wire(Polyline(*pts, close=True).edges())), width, dir=(1, 0, 0))


def lid():
    """底蓋。"""
    body = prism(lid_outline()) & slab(0, LID_T)

    # 固定ツメ（手前の辺）
    for x in TONGUES_X:
        body += tongue_wedge(x, 0.0)

    # ラッチ: 奥の辺に沿った腕。内側にすき間を切って、付け根（LATCH_X[0]）だけでつなぐ
    a0, a1 = LATCH_X
    slot = latch_band(LID_BACK + LATCH_ARM, LID_BACK + LATCH_ARM + LATCH_SLOT, a0, a1 + LATCH_SLOT)
    slot += latch_band(LID_BACK - 1.0, LID_BACK + LATCH_ARM + LATCH_SLOT, a1, a1 + LATCH_SLOT)
    body -= prism(slot) & slab(-1, LID_T + 1)
    post = latch_band(LID_BACK, LID_BACK + LATCH_ARM, a1 - LATCH_POST[0], a1)
    body += prism(post) & slab(0, LATCH_POST[1])
    body += latch_nose()

    # 台座（M2 タッピングネジの下穴つき）
    for x, v in board_holes_local():
        body += local_cylinder(x, v, LID_T - 0.1, LID_T + STANDOFF, BOSS_D)
        body -= local_cylinder(x, v, LID_T + STANDOFF - PILOT_DEPTH, LID_T + STANDOFF + 0.1, PILOT_D)

    # 電池の枠（基板側は J2 へのリード線のために開ける）
    t, h, c = BATTERY_FRAME
    ring = battery_outline(c + t) - battery_outline(c)
    yc = battery_outline().center().Y
    gap = Pos(BATTERY_X0 + BATTERY[0] + c + t / 2, yc) * Rectangle(t + 2, 8)
    body += prism(ring - gap) & slab(LID_T - 0.1, LID_T + h)
    return body


def case_cuts():
    """ボトムケースから削る形状。"""
    cuts = []
    # 収納部（底蓋の開口から天井まで）
    cuts.append(prism(lid_outline(LID_GAP), -30, CEILING_Z))
    # 基板の真上の床を抜く
    cuts.append(prism(offset(board_outline(), FLOOR_OPEN_MARGIN, kind=Kind.ARC), CEILING_Z - 1, B.HEIGHT))
    # USB-C の通り道と、面取りの斜面のケーブル用のくぼみ
    vt, wc = board_v_top(), usb_axis_w()
    mouth = vt + USB_OVERHANG
    # 通り道は下まで開ける（底蓋は手前のツメを軸に回して開けるので、USB-C は少し奥へ動きながら下がる）。
    # 側面は収納部の壁と同じく鉛直、上面は底蓋に平行
    w_top = wc + USB_H / 2 + USB_CLEAR
    y_tip = FRAME.from_local_coords((BOARD_CX, mouth, w_top)).Y + 0.5
    y_in = back_y(BOARD_CX, LID_BACK) - 3
    slot = Pos(BOARD_CX, (y_in + y_tip) / 2) * Rectangle(USB_W + 2 * USB_CLEAR, y_tip - y_in)
    cuts.append(prism(slot) & slab(-30, w_top))
    cuts.append(local_box(BOARD_CX - PLUG_W / 2, BOARD_CX + PLUG_W / 2, mouth + 0.2, mouth + 20,
                          wc - PLUG_H / 2, wc + PLUG_H / 2))
    # 固定ツメの受け（ツメと同じくさび形の溝）
    for x in TONGUES_X:
        cuts.append(tongue_wedge(x, TONGUE_SLOT_CLEAR))
    # ラッチの受けの溝と、指をかける切り欠き
    a1 = LATCH_X[1]
    px0, px1 = a1 - LATCH_POST[0], a1
    depth, w_low, _ = LATCH_NOSE
    gv, gw = LATCH_GROOVE
    groove = latch_band(LID_BACK - depth - gv, LID_BACK - LID_GAP + 0.1, px0 - 0.4, px1 + 0.4)
    cuts.append(prism(groove) & slab(w_low - gw, LATCH_POST[1] + gw))
    fw, fd, fh = FINGER_NOTCH
    notch = latch_band(LID_BACK - LID_GAP - fd, LID_BACK - LID_GAP + 0.1, px0 - fw, px1 + fw)
    cuts.append(prism(notch) & slab(-5, fh))
    return cuts


def dummies():
    """プレビュー用: コントローラー基板、XIAO、USB-C、電池。"""
    w_b = LID_T + STANDOFF
    board = prism(board_outline()) & slab(w_b, w_b + BOARD_T)
    vt = board_v_top()
    xiao = local_box(BOARD_CX - XIAO_W / 2, BOARD_CX + XIAO_W / 2, vt - XIAO_D, vt, w_b + BOARD_T, w_b + STACK_H)
    wc = usb_axis_w()
    usb = local_box(BOARD_CX - USB_W / 2, BOARD_CX + USB_W / 2, vt - 7, vt + USB_OVERHANG, wc - USB_H / 2, wc + USB_H / 2)
    battery = prism(battery_outline()) & slab(LID_T, LID_T + BATTERY[2])
    return board, xiao, usb, battery


def latch_test(case, lid_part):
    """ラッチのテストピース: ラッチの周りだけを切り出したボトムケースと底蓋。"""
    x0, x1 = LATCH_X[0] - 4, LATCH_X[1] + 6
    region = latch_band(-1, LID_BACK + 12, x0, x1)
    keep = prism(region, -30, CEILING_Z + 1.5)
    return case & keep, lid_part & prism(region)


FIT_TEST_MARGIN = 6.0         # 合わせ確認用の切り出し: 底蓋の範囲から手前と左右へ広げる量


def fit_test(case):
    """底蓋とボトムケースの合わせ確認用: ボトムケースのうち、底蓋のまわり（手前と左右に FIT_TEST_MARGIN、奥は外面まで）を、
    底面から床の上面までの高さで切り出したもの。床の上面を下にして印刷できる。"""
    m = FIT_TEST_MARGIN
    x0, x1 = LID_X[0] - m, LID_X[1] + m
    y0, y1 = LID_FRONT_Y - TONGUE_LEN - m, 20.0
    region = Pos((x0 + x1) / 2, (y0 + y1) / 2) * Rectangle(x1 - x0, y1 - y0)
    return case & prism(region, -30, B.FLOOR_Z)


def export_board_dxf(path, hole_d=2.2):
    """コントローラー基板の外形と M2 の穴を、基板の面の座標（mm、原点は奥の辺の直線部分の中央、y は奥が正）で
    DXF に書き出す。KiCad の Edge.Cuts に読み込む用。円弧は平面の円弧を基板の面に写したもの（差は 0.01 mm 未満）。"""
    import ezdxf
    vt = board_v_top()

    def local(p):  # 平面の点（底面上）→ 基板の面の座標
        return (p[0] - BOARD_CX, v_of(p[1]) - vt)

    doc = ezdxf.new("R2010")
    doc.units = ezdxf.units.MM
    msp = doc.modelspace()
    outline = board_path()
    for i, (p0, c) in enumerate(outline):
        p1 = outline[(i + 1) % len(outline)][0]
        a, b = local(p0), local(p1)
        if c is None:
            msp.add_line(a, b)
            continue
        # 円弧: 平面での中点を写し、3 点を通る円弧にする
        (cx, cy), r0 = c[1:]
        am = (math.atan2(p0[1] - cy, p0[0] - cx) + math.atan2(p1[1] - cy, p1[0] - cx)) / 2
        m = local((cx + r0 * math.cos(am), cy + r0 * math.sin(am)))
        (ax, ay), (bx, by), (mx, my) = a, b, m
        d = 2 * (ax * (my - by) + mx * (by - ay) + bx * (ay - my))
        ux = ((ax**2 + ay**2) * (my - by) + (mx**2 + my**2) * (by - ay) + (bx**2 + by**2) * (ay - my)) / d
        uy = ((ax**2 + ay**2) * (bx - mx) + (mx**2 + my**2) * (ax - bx) + (bx**2 + by**2) * (mx - ax)) / d
        r = math.hypot(ax - ux, ay - uy)
        s0, s1 = math.degrees(math.atan2(ay - uy, ax - ux)), math.degrees(math.atan2(by - uy, bx - ux))
        if (s1 - s0) % 360 > 180:  # 反時計回りに短い方を描く
            s0, s1 = s1, s0
        msp.add_arc((ux, uy), r, s0, s1)
    for x, v in board_holes_local():
        msp.add_circle((x - BOARD_CX, v - vt), hole_d / 2)
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
    case_piece, lid_piece = latch_test(case, lid_part)
    export([("latch-test-case", case_piece), ("latch-test-lid", Pos(0, -25, 0) * lid_piece)], out / "latch-test-l")
    export([("bay-fit-test-case", fit_test(case))], out / "bay-fit-test-l")
    export_board_dxf(out / "controller-board-l.dxf")
    print(f"lid volume = {lid_part.volume:.2f} mm^3, solids = {len(lid_part.solids())}")
    if "--show" in sys.argv:
        from ocp_vscode import show
        show(case, lid_part, *dummies(), names=["bottom-l", "lid", "board", "xiao", "usb-c", "battery"],
             colors=["#5b8fd6", "#7f77dd", "#3b6d11", "#1d9e75", "#ba7517", "#888780"])
