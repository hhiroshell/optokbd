# XIAO のリセットボタンを、収納部の天井の開口（bay.reset_opening）から押す棒（3D プリント、左右の手で共通）。
# - 床の上面に載る柄の下に、天井の開口に入る位置決めのブロックと、ブロックの下からボタンへ下りる押し当ての突起が付いた形。
#   マトリクス基板と床のフォームを外し、柄を床（天井の部品の上面とボトムケースの床の上面。どちらも B.FLOOR_Z）に沿わせて
#   ブロックを開口に入れ、柄を外（USB-C と反対の向き）と手前へ斜めに軽く引いて、ブロックの角を開口の外の縁と手前の縁の間の
#   角の丸めに当てる。そこで突起がボタンの真上に来るので、柄を押し下げる
# - 細い軸を目分量でボタンに合わせると外れやすいので、位置はケース（開口の縁）で決める。ブロックは開口の縁から
#   LOCATE_FIT 内側の形（角の丸めは同心で半径が LOCATE_FIT 小さい）で、角の丸めに当てた位置（外と手前へ 45° に LOCATE_FIT
#   動かした位置）で突起の中心がボタンの中心に来るように、突起をずらしておく。開口の奥は収納部の
#   奥の壁で、左右の手で位置が違う（奥の外形の円弧の中心がずれている）ので、近い方の壁に合わせる（遠い方の手では奥へ
#   すき間が空くので、手前の縁に当てて決める）。内の側は USB-C から LOCATE_USB_CLEAR 離す
# - 柄の下面が床に当たる所で止まる: 突起の先は XIAO の基板の上面から TIP_STOP 上まで。ボタンの高さによらず押せて、それより
#   下へは行かないので、先が XIAO の基板や部品を突かない（ボタンは押しきると突起がそれ以上入らない）。ブロックの下面は、
#   XIAO の奥の端の部品（bay.XIAO_EDGE_PARTS）の上に LOCATE_CLEAR 空ける
# - 柄はブロックから外へ伸ばし、天井の部品の開口のない所と、ボトムケースの床、重りの穴の蓋の平らな面に載せる。手前へ出すと、
#   柄の下は USB-C と J1 のケーブルの開口で、止まる面がほとんどない。基板は左右の手で同じ向きに入るので、ボタンはどちらの手
#   でも USB-C の左にあり、棒も同じ向きで使う（鏡像の左手の座標では、右手のボタンは USB-C の右、柄は右へ出る）
# - 座標は組み付けた位置の平面とボトムケースの z（左手のケースのボタンの位置に置いたもの）。印刷は上下を逆さにして、柄の
#   上面を下にする。柄、ブロック、突起の順に平面形が小さくなって中に収まるので、サポートが要らない。ブロックと突起の平面形は
#   印刷の向きでも平面の形で決まる
# - 実行: uv run python reset_pin.py [--show]   （out/print/reset-pin.{step,stl,3mf}）
import math
import sys
from pathlib import Path

from build123d import Axis, Box, Pos, Rectangle, Rot, extrude, offset

import bay
import bottom as B
import layout as L

TIP = (1.6, 2.0)              # 押し当ての突起の平面形（x、y）。ボタンの本体（ピン配置図で約 1.6 × 2.7）に収める
TIP_STOP = 0.3                # 柄が床に当たったときの突起の先: XIAO の基板の上面から上へ
LOCATE_FIT = 0.15             # 位置決めのブロックと開口の縁、奥の壁のすき間
LOCATE_USB_CLEAR = 0.3        # ブロックの内の側の面と USB-C の横の面の間
LOCATE_CLEAR = 0.3            # ブロックの下面: XIAO の奥の端の部品の上から（ブロックの手前の端で）
HANDLE = (28.0, 5.0, 4.0)     # 柄の長さ（x。ブロックの内の側の面から外へ）、幅（y。奥の面から手前へ）、高さ（z。床の上面から
                              # 上へ）。長さは、鏡像の左手の座標の右のボタンで、ケースの側面の壁に当たらない長さ
HANDLE_CHAMFER = 0.5          # 柄の上面の縁と外の端の縦の縁の面取り（下面の縁はブロックとの付け根に溝ができないよう残す）


def xiao_top_z(y):
    """ボタンの x の、平面の y での XIAO の基板の上面の z（左右のボタンで同じ）。"""
    w = bay.LID_T + bay.STANDOFF + bay.BOARD_T + bay.XIAO_PCB_T
    vt = bay.board_v_top()
    lo, hi = vt - 10, vt + 10
    for _ in range(60):   # 平面の y が y になる v を探す
        mid = (lo + hi) / 2
        if bay.FRAME.from_local_coords((bay.BOARD_CX + bay.RESET_AT[0], mid, w)).Y < y:
            lo = mid
        else:
            hi = mid
    return bay.FRAME.from_local_coords((bay.BOARD_CX + bay.RESET_AT[0], lo, w)).Z


def locate_back_y():
    """ブロックの奥の面（平面の y）: 左右の手のうち近い方の奥の壁から LOCATE_FIT 手前。"""
    bb = bay.reset_opening().bounding_box()
    ys = []
    for sx in (-1, 1):
        for i in range(21):
            d = bay.USB_W / 2 + LOCATE_USB_CLEAR + (bb.max.X - bay.BOARD_CX - bay.USB_W / 2 - LOCATE_USB_CLEAR) * i / 20
            ys.append(bay.back_y(bay.BOARD_CX + sx * d, bay.LID_BACK - bay.LID_GAP))
    return min(ys) - LOCATE_FIT


def locate_face(sx):
    """位置決めのブロックの平面形（sx = -1 が USB-C の左のボタン、1 が右）: 開口を LOCATE_FIT 内側へ縮めた形のうち、USB-C
    の横から外の縁まで、奥の壁の手前まで。"""
    bb = bay.reset_opening().bounding_box()
    x_in = bay.BOARD_CX + sx * (bay.USB_W / 2 + LOCATE_USB_CLEAR)
    x_out = bb.max.X + 1 if sx > 0 else bb.min.X - 1
    y0, y1 = bb.min.Y - 1, locate_back_y()
    rect = Pos((x_in + x_out) / 2, (y0 + y1) / 2) * Rectangle(abs(x_out - x_in), y1 - y0)
    return offset(bay.reset_opening(), -LOCATE_FIT) & rect


def locate_bottom_z():
    """ブロックの下面の z: ブロックの手前の端（XIAO の基板がいちばん高い所）で、奥の端の部品の上から LOCATE_CLEAR 上。"""
    y_front = bay.reset_opening().bounding_box().min.Y + LOCATE_FIT
    return xiao_top_z(y_front) + bay.XIAO_EDGE_PARTS + LOCATE_CLEAR


def register_shift(sx, k=1.0):
    """基準の位置から、ブロックの角を開口の角の丸めに当てた位置までの移動（平面の dx, dy）: 外と手前へ 45° に LOCATE_FIT。"""
    d = k * LOCATE_FIT / math.sqrt(2)
    return sx * d, -d


def reset_pin(x, y, sx):
    """縁から LOCATE_FIT 離れた位置（基準の位置）に置いた棒: (x, y) はボタンの中心（平面）、sx はボタンが USB-C の
    どちらにあるか（柄を伸ばす向き）。register_shift(sx) だけ動かして角の丸めに当てると、突起の中心がボタンの中心に来る。"""
    loc = locate_face(sx)
    lb = loc.bounding_box()
    x_in = lb.max.X if sx < 0 else lb.min.X
    length, w, h = HANDLE
    y_back = lb.max.Y
    handle = Pos(x_in + sx * length / 2, y_back - w / 2, B.FLOOR_Z + h / 2) * Box(length, w, h)
    top = handle.faces().sort_by(Axis.Z)[-1]
    far = handle.faces().sort_by(Axis.X)[-1 if sx > 0 else 0]
    edges = top.edges() + [e for e in far.edges() if abs(e.tangent_at().Z) > 0.99]
    handle = handle.chamfer(HANDLE_CHAMFER, None, edges)
    z_loc = locate_bottom_z()
    block = Pos(0, 0, z_loc) * extrude(loc, B.FLOOR_Z - z_loc + 0.1)
    # 突起: 角の丸めに当てた位置でボタンの中心に来るよう、基準の位置では逆へずらす。ブロックの平面形の中に収める
    dx, dy = register_shift(sx)
    tx, ty = x - dx, y - dy
    tip_face = (Pos(tx, ty) * Rectangle(*TIP)) & loc
    z_tip = xiao_top_z(y) + TIP_STOP
    tip = Pos(0, 0, z_tip) * extrude(tip_face, z_loc - z_tip + 0.1)
    return handle + block + tip


def check(case, ceiling, weight_lids):
    """左手の基板と右手の基板のボタン（どちらも左手のケースの座標）の位置で確かめる:
    - 基準の位置でも、縁に当てた位置（の少し手前）でも、ボトムケース、天井の部品、重りの穴の蓋に当たらないこと
    - 柄の下面がそれらの上面（床の上面）に十分載ること
    - 突起が開口の範囲（天井の下面より下）とボタンの本体に収まり、ブロックが天井の厚み全体で縁に当たること"""
    loc_bottom = locate_bottom_z()
    assert TIP[0] <= 1.6 and TIP[1] <= 2.7
    for x, y in bay.reset_points():
        sx = 1 if x > bay.BOARD_CX else -1
        tool = reset_pin(x, y, sx)
        area = 0.0
        for part in (case, ceiling, weight_lids):
            for t in (tool, Pos(*register_shift(sx, 0.95), 0) * tool):
                hit = t & part
                assert hit is None or hit.volume < 1e-6, (x, y)
            rest = (Pos(0, 0, -0.1) * tool) & part   # 柄の下面のすぐ下（0.1 mm）にある面
            area += 0.0 if rest is None else rest.volume / 0.1
        assert area >= 0.5 * HANDLE[0] * HANDLE[1], (x, y)
        # ブロックの下面が天井の下面より下（縁に当たる高さが天井の厚み全体）
        y_front = bay.reset_opening().bounding_box().min.Y
        ceiling_bottom = xiao_top_z(y_front) + (bay.CEILING_W - (bay.LID_T + bay.STANDOFF + bay.BOARD_T + bay.XIAO_PCB_T)) \
            * bay.FRAME.z_dir.Z
        assert loc_bottom < ceiling_bottom - 0.3


def print_pose(part):
    """印刷の向き: 上下を逆さにして、柄の上面を下（z = 0）にする。"""
    flipped = Rot(180, 0, 0) * part
    bb = flipped.bounding_box()
    return Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * flipped


if __name__ == "__main__":
    case = B.build()
    ceiling = bay.ceiling()
    check(case, ceiling, bay.weight_lids())
    x, y = bay.reset_points()[0]
    tool = reset_pin(x, y, -1)
    out = Path(__file__).parent / "out" / "print"
    out.mkdir(parents=True, exist_ok=True)
    L.export([("reset-pin", print_pose(tool))], out / "reset-pin")
    lb = locate_face(-1).bounding_box()
    print(f"block = {lb.size.X:.2f} x {lb.size.Y:.2f}, bottom z = {locate_bottom_z():.2f}, "
          f"tip length = {locate_bottom_z() - xiao_top_z(y) - TIP_STOP:.2f}, volume = {tool.volume:.1f} mm^3")
    if "--show" in sys.argv:
        from ocp_vscode import show
        board, xiao, usb, battery = bay.dummies()
        show(case, ceiling, board, xiao, usb, tool,
             names=["bottom-l", "ceiling", "board", "xiao", "usb-c", "reset-pin"],
             colors=["#5b8fd6", "#d98c5f", "#3b6d11", "#1d9e75", "#ba7517", "#e03030"])
