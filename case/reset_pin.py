# XIAO のリセットボタンを、収納部の天井の開口（bay.reset_opening）から押す棒（3D プリント、左右の手で共通）。
# - 柄（床の上面に載る棒）の端から、角の軸が下へ出る L 字形。マトリクス基板と床のフォームを外し、柄を床（天井の部品の上面と
#   ボトムケースの床の上面。どちらも B.FLOOR_Z）に沿わせて、軸を開口からボタンの上へ下ろして押す
# - 柄の下面が床に当たる所で止まる: 軸の先は XIAO の基板の上面から TIP_STOP 上まで。ボタンの高さによらず押せて、それより
#   下へは行かないので、先が XIAO の基板や部品を突かない（ボタンは押しきると軸がそれ以上入らない）
# - 柄はボタンから USB-C と反対の向き（左）へ横に出し、軸の奥の面と柄の奥の面をそろえて柄を手前へ広げる。その範囲は、
#   天井の部品の開口のない所と、ボトムケースの床、重りの穴の蓋の平らな面なので柄が載る。手前へ出すと、柄の下は USB-C と
#   J1 のケーブルの開口で、止まる面がほとんどない。基板は左右の手で同じ向きに入るので、ボタンはどちらの手でも USB-C の左に
#   あり、棒も同じ向きで使う（鏡像の左手の座標では、右手のボタンは USB-C の右、柄は右へ出る）
# - 座標は組み付けた位置の平面とボトムケースの z（左手のケースのボタンの位置に置いたもの）。印刷するときは横に寝かせ、
#   軸と柄の奥の面（y の大きい側）を下にする。柄と軸の形はどちらも、その面から上へ押し出した形になるので
#   サポートが要らない。軸の太さも柄の下面（止まる面）の位置も、印刷の向きでは層の重なりではなく平面の形で決まる
# - 実行: uv run python reset_pin.py [--show]   （out/print/reset-pin.{step,stl,3mf}）
import sys
from pathlib import Path

from build123d import Axis, Box, Pos, Rot

import bay
import bottom as B
import layout as L

PIN_W = 1.2                   # 軸の断面（正方形）の辺。天井の開口の縁とボタンの中心の間（bay.reset_access_margin()）より細く
TIP_STOP = 0.3                # 柄が床に当たったときの軸の先: XIAO の基板の上面から上へ
HANDLE = (35.0, 5.0, 4.0)     # 柄の長さ（x。軸の外の面から外へ）、幅（y。軸の奥の面から手前へ）、高さ（z。床の上面から上へ）
HANDLE_CHAMFER = 0.5          # 柄の縁の面取り（印刷でベッドに当たる面と、軸の側の端の面の縁は除く）


def xiao_top_z():
    """リセットボタンの中心の位置の、XIAO の基板の上面の z（左右のボタンで同じ）。"""
    w = bay.LID_T + bay.STANDOFF + bay.BOARD_T + bay.XIAO_PCB_T
    return bay.FRAME.from_local_coords((bay.BOARD_CX + bay.RESET_AT[0], bay.board_v_top() + bay.RESET_AT[1], w)).Z


def pin_len():
    """軸の長さ: 柄の下面（床の上面）から先まで。"""
    return B.FLOOR_Z - (xiao_top_z() + TIP_STOP)


def reset_pin(x, y, sx):
    """押す位置に置いた棒: 軸の中心が平面の (x, y)、柄の下面が床の上面の高さ。柄は軸から sx の向き（-1 が左、1 が右）へ
    伸ばし、奥の面（印刷でベッドに当たる面）を軸とそろえる。"""
    length, w, h = HANDLE
    x0 = x - sx * PIN_W / 2   # 柄の軸の側の端（軸の内の面）
    handle = Pos(x0 + sx * (PIN_W + length) / 2, y + PIN_W / 2 - w / 2, B.FLOOR_Z + h / 2) * Box(PIN_W + length, w, h)
    bed = handle.faces().sort_by(Axis.Y)[-1]
    # 軸の側の端の面は軸の内の面とそろうので、その縁も面取りしない（下の縁を面取りすると、軸の付け根に V 字の溝ができて
    # 折れやすい）
    end = handle.faces().sort_by(Axis.X)[0 if sx > 0 else -1]
    keep = bed.edges() + end.edges()
    handle = handle.chamfer(HANDLE_CHAMFER, None, [e for e in handle.edges() if e not in keep])
    shaft = Pos(x, y, B.FLOOR_Z - pin_len() / 2 + 0.05) * Box(PIN_W, PIN_W, pin_len() + 0.1)
    return handle + shaft


def check(case, ceiling, weight_lids):
    """左手の基板と右手の基板のボタン（どちらも左手のケースの座標）の位置に置いたとき、棒がボトムケース、天井の部品、
    重りの穴の蓋に当たらないこと。柄の下面がそれらの上面（床の上面）に十分載ること。軸の太さが開口の中で動かせる余裕を
    残すこと。"""
    assert PIN_W / 2 <= bay.reset_access_margin() - 0.3
    for x, y in bay.reset_points():
        sx = 1 if x > bay.BOARD_CX else -1
        tool = reset_pin(x, y, sx)
        area = 0.0
        for part in (case, ceiling, weight_lids):
            hit = tool & part
            assert hit is None or hit.volume < 1e-6, (x, y)
            rest = (Pos(0, 0, -0.1) * tool) & part   # 柄の下面のすぐ下（0.1 mm）にある面
            area += 0.0 if rest is None else rest.volume / 0.1
        assert area >= 0.5 * HANDLE[0] * HANDLE[1], (x, y)


def print_pose(part):
    """印刷の向き: 柄と軸の奥の面を下（z = 0）に寝かせる。"""
    bb = part.bounding_box()
    return Rot(-90, 0, 0) * Pos(-bb.center().X, -bb.max.Y, -bb.center().Z) * part


if __name__ == "__main__":
    case = B.build()
    ceiling = bay.ceiling()
    check(case, ceiling, bay.weight_lids())
    x, y = bay.reset_points()[0]
    tool = reset_pin(x, y, -1)
    out = Path(__file__).parent / "out" / "print"
    out.mkdir(parents=True, exist_ok=True)
    L.export([("reset-pin", print_pose(tool))], out / "reset-pin")
    print(f"pin length = {pin_len():.2f}, XIAO top z = {xiao_top_z():.2f}, volume = {tool.volume:.1f} mm^3")
    if "--show" in sys.argv:
        from ocp_vscode import show
        board, xiao, usb, battery = bay.dummies()
        show(case, ceiling, board, xiao, usb, tool,
             names=["bottom-l", "ceiling", "board", "xiao", "usb-c", "reset-pin"],
             colors=["#5b8fd6", "#d98c5f", "#3b6d11", "#1d9e75", "#ba7517", "#e03030"])
