# コントローラー基板のはんだ付けの治具（3D プリント、左右の基板で共通）。
# - 表向きの台（solder-jig-top）: 基板を部品の面を上にして載せる。J1 と XIAO のはんだ付けに使う
#   - 取り付け穴 H1〜H4 にピンを通して基板の位置を決め、穴のまわりの台で受ける（基板の下はケースと同じ STANDOFF の空き）。
#     ネジ頭やナットは H1・H2 の横の XIAO のパッドにこてが入らなくなるので使わない。押さえはカプトンテープ
#   - J1 の本体の下に受けの帯（ピンにこてを当てたときに基板がたわまないように。パッドの真下は避ける）
#   - 奥の壁: XIAO の奥の端を当て、USB-C のレセプタクルを上の開いた溝（下半分が長円）に落とし込んで XIAO の位置を決める。
#     ケースと同じく「基板の穴と USB-C」で位置が決まるので、付けた XIAO の USB-C はケースの穴（すき間 bay.USB_CLEAR）に合う。
#     基板の縁と、XIAO の上の奥の端の部品（ボタンや LED）には当てない（壁を少し下げる）。以前は長円の穴に通していたが、
#     はんだ付けした後に抜けなかった（基板はピンで真上にしか動かせず、USB-C は奥へしか抜けない）。溝なら基板ごと真上に外せる
# - 裏返しの台（solder-jig-flip）: 基板を部品の面を下にして載せる（基板の y 軸のまわりに裏返すので、x は鏡像）。XIAO の電池の
#   穴（VBAT、GND）と、J2 / J3 のはんだ付けに使う
#   - H1〜H4 の位置に、XIAO と USB-C より高い柱とピン
#   - J2 / J3 の本体の下に低い受けの段: 本体が載ると、基板の重さで本体が基板に押し付けられる（浮かない）。KiCad の 3D モデルの
#     高さ（bay.J2_H）から決めた段（高さ 0.55）では高すぎ、段なしでは実物の J2 が少し浮いたので、その半分ほどにした
#   - USB-C の下に受け（電池の穴にこてを当てたときに XIAO を支える）。基板を持ち上げないよう少し低くする
# - はんだ付けする所から、樹脂（PLA は 60 ℃ 前後で柔らかくなる）を離す: check() で確かめる
# - 座標は基板の面の座標（mm、原点は奥の辺の直線部分の中央、y は奥が正。KiCad の補助原点と同じ）。台の上面が z = 0
# - 実行: uv run python jig.py [--show]   （pcbnew の使える /usr/bin/python3 が必要。out/solder-jig-*.{step,stl,3mf}）
import json
import math
import subprocess
import sys
from pathlib import Path

from build123d import Box, Plane, Pos, RectangleRounded, SlotOverall, Solid, extrude

import bay

KICAD_PCB = Path(__file__).resolve().parent.parent / "pcbs" / "controller" / "controller.kicad_pcb"

BASE_T = 2.4                  # 台の板の厚さ
BASE_MARGIN = 5.0             # 台の板: 基板の外形から外へ
BASE_R = 3.0                  # 台の板の角の丸め
PIN_D = 2.0                   # 位置決めのピン（基板の穴は Ø2.2）
PIN_OVER = 0.8                # ピンの先: 基板の上面から
PIN_TIP = 0.3                 # ピンの先の面取り

# 表向きの台
TOP_BOSS_D = 4.0              # 穴のまわりの台（ケースの台座は Ø5.0。H1・H2 の横の XIAO のパッドから離すため細くする）
J1_SUPPORT = ((-5.0, 5.0), (-25.0, -23.8))   # J1 の本体の下の受けの帯（x, y の範囲）: 信号のピンのパッド（y −21.6〜−23.2）と
                                              # 固定用のパッド MP（y −25.4〜−27.2、x ±5.7〜6.9）の間
FENCE_T = 3.0                 # 奥の壁の厚さ（XIAO の奥の端から奥へ）
FENCE_HALF = bay.USB_W / 2 + 1.5   # 奥の壁の幅の半分（XIAO の角のパッドから離す）
FENCE_BOARD_RELIEF = 0.3      # 基板の高さでは壁を奥へ下げる（基板の縁の加工の誤差で、XIAO より先に基板が当たらないように）
FENCE_TOP_RELIEF = 0.5        # XIAO の上面より上も壁を奥へ下げる（XIAO の奥の端の部品に当てない）
FENCE_ABOVE = 1.5             # 奥の壁の上端: USB-C の上端から
USB_FIT = 0.15                # USB-C の溝: レセプタクルの長円から外へ（要実測。きつければ広げる）
PIN_RECESS_D = 3.0            # J2 / J3 の足の逃げの穴（台の板を貫通）

# 裏返しの台
FLIP_POST_D = 3.6             # 柱（基板の部品の面に当たる。XIAO のパッド、J2 / J3 から離す）
FLIP_CLEAR = 0.7              # 基板の部品の面から台の上面まで: XIAO と USB-C（基板の上面から bay.STACK_H − 基板の厚さ）より上に
J2_SUPPORT_H = 0.3            # J2 / J3 の受けの段の高さ（台の上面から。実物に合わせた値）
J2_SUPPORT_INSET = 0.5        # J2 / J3 の受けの平面形: コートヤードから内へ
USB_SUPPORT = ((-3.5, 3.5), (-5.0, 0.0))     # USB-C の下の受け（x, y の範囲。鏡像にする前）
USB_SUPPORT_GAP = 0.1         # USB-C の受けと USB-C の上面のすき間

HEAT_CLEAR = 1.5              # はんだ付けする所（パッドの縁）と樹脂の間の平面の距離の下限（台の上面より上の部分）

XIAO_H = bay.STACK_H - bay.BOARD_T   # 基板の上面から XIAO と USB-C の上端まで（4.5）
FLIP_H = XIAO_H + FLIP_CLEAR         # 裏返しの台: 柱の上端（基板の部品の面）の高さ


# ---- KiCad ----

_FOOTPRINTS = r"""
import json, sys
import pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
o = b.GetDesignSettings().GetAuxOrigin()
def pt(x, y):
    return [pcbnew.ToMM(x - o.x), -pcbnew.ToMM(y - o.y)]
out = {"courtyards": {}, "pads": []}
for fp in b.GetFootprints():
    ps = fp.GetCourtyard(pcbnew.F_CrtYd)
    if ps.OutlineCount():
        bb = ps.BBox()
        out["courtyards"][fp.GetReference()] = pt(bb.GetX(), bb.GetBottom()) + pt(bb.GetRight(), bb.GetY())
    for pad in fp.Pads():
        if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
            continue
        bb = pad.GetBoundingBox()
        out["pads"].append([fp.GetReference() + "-" + pad.GetNumber(), pad.IsOnLayer(pcbnew.F_Cu), pad.IsOnLayer(pcbnew.B_Cu)]
                           + pt(bb.GetX(), bb.GetBottom()) + pt(bb.GetRight(), bb.GetY()))
print(json.dumps(out))
"""


XIAO_UNDER_PADS = {"U1-15", "U1-16", "U1-17", "U1-18", "U1-21", "U1-22"}   # XIAO の裏の小さなパッド（はんだ付けしない）


def kicad_footprints(pcb=KICAD_PCB):
    """KiCad の基板の、表の部品のコートヤード {参照: (x0, y0, x1, y1)} と、はんだ付けするパッド
    [(名前, 表, 裏, x0, y0, x1, y1)]（基板の面の座標。XIAO の裏の小さなパッドを除く）。"""
    res = subprocess.run(["/usr/bin/python3", "-c", _FOOTPRINTS, str(pcb)], check=True, capture_output=True, text=True)
    data = json.loads(res.stdout.strip().splitlines()[-1])
    pads = [tuple(p) for p in data["pads"] if p[0] not in XIAO_UNDER_PADS]
    return {k: tuple(v) for k, v in data["courtyards"].items()}, pads


# ---- 形 ----

def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def cylinder(x, y, z0, z1, d):
    return Pos(x, y, z0) * Solid.make_cylinder(d / 2, z1 - z0)


def pin(x, y, z0, z1):
    """位置決めのピン（先を面取り）。"""
    body = cylinder(x, y, z0, z1 - PIN_TIP, PIN_D)
    return body + Pos(x, y, z1 - PIN_TIP) * Solid.make_cone(PIN_D / 2, PIN_D / 2 - PIN_TIP, PIN_TIP)


def base(y1):
    """台の板（上面が z = 0）。基板の外形から BASE_MARGIN 外へ、奥は y1 まで。"""
    x0, x1 = -bay.BOARD_W / 2 - BASE_MARGIN, bay.BOARD_W / 2 + BASE_MARGIN
    y0 = -bay.BOARD_D - BASE_MARGIN
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, -BASE_T) * extrude(RectangleRounded(x1 - x0, y1 - y0, BASE_R), BASE_T)


def usb_hole(z_axis, y0, y1, grow):
    """USB-C のレセプタクルの長円（中心の高さ z_axis）を grow だけ広げて、y0 から y1 まで押し出したもの。"""
    plane = Plane(origin=(0, y0, z_axis), x_dir=(1, 0, 0), z_dir=(0, 1, 0))
    return extrude(plane * SlotOverall(bay.USB_W + 2 * grow, bay.USB_H + 2 * grow), y1 - y0)


def top_jig():
    """表向きの台。基板の下面が z = bay.STANDOFF。"""
    z_b = bay.STANDOFF
    z_t = z_b + bay.BOARD_T
    z_x = z_t + bay.XIAO_PCB_T
    z_u = z_x + bay.USB_H / 2
    jig = base(FENCE_T + 2.0)
    for x, y in bay.board_holes_board():
        jig += cylinder(x, y, -0.1, z_b, TOP_BOSS_D)
        jig += pin(x, y, z_b - 0.1, z_t + PIN_OVER)
    (xa, xb), (ya, yb) = J1_SUPPORT
    jig += box(xa, xb, ya, yb, -0.1, z_b)
    # 奥の壁: XIAO の奥の端（y = 0。bay と同じく基板の奥の辺の直線部分と同じ位置）に当てる面は、XIAO の基板の高さだけ
    h = FENCE_HALF
    top = z_u + bay.USB_H / 2 + USB_FIT + FENCE_ABOVE
    fence = box(-h, h, FENCE_BOARD_RELIEF, FENCE_T, -0.1, z_t)
    fence += box(-h, h, 0.0, FENCE_T, z_t, z_x)
    fence += box(-h, h, FENCE_TOP_RELIEF, FENCE_T, z_x, top)
    fence -= usb_hole(z_u, -1.0, FENCE_T + 1.0, USB_FIT)
    w = bay.USB_W / 2 + USB_FIT
    fence -= box(-w, w, -1.0, FENCE_T + 1.0, z_u, top + 1.0)   # 長円の上半分から上を抜いて、上の開いた溝にする
    jig += fence
    # J2 / J3 の足の逃げ（付けた後の基板を載せ直すとき用。足は基板の下面から約 2.4 出る）
    for pair in bay.BATTERY_PINS:
        for x, y in pair:
            jig -= cylinder(x, y, -BASE_T - 1, 1.0, PIN_RECESS_D)
    return jig


def flip_jig(courtyards):
    """裏返しの台。基板の部品の面が z = FLIP_H（基板の x は鏡像）。"""
    def mx(x):
        return -x

    jig = base(3.0)
    for x, y in bay.board_holes_board():
        jig += cylinder(mx(x), y, -0.1, FLIP_H, FLIP_POST_D)
        jig += pin(mx(x), y, FLIP_H - 0.1, FLIP_H + bay.BOARD_T + PIN_OVER)
    for ref in ("J2", "J3"):
        x0, y0, x1, y1 = courtyards[ref]
        i = J2_SUPPORT_INSET
        jig += box(min(mx(x0), mx(x1)) + i, max(mx(x0), mx(x1)) - i, y0 + i, y1 - i, -0.1, J2_SUPPORT_H)
    (xa, xb), (ya, yb) = USB_SUPPORT
    z_u = FLIP_H - bay.XIAO_PCB_T - bay.USB_H - USB_SUPPORT_GAP
    jig += box(min(mx(xa), mx(xb)), max(mx(xa), mx(xb)), ya, yb, -0.1, z_u)
    return jig


# ---- 確かめること ----

def rect_dist(a, b):
    """平面の 2 つの長方形 (x0, y0, x1, y1) の距離（重なれば 0）。"""
    dx = max(a[0] - b[2], b[0] - a[2], 0.0)
    dy = max(a[1] - b[3], b[1] - a[3], 0.0)
    return math.hypot(dx, dy)


def check(top, flip, courtyards, pads):
    """部品・パッドと治具の距離。返り値は確認用の数値。"""
    res = {}
    # 表向きの台: 基板の上（部品の面）に出るのは奥の壁とピンだけ。奥の壁と、表のパッドの距離
    z_t = bay.STANDOFF + bay.BOARD_T
    fence = top & box(-50, 50, 0.0 - 1e-6, 50, z_t, 50)
    fb = fence.bounding_box()
    f_rect = (fb.min.X, fb.min.Y, fb.max.X, fb.max.Y)
    res["top: fence to F pads"] = min(rect_dist(f_rect, p[3:]) for p in pads if p[1])
    # 表向きの台: 基板の下の台と、J1 の信号のピンのパッド（パッドの真下を避ける）
    j1 = [p for p in pads if p[0].startswith("J1-")]
    (xa, xb), (ya, yb) = J1_SUPPORT
    res["top: J1 support to J1 pads"] = min(rect_dist((xa, ya, xb, yb), p[3:]) for p in j1)
    # 裏返しの台: 柱と、表の部品のコートヤード（鏡像）
    posts = [(-x - FLIP_POST_D / 2, y - FLIP_POST_D / 2, -x + FLIP_POST_D / 2, y + FLIP_POST_D / 2)
             for x, y in bay.board_holes_board()]

    def mirrored(r):
        return (-r[2], r[1], -r[0], r[3])

    res["flip: posts to courtyards"] = min(rect_dist(a, mirrored(c)) for a in posts
                                           for ref, c in courtyards.items() if not ref.startswith("H"))
    # 裏返しの台: USB-C の受けと、電池の穴（裏からこてを当てる。真下の XIAO を介して熱が伝わる）
    (xa, xb), (ya, yb) = USB_SUPPORT
    bat = [p for p in pads if p[0] in ("U1-19", "U1-20")]
    assert len(bat) == 2
    res["flip: USB support to battery holes"] = min(rect_dist((xa, ya, xb, yb), p[3:]) for p in bat)
    return res


if __name__ == "__main__":
    courtyards, pads = kicad_footprints()
    # 取り付け穴が bay の穴と同じ位置にあること（ピンの位置）
    holes = sorted((round((c[0] + c[2]) / 2, 3), round((c[1] + c[3]) / 2, 3)) for r, c in courtyards.items() if r.startswith("H"))
    assert all(abs(a - c) < 1e-2 and abs(b - d) < 1e-2 for (a, b), (c, d) in zip(holes, sorted(bay.board_holes_board()))), holes
    top, flip = top_jig(), flip_jig(courtyards)
    res = check(top, flip, courtyards, pads)
    for k, v in res.items():
        print(f"{k} = {v:.2f}")
    assert res["top: fence to F pads"] >= HEAT_CLEAR
    assert res["flip: posts to courtyards"] >= 0.3
    assert res["top: J1 support to J1 pads"] >= 0.3
    assert res["flip: USB support to battery holes"] >= 0.5
    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    bay.export([("solder-jig-top", top)], out / "solder-jig-top")
    bay.export([("solder-jig-flip", flip)], out / "solder-jig-flip")
    print(f"top volume = {top.volume:.2f} mm^3, solids = {len(top.solids())}, valid = {top.is_valid}")
    print(f"flip volume = {flip.volume:.2f} mm^3, solids = {len(flip.solids())}, valid = {flip.is_valid}")
    if "--show" in sys.argv:
        from ocp_vscode import show
        show(top, Pos(0, -50, 0) * flip, names=["solder-jig-top", "solder-jig-flip"])
