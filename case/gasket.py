# ガスケット（左手、モック用）。プレートのタブの上下に接着する硬いパッドで、タブ 1 つにつき上下 2 枚。
# - 本物のガスケットのようには縮まないので、ボトムケースの柱からトップケースのポケットの天井までの高さに、
#   プレートと上下 2 枚がほぼぴったり収まる厚さにする
# - 上側のガスケットはベゼルの下面（top.CAVITY_TOP）より高くなるので、平面形はポケットの中（ケースの内面より外）に収める
# - 座標はプレート DXF と同じ。z はトップケースの座標
# - 実行: uv run python gasket.py [--show]   （out/ に印刷用の配置で STEP / STL / 3MF を書き出す。--show で組み付けた状態を表示）
import sys
from pathlib import Path

from build123d import Compound, Mesher, Pos, export_step, export_stl, extrude

import bottom as B
import layout as L
import plate as P
import top as T

PLATE_T = 1.6         # プレートの厚さ
PLAY = 0.1            # 上下方向の遊び（プレートと上下のガスケットを重ねた高さと、収める高さの差）
INSET = 0.2           # タブの両側と先端から内側へ
INNER_CLEARANCE = 0.2  # ケースの内面（ポケットの内側の縁）とのすき間

PILLAR_TOP = B.HEIGHT - L.Z_OFFSET        # ボトムケースの柱の上端（トップケースの座標で 13.5）
STACK = T.SLOT_TOP - PILLAR_TOP           # 柱の上からポケットの天井まで（6.5）
THICKNESS = (STACK - PLATE_T - PLAY) / 2  # 2.4
LOWER_Z = PILLAR_TOP
PLATE_Z = LOWER_Z + THICKNESS
UPPER_Z = PLATE_Z + PLATE_T


def footprint(tab):
    """ガスケットの平面形: タブのうち、ケースの内面 + INNER_CLEARANCE から先端 - INSET までを、両側から INSET 削ったもの。"""
    ring = P.base(L.PLATE_GAP + L.BOTTOM_WALL - INSET) - P.base(L.PLATE_GAP + INNER_CLEARANCE)
    return ring & L.tab_strip(tab, tab.width / 2 - INSET)


def gaskets():
    """[(名前, 組み付けた位置のガスケット), ...]"""
    out = []
    names = {P.I_BACK: "back", P.I_FRONT_LEFT: "front-left", P.I_THUMB: "thumb"}
    for tab in P.tabs():
        f = footprint(tab)
        name = f"{names.get(tab.edge, tab.edge)}-x{tab.cx:.0f}"
        out.append((f"{name}-lower", Pos(0, 0, LOWER_Z) * extrude(f, THICKNESS)))
        out.append((f"{name}-upper", Pos(0, 0, UPPER_Z) * extrude(f, THICKNESS)))
    return out


def print_layout(parts):
    """印刷用の配置: すべて z = 0 に置き、上側のガスケットは下側の 8 mm 奥に並べる。"""
    placed = []
    for name, g in parts:
        bb = g.bounding_box()
        dy = 8 if name.endswith("upper") else 0
        placed.append((name, Pos(0, dy, -bb.min.Z) * g))
    return placed


if __name__ == "__main__":
    parts = gaskets()
    placed = print_layout(parts)
    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    compound = Compound([g for _, g in placed])
    export_step(compound, out / "gasket-l.step")
    export_stl(compound, out / "gasket-l.stl")
    mesher = Mesher()  # 3D プリンタのスライサー用。単位は mm。部品ごとに名前をつける
    for name, g in placed:
        mesher.add_shape(g, part_number=name)
    mesher.write(out / "gasket-l.3mf")
    print(f"thickness = {THICKNESS:.2f} mm, plate z = {PLATE_Z:.2f}..{PLATE_Z + PLATE_T:.2f}")
    for name, g in parts:
        bb = g.bounding_box()
        print(f"{name:22s} {bb.max.X - bb.min.X:.2f} x {bb.max.Y - bb.min.Y:.2f} mm")
    if "--show" in sys.argv:
        from ocp_vscode import show
        show(*[g for _, g in parts], names=[n for n, _ in parts])
