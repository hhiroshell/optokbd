# 組み付けを試すためのマトリクス基板（左手）のモック。キースイッチでプレートに固定する（本物と同じ）。
# - 外形はプレートの外形（タブを除く）を、奥と手前の辺は MATRIX_INSET だけ内側に、左右と右下の斜めの辺は
#   MATRIX_SIDE_GROW だけ外側にずらしたもの。厚さは KiCad の基板と同じ 1.6
# - スイッチの穴は KiCad の基板（pcbs/matrix-left/matrix-left.kicad_pcb）のドリルデータから取る（中央の Ø4、
#   左右の突起の Ø2、ホットスワップソケットのピンの Ø3）。中央の穴はスイッチの軸にきつめにはまる大きさにして、
#   基板がスイッチから抜け落ちないようにする（Ø4.1 では印刷したモックからスイッチが簡単に抜けたので Ø3.9 にした）
# - 裏面に、ホットスワップソケット（Kailh MX）を大まかにかたどった形を付ける。向きはスイッチの 2 つのピンの穴から求める
#   （R1C3 のように 180° 回したスイッチや、親指のキーの回転もそのまま反映される）
# - KiCad の基板の座標とプレートの座標の対応: x = X - 50、y = 50 - Y（KiCad の Y は下が正）。スイッチの中心が
#   plate.keys() と一致することを確かめる
# - キーの配置スクリプト（scripts/place_matrix.py）の設定（反転するキー FLIP_KEYS と J1 の位置）を読み込んで反映する。
#   KiCad の基板でスクリプトを実行する前でも後でも、同じモックになる（すでに反転しているスイッチはそのまま）
# - J1 はモックに付けない（組み付けの試しでは、実物のコネクタを両面テープで貼る）。J1 の位置は、ボトムケースの天井の
#   開口（bay.floor_openings）を決めるのに使う。プレビュー用の形は matrix_j1()
# - 高さ（ボトムケースの座標）: プレートの上面 24.0 から、MX の規格で基板の上面は 5.0 下
# - 実行: uv run python matrix_mock.py [--show]   （kicad-cli が必要。out/matrix-mock-l.* を印刷の向き
#   （基板の上面を下、ソケットが上）で書き出し、KiCad の Edge.Cuts に入れる外形を out/matrix-board-l-outline.json に
#   書き出す。--show でボトムケース、底蓋、コントローラーと組み付けた状態を表示）
import json
import math
import re
import subprocess
import sys
import tempfile
from pathlib import Path

from build123d import GeomType, Pos, Rectangle, Rot, Solid, extrude

import bottom as B
import gasket as G
import layout as L
import plate as P

KICAD_PCB = Path(__file__).resolve().parent.parent / "pcbs" / "matrix-left" / "matrix-left.kicad_pcb"
KICAD_OFFSET = (50.0, 50.0)   # KiCad の座標 (X, Y) → プレートの座標 (X - 50, 50 - Y)
MATRIX_T = 1.6                # 基板の厚さ
MATRIX_INSET = 0.75           # 外形の奥と手前の辺: プレートの外形から内側へ
MATRIX_SIDE_GROW = 0.3        # 外形の左右と右下の斜めの辺: プレートの外形から外へ。端の列のソケット（R0C0 と、反転した
                              # R1〜R3 の C6）のはんだパッドを基板端から 0.5 mm 以上離す。この 3 辺にはボトムケースの
                              # 壁がなく、トップケースの内面はプレートから L.TOP_CAVITY["side"] 外にある
PLATE_TO_PCB = 5.0            # MX: プレートの上面から基板の上面まで
PCB_TOP_Z = G.UPPER_Z + L.Z_OFFSET - PLATE_TO_PCB   # 基板の上面（ボトムケースの座標で 19.0）

CENTER_D = 4.0                # スイッチの中央の穴（KiCad の値）
CENTER_PRINT_D = 3.9          # 印刷する中央の穴（スイッチの軸 Ø4.0 より小さく、押し込んできつくはまる）
HOLE_ALLOWANCE = 0.3          # そのほかの穴の直径に足す量（印刷で細る分）

# ホットスワップソケット（Kailh MX）の大まかな形: スイッチの中心を原点、y は奥が正（スイッチを上から見た向き）。
# ピンの穴は (-3.81, 2.54) と (2.54, 5.08)。本体は 2 つの長方形をずらして重ねた形、両端にハンダ付けの端子
SOCKET_H = 1.8                # 本体の高さ（基板の裏から下へ）
SOCKET_TAB_H = 0.5            # 端子の高さ
SOCKET_BODY = [(-5.6, 0.6, 0.3, 4.6), (-1.3, 3.0, 4.6, 7.0)]   # (x0, y0, x1, y1)
SOCKET_TABS = [(-8.65, 1.35, -5.35, 3.8), (4.05, 3.9, 7.35, 6.35)]
PIN_LOCAL = ((-3.81, 2.54), (2.54, 5.08))

PLACE_SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "place_matrix.py"


def place_settings(path=PLACE_SCRIPT):
    """配置スクリプトの設定とキーの配置（KEYS まで）を読む。スクリプトの残り（pcbnew を使う部分）は実行しない。"""
    text = path.read_text()
    ns = {"math": math}
    exec(text[text.index("# ===== 設定 ====="):text.index("ROW_RE = ")], ns)
    return ns


_S = place_settings()
# 反転するキーの中心（プレートの座標。スクリプトの KEYS は y が下向き）。左手（MIRROR = False）として読む
FLIP_AT = [(_S["KEYS"][k][0], -_S["KEYS"][k][1]) for k in sorted(_S["FLIP_KEYS"])]
_jx, _jy, _jr = _S["KEYS"][_S["J1_KEY"]]
assert _jr == 0 and _S["J1_ANGLE"] == 0
J1_AT = (_jx + _S["J1_OFFSET"][0], -(_jy + _S["J1_OFFSET"][1]))   # 裏面の J1 の原点
# 裏面の J1（JST SM11B-SRSS-TB、横型）の大まかな形: フットプリントの原点からの範囲（プレートの向き、y は奥が正）。
# 差し込み口は手前（コントローラーの側）で、手前の端はコートヤード、奥の端は本体（Fab）。本体の高さは 2.95
J1_BODY = (-7.0, -3.3, 7.0, 1.75)
J1_H = 2.95


def kicad_holes(pcb=KICAD_PCB):
    """KiCad の基板の穴をプレートの座標で: [(x, y, 直径)]。"""
    with tempfile.TemporaryDirectory() as d:
        subprocess.run(["kicad-cli", "pcb", "export", "drill", "--format", "excellon", "--drill-origin", "absolute",
                        "--excellon-units", "mm", "--excellon-zeros-format", "decimal", "-o", d + "/", str(pcb)],
                       check=True, capture_output=True)
        text = "".join(p.read_text() for p in Path(d).glob("*.drl"))
    tools = {t: float(dia) for t, dia in re.findall(r"^T(\d+)C([\d.]+)", text, re.M)}
    holes, tool = [], None
    for line in text.split("%", 1)[1].splitlines():
        if m := re.fullmatch(r"T(\d+)", line):
            tool = m.group(1)
        elif m := re.fullmatch(r"X(-?[\d.]+)Y(-?[\d.]+)", line):
            # ドリルデータの Y は上が正（KiCad の画面の Y の符号を反転したもの）
            holes.append((float(m.group(1)) - KICAD_OFFSET[0], float(m.group(2)) + KICAD_OFFSET[1], tools[tool]))
        elif line.startswith(("G85", "G00", "M15")):
            raise ValueError("長穴（ルート加工）には対応していない: " + line)
    return holes


def switches(holes):
    """スイッチの中心と向き（度、反時計回り）: [(x, y, angle)]。中央の穴と、その近くの 2 つのピンの穴から求める。"""
    centers = [(x, y) for x, y, d in holes if abs(d - CENTER_D) < 1e-6]
    pins = [(x, y) for x, y, d in holes if abs(d - 3.0) < 1e-6]
    out = []
    for cx, cy in centers:
        near = sorted(pins, key=lambda p: math.hypot(p[0] - cx, p[1] - cy))[:2]
        assert all(math.hypot(p[0] - cx, p[1] - cy) < 6.5 for p in near)
        # 2 つのピンのうち、中心に近い方が (-3.81, 2.54)
        p1, p2 = sorted(near, key=lambda p: math.hypot(p[0] - cx, p[1] - cy))
        (a1, b1), (a2, b2) = PIN_LOCAL
        ang = math.atan2(p2[1] - p1[1], p2[0] - p1[0]) - math.atan2(b2 - b1, a2 - a1)
        out.append((cx, cy, math.degrees(ang)))
    return out


def board_outline():
    return L.face({"edge": -MATRIX_INSET, "side": MATRIX_SIDE_GROW})


# 左右と斜めの辺で、基板の端とトップケースの内面の間に 0.3 mm 以上残ること
assert L.TOP_CAVITY["side"] - MATRIX_SIDE_GROW >= 0.3


def board_outline_segments():
    """基板の外形の辺の並び（プレートの座標）: [(始点, 円弧の中点 または None, 終点)]。KiCad の Edge.Cuts に入れる用
    （scripts/set_edge_cuts.py で読む）。"""
    out = []
    for e in board_outline().outer_wire().edges():
        a, b = ((e @ 0).X, (e @ 0).Y), ((e @ 1).X, (e @ 1).Y)
        m = None if e.geom_type == GeomType.LINE else ((e @ 0.5).X, (e @ 0.5).Y)
        out.append((a, m, b))
    return out


def socket(x, y, angle):
    """ホットスワップソケットの大まかな形（基板の裏 z = 0 から下へ）。ピンの穴は基板と同じく貫通させる。"""
    body = None
    for x0, y0, x1, y1, h in [(*r, SOCKET_H) for r in SOCKET_BODY] + [(*r, SOCKET_TAB_H) for r in SOCKET_TABS]:
        part = Pos((x0 + x1) / 2, (y0 + y1) / 2, -h) * extrude(Rectangle(x1 - x0, y1 - y0), h)
        body = part if body is None else body + part
    return Pos(x, y) * Rot(0, 0, angle) * body


def flip_holes(holes):
    """FLIP_AT のスイッチの穴を、スイッチの中心のまわりに 180° 回す。KiCad の基板で配置スクリプトを実行済みで、
    すでに 180° 回っているスイッチはそのままにする。"""
    def base_angle(x, y):   # スクリプトの KEYS のキーの回転（時計回り）を、反時計回りの角度にしたもの
        return next(-r for kx, ky, r in _S["KEYS"].values() if math.hypot(kx - x, -ky - y) < 0.01)

    done = [(x, y) for x, y, a in switches(holes) if abs((a - base_angle(x, y)) % 360 - 180) < 1]
    todo = [(cx, cy) for cx, cy in FLIP_AT if not any(math.hypot(cx - x, cy - y) < 0.01 for x, y in done)]
    out = []
    for x, y, d in holes:
        for cx, cy in todo:
            if math.hypot(x - cx, y - cy) < 8.0:
                x, y = 2 * cx - x, 2 * cy - y
        out.append((x, y, d))
    return out


def matrix_j1():
    """裏面の J1 の大まかな形（基板の裏 z = 0 から下へ）。プレビュー用（モックには付けない）。"""
    x0, y0, x1, y1 = J1_BODY
    return Pos(J1_AT[0] + (x0 + x1) / 2, J1_AT[1] + (y0 + y1) / 2, -J1_H) * extrude(Rectangle(x1 - x0, y1 - y0), J1_H)


def matrix_mock():
    """マトリクス基板のモック（プレートの座標、基板の上面が z = 0、裏面とソケットが下）。"""
    holes = flip_holes(kicad_holes())
    sw = switches(holes)
    # スイッチの中心が、プレートのスイッチ穴（plate.keys()）と一致すること
    keys = [(k.center.X, k.center.Y) for k in P.keys()]
    assert len(sw) == len(keys)
    for x, y, _ in sw:
        assert min(math.hypot(x - a, y - b) for a, b in keys) < 0.01, (x, y)
    board = Pos(0, 0, -MATRIX_T) * extrude(board_outline(), MATRIX_T)
    for x, y, angle in sw:
        board += Pos(0, 0, -MATRIX_T) * socket(x, y, angle)
    for x, y, d in holes:
        dp = CENTER_PRINT_D if abs(d - CENTER_D) < 1e-6 else d + HOLE_ALLOWANCE
        board -= Pos(x, y, -10) * Solid.make_cylinder(dp / 2, 20)
    return board, sw


if __name__ == "__main__":
    import bay   # bay は天井の開口の位置をこのモジュール（J1 の位置）から読むので、ここで読み込む
    board, sw = matrix_mock()
    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    # 印刷の向き: 基板の上面を下にする（x 軸まわりに 180° 回す）
    bay.export([("matrix-mock-l", Rot(180, 0, 0) * board)], out / "matrix-mock-l")
    (out / "matrix-board-l-outline.json").write_text(json.dumps(board_outline_segments()))
    print(f"switches = {len(sw)}, rotated = {[round(a, 1) for *_, a in sw if abs(a) > 0.1]}")
    print(f"board z (bottom case) = {PCB_TOP_Z - MATRIX_T:.2f}..{PCB_TOP_Z:.2f}, socket bottom = {PCB_TOP_Z - MATRIX_T - SOCKET_H:.2f}")
    print(f"volume = {board.volume:.2f} mm^3, solids = {len(board.solids())}")
    if "--show" in sys.argv:
        from ocp_vscode import show
        case, lid = B.build(), bay.lid()
        cboard, xiao, usb, batt = bay.dummies()
        show(case, lid, cboard, xiao, usb, batt, Pos(0, 0, PCB_TOP_Z) * board,
             names=["bottom-l", "lid", "board", "xiao", "usb-c", "battery", "matrix-mock"],
             colors=["#5b8fd6", "#7f77dd", "#3b6d11", "#1d9e75", "#ba7517", "#888780", "#2f7d4f"],
             alphas=[0.4, 1, 1, 1, 1, 1, 0.9])
