# 3D プリントで組み付けを試すための仮のコントローラー基板（左右共通）と、XIAO と USB-C のモック。
# - 外形は bay.py と同じ（角の丸めつき。KiCad の Edge.Cuts と同じ形）。穴は KiCad の基板
#   （pcbs/controller/controller.kicad_pcb）のドリルデータから取るので、部品の配置を変えたら書き出し直す
# - 印刷すると穴は細るので PRINT_ALLOWANCE だけ広げ、細いピンの穴は PRINT_MIN_D 以上にする
# - 座標は基板の面の座標（mm、原点は奥の辺の直線部分の中央、y は奥が正）。KiCad の補助原点も同じ点に置いてある
# - 実行: uv run python controller_test_board.py [--show]   （kicad-cli が必要。out/controller-test-board.* を書き出す）
import re
import subprocess
import sys
import tempfile
from pathlib import Path

from build123d import Pos, Solid, extrude

import bay

KICAD_PCB = Path(__file__).resolve().parent.parent / "pcbs" / "controller" / "controller.kicad_pcb"
PRINT_ALLOWANCE = 0.3         # 穴の直径に足す量
PRINT_MIN_D = 1.2             # 印刷する穴の最小の直径（JST-PH のピン 0.64 角の対角 0.9 が通る大きさ）


def kicad_holes(pcb=KICAD_PCB):
    """KiCad の基板の穴: [(x, y, 直径)]。ドリルデータ（Excellon、補助原点基準、mm）を書き出して読む。"""
    with tempfile.TemporaryDirectory() as d:
        subprocess.run(["kicad-cli", "pcb", "export", "drill", "--format", "excellon", "--drill-origin", "plot",
                        "--excellon-units", "mm", "--excellon-zeros-format", "decimal", "-o", d + "/", str(pcb)],
                       check=True, capture_output=True)
        text = "".join(p.read_text() for p in Path(d).glob("*.drl"))
    tools = {t: float(dia) for t, dia in re.findall(r"^T(\d+)C([\d.]+)", text, re.M)}
    holes, tool = [], None
    for line in text.split("%", 1)[1].splitlines():
        if m := re.fullmatch(r"T(\d+)", line):
            tool = m.group(1)
        elif m := re.fullmatch(r"X(-?[\d.]+)Y(-?[\d.]+)", line):
            holes.append((float(m.group(1)), float(m.group(2)), tools[tool]))
        elif line.startswith(("G85", "G00", "M15")):
            raise ValueError("長穴（ルート加工）には対応していない: " + line)
    return holes


def test_board():
    """仮のコントローラー基板: 外形を基板の厚さで押し出し、KiCad の穴を印刷用に広げて開ける。下面が z = 0。"""
    holes = kicad_holes()
    # 取り付け穴が、ケースの底蓋の台座（bay.py）と同じ位置にあること
    mounts = sorted((x, y) for x, y, d in holes if abs(d - bay.BOARD_HOLE_D) < 1e-6)
    assert all(abs(a - c) < 1e-3 and abs(b - e) < 1e-3
               for (a, b), (c, e) in zip(mounts, sorted(bay.board_holes_board()))), mounts
    # 電池のコネクタ（J2、J3）の足が、底蓋の凹み（bay.BATTERY_PINS）の位置にあること
    assert all(any(abs(x - px) < 1e-3 and abs(y - py) < 1e-3 for x, y, _ in holes)
               for pair in bay.BATTERY_PINS for px, py in pair)
    board = extrude(bay.board_face_local(), bay.BOARD_T, dir=(0, 0, 1))
    for x, y, d in holes:
        dp = max(d + PRINT_ALLOWANCE, PRINT_MIN_D)
        board -= Pos(x, y, -1) * Solid.make_cylinder(dp / 2, bay.BOARD_T + 2)
    return board, holes


if __name__ == "__main__":
    board, holes = test_board()
    xiao = bay.xiao_mock()
    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    bay.export([("controller-test-board", board), ("xiao-mock", Pos(0, 25, 0) * xiao)], out / "controller-test-board")
    for x, y, d in holes:
        print(f"hole ({x:7.3f}, {y:7.3f})  KiCad Ø{d:.2f} -> print Ø{max(d + PRINT_ALLOWANCE, PRINT_MIN_D):.2f}")
    print(f"board volume = {board.volume:.2f} mm^3, solids = {len(board.solids())}")
    if "--show" in sys.argv:
        from ocp_vscode import show
        show(board, Pos(0, 25, 0) * xiao, names=["controller-test-board", "xiao-mock"], colors=["#3b6d11", "#1d9e75"])
