# KiCad PCB エディタの「ツール → スクリプトコンソール」で、基板（コントローラーかマトリクス）を開いて実行する。
# 基板の Edge.Cuts を消して、case/ のスクリプトが書き出した外形（case/out/*.json）で描き直す。どの外形を使うかは、
# 開いている基板のファイル名で決める（BOARDS）。先に case/ で対応するスクリプトを実行しておく。
#   exec(open('/home/hhiroshell/src/github.com/hhiroshell/optokbd/scripts/set_edge_cuts.py').read())
# 外形は [(始点, 円弧の中点 または None, 終点)] の並び（mm、y は奥が正）。その座標の原点を KiCad の ORIGIN に置く
# （KiCad の y は下が正なので符号を反転する）。

import json
import os
import pcbnew

# ===== 設定 =====
# 基板のファイル名（拡張子なし）: (外形の JSON、外形の座標の原点を置く KiCad の位置 [mm]、書き出すスクリプト)
BOARDS = {
    "controller": ("controller-board-outline.json", (100.0, 90.0), "bay.py"),       # 原点は奥の辺の直線部分の中央
    "matrix-left": ("matrix-board-l-outline.json", (50.0, 50.0), "matrix_mock.py"),  # 原点はプレートの座標の原点
}
WIDTH = 0.05               # 線の幅 [mm]
# ================

board = pcbnew.GetBoard()
board_file = os.path.abspath(board.GetFileName())
name = os.path.splitext(os.path.basename(board_file))[0]
if name not in BOARDS:
    raise ValueError(f"{name}: 外形の設定がない（BOARDS に追加する）")
json_name, origin, maker = BOARDS[name]
outline_json = os.path.normpath(os.path.join(os.path.dirname(board_file), "..", "..", "case", "out", json_name))
if not os.path.exists(outline_json):
    raise FileNotFoundError(f"{outline_json} がない（case/ で uv run python {maker} を実行する）")


def P(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(origin[0] + x), pcbnew.FromMM(origin[1] - y))


segments = json.load(open(outline_json))

old = [d for d in board.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts]
for d in old:
    board.Remove(d)

for start, mid, end in segments:
    s = pcbnew.PCB_SHAPE(board)
    if mid is None:
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(P(*start))
        s.SetEnd(P(*end))
    else:
        s.SetShape(pcbnew.SHAPE_T_ARC)
        s.SetArcGeometry(P(*start), P(*mid), P(*end))
    s.SetLayer(pcbnew.Edge_Cuts)
    s.SetWidth(pcbnew.FromMM(WIDTH))
    board.Add(s)

pcbnew.Refresh()
print(f"Edge.Cuts: {len(old)} 本を消して {len(segments)} 本を描いた（{outline_json}）")
