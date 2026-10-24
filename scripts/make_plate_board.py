# スイッチプレートを JLCPCB に FR-4 の基板（1.6 mm）として出すための KiCad の基板（pcbs/plate）を作る。KiCad の外で、
# KiCad の Python（/usr/bin/python3 の pcbnew）で実行する。基板のファイルは作り直す（手で編集しない）。
#   /usr/bin/python3 scripts/make_plate_board.py
# 先に case/ で uv run python plate.py を実行して、外形とスイッチ穴（case/out/plate-outline.json）を書き出しておく。
# - 左右共通: 左手の形で作り、右手には裏返して使う（銅も部品もないので、裏返すと x を反転した右手の形になる）
# - Edge.Cuts: プレートの外形（ガスケットのタブ込み）とスイッチ穴（14 mm 角）
# - 置く位置は左手のマトリクス基板と同じ（プレートの座標の原点を ORIGIN に。place_matrix.py の左手と同じ）
# - 裏面のシルクに、名前と版と、JLCPCB の注文番号を印刷する場所の指定（JLCJLCJLCJLC。注文の画面で「Specify a location」
#   を選ぶ）を入れる。右手では裏面が上を向くので、どちらの向きでもトップケースに隠れる、ガスケットのタブの根元に置く
# - 銅はない。ガーバーとドリルは scripts/export_gerbers.py で書き出す

import json
from pathlib import Path

import pcbnew

# ===== 設定 =====
ROOT = Path(__file__).resolve().parent.parent
NAME = "plate"
OUTLINE_JSON = ROOT / "case/out/plate-outline.json"
ORIGIN = (50.0, 50.0)    # プレートの座標の原点を置く KiCad の位置 [mm]
THICKNESS = 1.6          # 板厚 [mm]（ケースの plate.PLATE_T と同じ）
EDGE_WIDTH = 0.05        # Edge.Cuts の線の太さ
# 裏面のシルク: (文字, プレートの座標の中心 [mm], 文字の高さ)。どちらも奥のタブの根元の上（タブの奥の辺は斜めなので、
# 低い側の端でも縁から 0.5 mm 以上離す）。R0 のキーの開口（トップケースの開口。y -3.6 より手前）にはかからない
TEXTS = [
    ("optokbd plate\nv0.2.0 2026-10", (24.0, -2.6), 1.0),     # 奥の左のタブ（x 9.53..35.58。右ほど高い）
    ("JLCJLCJLCJLC", (104.5, -1.5), 1.0),                      # 奥の右のタブ（x 97.78..123.83。左ほど高い）。注文番号の位置
]
TEXT_THICKNESS = 0.15
# ================


def at(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(ORIGIN[0] + x), pcbnew.FromMM(ORIGIN[1] - y))


segments = json.loads(OUTLINE_JSON.read_text())
board = pcbnew.BOARD()
board.SetCopperLayerCount(2)
board.GetDesignSettings().SetBoardThickness(pcbnew.FromMM(THICKNESS))

for start, mid, end in segments:
    s = pcbnew.PCB_SHAPE(board)
    if mid is None:
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(at(*start))
        s.SetEnd(at(*end))
    else:
        s.SetShape(pcbnew.SHAPE_T_ARC)
        s.SetArcGeometry(at(*start), at(*mid), at(*end))
    s.SetLayer(pcbnew.Edge_Cuts)
    s.SetWidth(pcbnew.FromMM(EDGE_WIDTH))
    board.Add(s)

for text, (x, y), size in TEXTS:
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text)
    t.SetLayer(pcbnew.B_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(size), pcbnew.FromMM(size)))
    t.SetTextThickness(pcbnew.FromMM(TEXT_THICKNESS))
    t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_CENTER)
    t.SetVertJustify(pcbnew.GR_TEXT_V_ALIGN_CENTER)
    t.SetMirrored(True)   # 裏面の文字は、裏から見て読めるように左右反転する
    t.SetPosition(at(x, y))
    board.Add(t)

path = ROOT / "pcbs" / NAME / f"{NAME}.kicad_pcb"
path.parent.mkdir(parents=True, exist_ok=True)
board.Save(str(path))
print(f"{path.relative_to(ROOT)}: Edge.Cuts {len(segments)} 本、裏面のシルク {len(TEXTS)} 個、板厚 {THICKNESS} mm")
