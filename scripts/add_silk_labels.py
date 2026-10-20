# KiCad PCB エディタの「ツール → スクリプトコンソール」で、コントローラー基板（pcbs/controller/controller.kicad_pcb）を開いて
# 実行する。裏面のシルク（B.Silkscreen）に、電池のコネクタの極性と左右の別（J2 は左手、J3 は右手）、基板の名前と版を入れる。
# もう一度実行すると、前に入れた文字（LABELS と同じ文字の B.Silkscreen のテキスト）を消して入れ直す。
#   exec(open('/home/hhiroshell/src/github.com/hhiroshell/optokbd/scripts/add_silk_labels.py').read())
# 座標は KiCad の絶対座標（mm）。基板の原点（奥の辺の直線部分の中央）は (100, 90)

import pcbnew

# ===== 設定 =====
SIZE = 1.0          # 文字の高さと幅
THICKNESS = 0.15    # 線の太さ
NAME_SIZE = 0.8     # 基板の名前と版の文字（H3 と H4 の台座の間に収める）

# (文字, x, y, 大きさ)。極性はパッド（y 101.495）の中心から 1.8 mm 手前
LABELS = [
    ("-", 84.495, 103.3, SIZE),       # J2-1 GND
    ("+", 86.495, 103.3, SIZE),       # J2-2 VBAT
    ("BAT L", 85.495, 104.8, SIZE),   # J2 は左手で使う
    ("-", 113.505, 103.3, SIZE),      # J3-1 GND
    ("+", 115.505, 103.3, SIZE),      # J3-2 VBAT
    ("BAT R", 114.505, 104.8, SIZE),  # J3 は右手で使う
    ("optokbd controller\nv0.2.0 2026-10", 100.0, 116.5, NAME_SIZE),
]
# ================

board = pcbnew.GetBoard()
texts = {text for text, *_ in LABELS}
old = [d for d in board.GetDrawings()
       if isinstance(d, pcbnew.PCB_TEXT) and d.GetLayer() == pcbnew.B_SilkS and d.GetText() in texts]
for d in old:
    board.Remove(d)

for text, x, y, size in LABELS:
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text)
    t.SetLayer(pcbnew.B_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(size), pcbnew.FromMM(size)))
    t.SetTextThickness(pcbnew.FromMM(THICKNESS))
    t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_CENTER)
    t.SetVertJustify(pcbnew.GR_TEXT_V_ALIGN_CENTER)
    t.SetMirrored(True)   # 裏面の文字は、裏から見て読めるように左右反転する
    t.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
    board.Add(t)

pcbnew.Refresh()
print(f"裏面のシルク: {len(old)} 個を消して {len(LABELS)} 個を入れた")
