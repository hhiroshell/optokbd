# KiCad PCB エディタの「ツール → スクリプトコンソール」で、マトリクス基板（左手か右手）を開いて実行する。
#   exec(open('/home/hhiroshell/src/github.com/hhiroshell/optokbd/scripts/add_matrix_silk.py').read())
# 裏面のシルク（B.Silkscreen）に、基板の名前と左右の別、版を入れる。左手か右手かは、開いている基板のファイル名で決める
# （BOARDS）。置き場所は R2 の C0・C1 のキーの手前（部品も配線もない所）。右手は左右を反転した位置。
# もう一度実行すると、前に入れた文字（NAME で始まる B.Silkscreen のテキスト）を消して入れ直す。

import os
import pcbnew

# ===== 設定 =====
# 基板のファイル名（拡張子なし）: (左右の文字, プレートの座標の原点を置く KiCad の位置 [mm], x を反転するか)。
# 原点と反転は place_matrix.py の BOARDS と同じ
BOARDS = {
    "matrix-left": ("L", (50.0, 50.0), False),
    "matrix-right": ("R", (185.0, 50.0), True),
}
NAME = "optokbd matrix"
TEXT = NAME + " {side}\nv0.2.0 2026-10"
AT = (20.0, -67.0)  # 文字の中心: 左手のプレートの座標（mm、y は奥が正）
SIZE = 1.0          # 文字の高さと幅
THICKNESS = 0.15    # 線の太さ
# ================

board = pcbnew.GetBoard()
name = os.path.splitext(os.path.basename(board.GetFileName()))[0]
if name not in BOARDS:
    raise ValueError(f"{name}: シルクの設定がない（BOARDS に追加する）")
side, origin, mirror = BOARDS[name]

old = [d for d in board.GetDrawings()
       if isinstance(d, pcbnew.PCB_TEXT) and d.GetLayer() == pcbnew.B_SilkS and d.GetText().startswith(NAME)]
for d in old:
    board.Remove(d)

x, y = AT
if mirror:
    x = -x
t = pcbnew.PCB_TEXT(board)
t.SetText(TEXT.format(side=side))
t.SetLayer(pcbnew.B_SilkS)
t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(SIZE), pcbnew.FromMM(SIZE)))
t.SetTextThickness(pcbnew.FromMM(THICKNESS))
t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_CENTER)
t.SetVertJustify(pcbnew.GR_TEXT_V_ALIGN_CENTER)
t.SetMirrored(True)   # 裏面の文字は、裏から見て読めるように左右反転する
t.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(origin[0] + x), pcbnew.FromMM(origin[1] - y)))
board.Add(t)

pcbnew.Refresh()
print(f"裏面のシルク: {len(old)} 個を消して「{TEXT.format(side=side)}」を入れた")
