# KiCad PCB エディタの「ツール → スクリプトコンソール」に貼り付けて実行する。
# スイッチ・ダイオードを、接続されているネット（R0〜R3, C0〜C6）から判別して配置する。
# 参照番号（SW1, D1 ...）には依存しない。

import math
import re
import pcbnew

# ===== 設定 =====
MIRROR = False             # matrix-right では True にする（x方向を反転）
ORIGIN = (50.0, 50.0)      # レイアウト座標 (0,0) を置く基板上の位置 [mm]
DIODE_OFFSET = (0.0, 5.5)  # スイッチ中心から見たダイオードの位置（キーの回転に追従）[mm]
DIODE_ANGLE = -90.0         # キーに対するダイオードの角度 [deg]。カソードが上を向いたら -90.0 に
DIODE_ON_BACK = True       # ダイオードを裏面に置く

# 上下を反転（180°回転）するキー。ソケットがキーの下半分に移る。
FLIP_KEYS = {(1, 3)}
# キーごとにダイオードの位置を変える場合: {(行, 列): (dx, dy, 角度)}（キー基準の座標）
DIODE_OVERRIDE = {
    (0, 3): (-5.5, 4.2, 0.0),  # J1 を避けて、キー左下に横向きで置く
    (1, 3): (-5.5, 4.2, 0.0),  # 反転キーなので、基板上ではキー右上に来る
}
# J1（11ピン）を置く位置: キー (0,3) の中心からキー基準で (0, 9.525)
# ＝ R0C3 と R1C3 の境目。裏面に置き、ケーブルを下（コントローラー側）へ出す。
J1_KEY = (0, 3)
J1_OFFSET = (0.0, 9.525)
J1_ANGLE = 0.0
# ================

# KLE から計算したキー中心 [mm] と回転 [deg, 画面上で時計回り]。左手基準。
KEYS = {
    (0, 0): (9.53, 17.14, 0.0),  (0, 1): (28.58, 17.14, 0.0),
    (0, 2): (47.62, 11.43, 0.0), (0, 3): (66.67, 9.53, 0.0),
    (0, 4): (85.73, 11.43, 0.0), (0, 5): (104.78, 13.88, 0.0),
    (0, 6): (123.83, 17.68, 0.0),
    (1, 0): (9.53, 36.20, 0.0),  (1, 1): (28.58, 36.20, 0.0),
    (1, 2): (47.62, 30.48, 0.0), (1, 3): (66.67, 28.58, 0.0),
    (1, 4): (85.73, 30.48, 0.0), (1, 5): (104.78, 32.93, 0.0),
    (1, 6): (123.83, 36.73, 0.0),
    (2, 0): (9.53, 55.24, 0.0),  (2, 1): (28.58, 55.24, 0.0),
    (2, 2): (47.62, 49.53, 0.0), (2, 3): (66.67, 47.62, 0.0),
    (2, 4): (85.73, 49.53, 0.0), (2, 5): (104.78, 51.98, 0.0),
    (2, 6): (123.83, 55.78, 0.0),
    # 親指: 5.5°ずつ回転し、隣のキーと下側の角を共有する円弧。R3C3 の上辺が R2C2 の下辺に接する。
    # C5/C6 列は、R2 が親指キーにちょうど接するよう KLE からそれぞれ 0.41mm / 0.61mm 上げている。
    (3, 3): (61.72, 68.58, 0.0),   (3, 4): (81.64, 69.54, 5.5),
    (3, 5): (101.38, 72.40, 11.0), (3, 6): (120.75, 77.14, 16.5),
}

ROW_RE = re.compile(r"^/?R(\d)$")
COL_RE = re.compile(r"^/?C(\d)$")


def mm(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))


def flip_to_back(fp):
    if fp.IsFlipped():
        return
    try:  # KiCad 9
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    except (AttributeError, TypeError):  # KiCad 8 以前
        fp.Flip(fp.GetPosition(), False)


def place(fp, x, y, r_cw):
    # KiCad の角度は反時計回りが正なので符号を反転する
    fp.SetPosition(mm(x, y))
    fp.SetOrientationDegrees(-r_cw)


def key_pose(row, col):
    kx, ky, r = KEYS[(row, col)]
    if MIRROR:
        kx, r = -kx, -r
    return ORIGIN[0] + kx, ORIGIN[1] + ky, r


def local_to_board(dx, dy, r_cw):
    # キー基準のオフセットを、キーの回転に合わせて基板座標に変換する
    if MIRROR:
        dx = -dx
    a = math.radians(r_cw)
    return dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)


board = pcbnew.GetBoard()
fps = list(board.GetFootprints())
diodes = [f for f in fps if f.GetReference().startswith("D")]

placed, problems = 0, []
for sw in fps:
    ref = sw.GetReference()
    if not ref.startswith("SW"):
        continue
    nets = [p.GetNetname() for p in sw.Pads()]
    col = next((int(COL_RE.match(n).group(1)) for n in nets if COL_RE.match(n)), None)
    mid = next((n for n in nets if n and not COL_RE.match(n)), None)
    if col is None or mid is None:
        problems.append(f"{ref}: 列ネットか中間ネットが見つからない {nets}")
        continue

    dio = next((d for d in diodes if mid in [p.GetNetname() for p in d.Pads()]), None)
    if dio is None:
        problems.append(f"{ref}: 対応するダイオードが見つからない (net {mid})")
        continue
    dnets = [p.GetNetname() for p in dio.Pads()]
    row = next((int(ROW_RE.match(n).group(1)) for n in dnets if ROW_RE.match(n)), None)
    if row is None or (row, col) not in KEYS:
        problems.append(f"{ref}/{dio.GetReference()}: R{row}C{col} はレイアウトにない")
        continue

    x, y, r = key_pose(row, col)
    if (row, col) in FLIP_KEYS:
        r += 180.0
    place(sw, x, y, r)

    dx, dy, dang = DIODE_OVERRIDE.get((row, col), (*DIODE_OFFSET, DIODE_ANGLE))
    ox, oy = local_to_board(dx, dy, r)
    if DIODE_ON_BACK:
        flip_to_back(dio)
    place(dio, x + ox, y + oy, r + dang)
    placed += 1
    print(f"R{row}C{col}: {ref} + {dio.GetReference()}")

# J1（11ピン）の配置。11ピンのコネクタを参照番号の J から探す。
j1 = next((f for f in fps if f.GetReference().startswith("J") and len(list(f.Pads())) >= 11), None)
if j1 is None:
    problems.append("11ピンのコネクタ（J）が見つからない")
else:
    x, y, r = key_pose(*J1_KEY)
    ox, oy = local_to_board(*J1_OFFSET, r)
    flip_to_back(j1)
    place(j1, x + ox, y + oy, r + J1_ANGLE)
    print(f"J1: {j1.GetReference()} を R{J1_KEY[0]}C{J1_KEY[1]} の下端に配置")

pcbnew.Refresh()
print(f"配置したキー: {placed} / 25")
for p in problems:
    print("要確認:", p)
