# KiCad PCB エディタの「ツール → スクリプトコンソール」で、マトリクス基板（左手か右手）を開いて実行する。
#   exec(open('/home/hhiroshell/src/github.com/hhiroshell/optokbd/scripts/place_socket_vias.py').read())
# スイッチのソケットの列のパッド（パッド 1、ネット C0〜C6）の横に、表の列の線へ乗り換えるビアを置き、パッドとビアを
# 裏の短い線でつなぐ。ビアは、スイッチの左右の固定用の穴（Ø2）を結ぶ線（スイッチの中心を通る横の線）の上で、パッド 1 の
# 真上か真下に置く。位置はソケットのフットプリントの実際の位置と向きから求めるので、反転したキー・親指のキー・右手でも、
# どのキーでも同じ相対位置にそろう。パッド 1 は形が基準点から 0.45 ずらして定義されているので、基準点ではなく形の中心を使う。
# 何度実行してもよい（パッド 1 から OLD_VIA_R 以内にある同じネットのビアと、パッド 1 とそのビアを結ぶ裏の線は、消してから
# 置き直す）。place_matrix.py でスイッチを配置し直したら、これも実行する。

import math
import re
import pcbnew

# ===== 設定 =====
VIA_SIZE = (0.6, 0.3)      # ビアの直径と穴の直径 [mm]（ネットクラス Default と同じ）
STUB_WIDTH = 0.25          # パッドとビアをつなぐ線の幅 [mm]
PEG_D = 2.0                # スイッチの左右の固定用の穴の直径 [mm]
OLD_VIA_R = 4.0            # 置き直すときに消すビアの範囲: パッド 1 の中心から [mm]
# ================

COL_RE = re.compile(r"^/?C(\d)$")


def mm(v):
    return pcbnew.ToMM(v.x), pcbnew.ToMM(v.y)


def at(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))


def near(v, x, y, tol=0.01):
    vx, vy = mm(v)
    return math.hypot(vx - x, vy - y) < tol


board = pcbnew.GetBoard()
targets, problems = [], []   # [(パッド 1, パッド 1 の中心, ビアの位置)]
for sw in list(board.GetFootprints()):
    ref = sw.GetReference()
    if not ref.startswith("SW"):
        continue
    pads = list(sw.Pads())
    pad1 = next((p for p in pads if COL_RE.match(p.GetNetname())), None)
    pegs = [p for p in pads if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH
            and abs(pcbnew.ToMM(p.GetDrillSize().x) - PEG_D) < 0.01]
    if pad1 is None or len(pegs) != 2:
        problems.append(f"{ref}: 列のパッドか固定用の穴が見つからない")
        continue
    # 固定用の穴を結ぶ線に、パッド 1 の中心から垂線を下ろした点
    (ax, ay), (bx, by) = mm(pegs[0].GetPosition()), mm(pegs[1].GetPosition())
    px, py = mm(pad1.ShapePos(pcbnew.B_Cu))
    ux, uy = bx - ax, by - ay
    t = ((px - ax) * ux + (py - ay) * uy) / (ux * ux + uy * uy)
    targets.append((pad1, (px, py), (ax + t * ux, ay + t * uy)))

# 前に置いたビアと線（パッド 1 の近くにある同じネットのビアと、パッド 1 とそのビアを結ぶ裏の線）を集めてから消す
tracks = list(board.GetTracks())
old = []
for pad1, (px, py), _ in targets:
    net = pad1.GetNetCode()
    vias = [tr for tr in tracks if tr.Type() == pcbnew.PCB_VIA_T and tr.GetNetCode() == net
            and math.hypot(mm(tr.GetPosition())[0] - px, mm(tr.GetPosition())[1] - py) < OLD_VIA_R]
    for tr in tracks:
        if tr.Type() == pcbnew.PCB_VIA_T or tr.GetNetCode() != net or tr.GetLayer() != pcbnew.B_Cu:
            continue
        s0, s1 = tr.GetStart(), tr.GetEnd()
        for e, f in ((s0, s1), (s1, s0)):
            if pad1.HitTest(e) and any(near(f, *mm(v.GetPosition())) for v in vias):
                old.append(tr)
                break
    old += vias
for tr in old:
    board.Remove(tr)

for pad1, (px, py), (vx, vy) in targets:
    net = pad1.GetNetCode()
    via = pcbnew.PCB_VIA(board)
    via.SetViaType(pcbnew.VIATYPE_THROUGH)
    via.SetPosition(at(vx, vy))
    via.SetWidth(pcbnew.FromMM(VIA_SIZE[0]))
    via.SetDrill(pcbnew.FromMM(VIA_SIZE[1]))
    via.SetNetCode(net)
    board.Add(via)

    stub = pcbnew.PCB_TRACK(board)
    stub.SetStart(at(px, py))
    stub.SetEnd(at(vx, vy))
    stub.SetWidth(pcbnew.FromMM(STUB_WIDTH))
    stub.SetLayer(pcbnew.B_Cu)
    stub.SetNetCode(net)
    board.Add(stub)

pcbnew.Refresh()
print(f"ビアを置いたスイッチ: {len(targets)} / 25（消した古いビアと線: {len(old)}）")
for p in problems:
    print("要確認:", p)
