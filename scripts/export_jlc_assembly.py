# JLCPCB の部品実装（PCBA）に出す BOM と CPL（部品の座標）を、基板のファイルから書き出す。KiCad の外で、KiCad の Python
# （/usr/bin/python3 の pcbnew）で実行する。基板のファイルは読むだけ。
#   /usr/bin/python3 scripts/export_jlc_assembly.py               # 左右のマトリクス基板
#   /usr/bin/python3 scripts/export_jlc_assembly.py a.kicad_pcb   # 指定した基板
# 出力: 基板のフォルダの jlcpcb/<基板>-bom.csv と jlcpcb/<基板>-cpl.csv（jlcpcb/ は .gitignore 済み）
# - 対象は LCSC のフィールド（scripts/add_lcsc_fields.py で入れる）がある部品。BOM から外す・実装しない（DNP）設定の部品は除く
# - BOM: Comment, Designator（カンマ区切り）, Footprint, LCSC Part #。LCSC の番号ごとにまとめる
# - CPL: Designator, Mid X, Mid Y, Rotation, Layer。座標は KiCad の絶対座標（mm、Y は上が正）。ガーバーも絶対座標で
#   書き出す（「補助原点を使用」をオフ）
# - 実装する面は、フットプリントの面ではなく銅のパッドの面で決める。スイッチのフットプリント（表）はホットスワップ
#   ソケットのパッドが裏にあるので、ソケットは Bottom。このような部品の中心は、フットプリントの原点（スイッチの中心）
#   ではなく、パッドの範囲の中心にする
# - 回転: 表の部品は KiCad の向き（反時計回り）、裏の部品は 180 - KiCad の向き。どちらにも ROTATION_OFFSET を足す。
#   JLCPCB のプレビューは、裏の部品を「部品のフットプリントを回転してから左右を反転したもの」として表から見せる。
#   EasyEDA の 3 つの部品のフットプリント（パッドと穴）をこの決め方で置くと、左右の基板のすべての部品のパッドと穴が
#   0.1 mm 以内で重なり、パッドの番号（ダイオードのカソード、J1 のピン 1）も合う（2026-10-24 に確かめた）。KiCad の
#   向きのままでは、ソケットと J1 が 180° ずれていた。注文画面のプレビューで向きを確かめ、ずれていたら ROTATION_OFFSET を直す

import csv
import re
import sys
from collections import defaultdict
from pathlib import Path

import pcbnew

# ===== 設定 =====
ROOT = Path(__file__).resolve().parent.parent
DEFAULT_BOARDS = [
    ROOT / "pcbs/matrix-left/matrix-left.kicad_pcb",
    ROOT / "pcbs/matrix-right/matrix-right.kicad_pcb",
]
FIELD = "LCSC"
# BOM の Comment（部品の名前）。ない番号は回路図の Value
COMMENT = {
    "C41430893": "MX hotswap socket CPG151101S11-16",
    "C81598": "1N4148W",
    "C515956": "SM11B-SRSS-TB",
}
# 回転の補正（度、LCSC の番号ごと）。注文画面のプレビューで向きがずれていたら直す
ROTATION_OFFSET = {
    "C41430893": 0.0,
    "C81598": 0.0,
    "C515956": 0.0,
}
# ================


def natural_key(ref):
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", ref)]


def pad_side(fp):
    """銅のパッド（穴だけのものを除く）がある面: "Top"、"Bottom"、両面なら None。"""
    front = back = False
    for p in fp.Pads():
        if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
            continue
        front |= p.IsOnLayer(pcbnew.F_Cu)
        back |= p.IsOnLayer(pcbnew.B_Cu)
    if front and not back:
        return "Top"
    if back and not front:
        return "Bottom"
    return None


def centroid(fp, side):
    """部品の中心（KiCad の座標、mm）。パッドの面がフットプリントの面と違う部品は、パッドの範囲の中心。"""
    fp_side = "Bottom" if fp.IsFlipped() else "Top"
    if side == fp_side:
        pos = fp.GetPosition()
        return pcbnew.ToMM(pos.x), pcbnew.ToMM(pos.y)
    box = None
    for p in fp.Pads():
        if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
            continue
        b = p.GetBoundingBox()
        if box is None:
            box = b
        else:
            box.Merge(b)
    c = box.GetCenter()
    return pcbnew.ToMM(c.x), pcbnew.ToMM(c.y)


def export(path):
    board = pcbnew.LoadBoard(str(path))
    groups = defaultdict(list)   # (LCSC, Comment, Footprint): [Designator]
    rows, problems = [], []
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        if not fp.HasField(FIELD) or not fp.GetFieldText(FIELD).strip():
            continue
        if fp.IsExcludedFromBOM() or fp.IsDNP():
            continue
        lcsc = fp.GetFieldText(FIELD).strip()
        side = pad_side(fp)
        if side is None:
            problems.append(f"{ref}: パッドが両面にある（スルーホール部品は対象外）")
            continue
        x, y = centroid(fp, side)
        rot = fp.GetOrientationDegrees() if side == "Top" else 180 - fp.GetOrientationDegrees()
        rot = (rot + ROTATION_OFFSET.get(lcsc, 0.0)) % 360
        footprint = fp.GetFPID().GetLibItemName().wx_str()
        groups[(lcsc, COMMENT.get(lcsc, fp.GetValue()), footprint)].append(ref)
        rows.append((ref, f"{x:.4f}mm", f"{-y:.4f}mm", f"{rot:.2f}", side))

    out = path.parent / "jlcpcb"
    out.mkdir(exist_ok=True)
    name = path.stem
    with open(out / f"{name}-bom.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
        for (lcsc, comment, footprint), refs in sorted(groups.items(), key=lambda kv: natural_key(min(kv[1], key=natural_key))):
            w.writerow([comment, ",".join(sorted(refs, key=natural_key)), footprint, lcsc])
    with open(out / f"{name}-cpl.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Mid X", "Mid Y", "Rotation", "Layer"])
        for row in sorted(rows, key=lambda r: natural_key(r[0])):
            w.writerow(row)
    counts = ", ".join(f"{lcsc} × {len(refs)}" for (lcsc, _, _), refs in sorted(groups.items()))
    print(f"{name}: 部品 {len(rows)} 個（{counts}）→ {out.relative_to(ROOT) if out.is_relative_to(ROOT) else out}/")
    for p in problems:
        print("  要確認:", p)


for p in [Path(a).resolve() for a in sys.argv[1:]] or DEFAULT_BOARDS:
    export(p)
