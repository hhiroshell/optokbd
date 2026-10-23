# JLCPCB に出すガーバーとドリルを書き出して、発注用の zip にまとめる。KiCad の外で、KiCad の Python
# （/usr/bin/python3 の pcbnew）で実行する。基板のファイルは読むだけ（ゾーンの塗り直しは書き出しの中だけで行う）。
#   /usr/bin/python3 scripts/export_gerbers.py                # 3 枚（コントローラー、左右のマトリクス）
#   /usr/bin/python3 scripts/export_gerbers.py a.kicad_pcb    # 指定した基板
#   /usr/bin/python3 scripts/export_gerbers.py --force ...    # 確認で問題があっても書き出す
# 出力: 基板のフォルダの jlcpcb/gerbers/（書き出すたびに作り直す）と jlcpcb/<基板>-gerbers.zip（jlcpcb/ は .gitignore 済み）
# - 基板の一時的なコピーで、ゾーン（ティアドロップを含む）を必ず塗り直してから、確かめて書き出す（ファイルに保存された
#   塗りが古いと、ティアドロップが別のネットのパッドにほぼ接していた。KiCad の DRC はそれを見逃した。塗り直すと
#   クリアランスを守った形になる）。確かめた形とガーバーの形は同じ
# - 確かめること: DRC のエラー（error のもの）と、ティアドロップと別のネットの銅の距離
# - ガーバー: JLCPCB の KiCad 向けの推奨に合わせる（Protel の拡張子、X2 なし、ネットリストの属性なし、シルクから
#   レジストの開口を除く）。座標は絶対座標（CPL と同じ。補助原点は使わない）
# - ドリル: Excellon、mm、小数点の形式、長穴は alternate、PTH と NPTH を別のファイルに、ドリルマップは Gerber X2

import math
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

import pcbnew

# ===== 設定 =====
ROOT = Path(__file__).resolve().parent.parent
DEFAULT_BOARDS = [
    ROOT / "pcbs/controller/controller.kicad_pcb",
    ROOT / "pcbs/matrix-left/matrix-left.kicad_pcb",
    ROOT / "pcbs/matrix-right/matrix-right.kicad_pcb",
]
LAYERS = ["F.Cu", "B.Cu", "F.Paste", "B.Paste", "F.Silkscreen", "B.Silkscreen", "F.Mask", "B.Mask", "Edge.Cuts"]
TEARDROP_CLEARANCE = 0.2   # ティアドロップと別のネットの銅の最小距離 [mm]（ネットクラスのクリアランス）
# ================


def run(args):
    r = subprocess.run(args, capture_output=True, text=True)
    return r.returncode, r.stdout + r.stderr


def refilled_copy(path, work):
    """基板（とプロジェクトのファイル）を work にコピーし、ゾーンを塗り直して保存したもの。"""
    for ext in (".kicad_pcb", ".kicad_pro"):
        src = path.with_suffix(ext)
        if src.exists():
            shutil.copy(src, work / src.name)
    copy = work / path.name
    board = pcbnew.LoadBoard(str(copy))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(str(copy))
    return copy


def drc_errors(path, work):
    """DRC のエラー（severity が error のもの）の行。"""
    report = work / "drc.rpt"
    run(["kicad-cli", "pcb", "drc", "--severity-error", "-o", str(report), str(path)])
    text = report.read_text() if report.exists() else ""
    return [line for line in text.splitlines() if line.startswith("[")]


def _outlines(ps):
    return [[(pcbnew.ToMM(ps.Outline(i).CPoint(k).x), pcbnew.ToMM(ps.Outline(i).CPoint(k).y))
             for k in range(ps.Outline(i).PointCount())] for i in range(ps.OutlineCount())]


def _seg_dist(p, a, b):
    (ax, ay), (bx, by), (px, py) = a, b, p
    dx, dy = bx - ax, by - ay
    n = dx * dx + dy * dy
    t = 0 if n == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / n))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def _poly_dist(p, q):
    best = 1e9
    for a, b in ((p, q), (q, p)):
        for poly in a:
            for pt in poly:
                for other in b:
                    for k in range(len(other)):
                        best = min(best, _seg_dist(pt, other[k], other[(k + 1) % len(other)]))
    return best


def teardrop_problems(path):
    """ティアドロップ（保存された塗り）のうち、同じ層の別のネットの銅に TEARDROP_CLEARANCE より近いもの。"""
    board = pcbnew.LoadBoard(str(path))
    items = [it for it in list(board.GetTracks()) + [p for f in board.GetFootprints() for p in f.Pads()]
             if it.GetNetCode() > 0]
    out = []
    for z in board.Zones():
        if not z.IsTeardropArea():
            continue
        for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
            if not z.IsOnLayer(layer) or not z.HasFilledPolysForLayer(layer):
                continue
            zone = _outlines(z.GetFilledPolysList(layer))
            box = z.GetBoundingBox()
            box.Inflate(pcbnew.FromMM(0.5))
            for it in items:
                if it.GetNetCode() == z.GetNetCode() or not it.IsOnLayer(layer) or not box.Intersects(it.GetBoundingBox()):
                    continue
                ps = pcbnew.SHAPE_POLY_SET()
                it.TransformShapeToPolygon(ps, layer, 0, pcbnew.FromMM(0.002), pcbnew.ERROR_INSIDE)
                d = _poly_dist(zone, _outlines(ps))
                if d < TEARDROP_CLEARANCE - 1e-3:
                    who = (f"{it.GetParentFootprint().GetReference()}-{it.GetNumber()}" if isinstance(it, pcbnew.PAD)
                           else "線" if it.Type() != pcbnew.PCB_VIA_T else "ビア")
                    c = box.GetCenter()
                    out.append(f"{z.GetNetname()} のティアドロップ（{board.GetLayerName(layer)}、"
                               f"{pcbnew.ToMM(c.x):.2f}, {pcbnew.ToMM(c.y):.2f}）と {who}（{it.GetNetname()}）: {d:.3f} mm")
    return out


def export(path, force):
    name = path.stem
    out = path.parent / "jlcpcb"
    gerbers = out / "gerbers"
    with tempfile.TemporaryDirectory() as tmp:
        work = Path(tmp)
        board = refilled_copy(path, work)
        problems = drc_errors(board, work) + teardrop_problems(board)
        if problems:
            print(f"{name}: 確認で問題が見つかった（{len(problems)} 件）")
            for p in problems[:10]:
                print("  ", p)
            if not force:
                print("  書き出さなかった（--force で書き出す）")
                return False
        if gerbers.exists():
            shutil.rmtree(gerbers)
        gerbers.mkdir(parents=True)
        code, log = run(["kicad-cli", "pcb", "export", "gerbers", "-o", str(gerbers) + "/", "-l", ",".join(LAYERS),
                         "--subtract-soldermask", "--no-x2", "--no-netlist", str(board)])
        if code:
            raise SystemExit(f"{name}: ガーバーの書き出しに失敗した\n{log}")
        code, log = run(["kicad-cli", "pcb", "export", "drill", "-o", str(gerbers) + "/", "--format", "excellon",
                         "--drill-origin", "absolute", "--excellon-units", "mm", "--excellon-zeros-format", "decimal",
                         "--excellon-oval-format", "alternate", "--excellon-separate-th", "--generate-map",
                         "--map-format", "gerberx2", str(board)])
        if code:
            raise SystemExit(f"{name}: ドリルの書き出しに失敗した\n{log}")
    files = sorted(p for p in gerbers.iterdir() if p.is_file())
    archive = out / f"{name}-gerbers.zip"
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as z:
        for p in files:
            z.write(p, p.name)
    rel = archive.relative_to(ROOT) if archive.is_relative_to(ROOT) else archive
    print(f"{name}: {len(files)} ファイル → {rel}" + ("（問題があったが --force で書き出した）" if problems else ""))
    return True


args = [a for a in sys.argv[1:] if a != "--force"]
force = "--force" in sys.argv[1:]
ok = [export(Path(a).resolve(), force) for a in args] if args else [export(p, force) for p in DEFAULT_BOARDS]
sys.exit(0 if all(ok) else 1)
