# README の分解図（左手）。部品を組み付けた位置から真上へずらして積み、斜め上から見た線画（隠れ線を消したもの）に
# 部品の名前を引き出し線で添えて、SVG に書き出す。
# - 座標はボトムケースの座標（z は底面の基準が 0）。トップケース、プレート、ガスケットは L.Z_OFFSET を足した組み付けた位置
# - キーキャップとスイッチは、形を大まかにかたどったもの（MX のスイッチと、上へすぼまるキーキャップ）
# - 実行: uv run python exploded.py   （kicad-cli と、pcbnew の使える /usr/bin/python3 が必要。../docs/images/exploded-l.svg を
#   書き出す）
from pathlib import Path

from build123d import Align, Box, Compound, Edge, Plane, Pos, Rot, RectangleRounded, Vector, extrude, loft
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopExp import TopExp_Explorer

import bay
import bottom as B
import foam as F
import gasket as G
import layout as L
import matrix_mock as M
import plate as P
import top as T

OUT = Path(__file__).resolve().parent.parent / "docs" / "images" / "exploded-l.svg"

VIEW = (-0.5, -1.0, 0.75)    # 部品から見る人への向き（手前の左の斜め上から見る）
UP = (0.0, 0.0, 1.0)
SCALE = 4.0                  # SVG の 1 mm あたりの px
LINE_W = 0.35                # 線の太さ（mm）
FONT = 4.2                   # 名前の文字の大きさ（mm）



def keycaps():
    """キーキャップとスイッチ（大まかな形）。プレートの上面にスイッチの上のハウジングが載る、組み付けた位置。"""
    z0 = G.UPPER_Z + L.Z_OFFSET   # プレートの上面
    out = []
    for k in P.keys():
        at = Pos(k.center.X, k.center.Y, 0) * Rot(0, 0, k.angle)
        lower = Pos(0, 0, z0 - 5.0) * Box(14.0, 14.0, 5.0 - P.PLATE_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
        upper = loft([Plane.XY.offset(z0) * RectangleRounded(15.6, 15.6, 1.0),
                      Plane.XY.offset(z0 + 5.6) * RectangleRounded(11.0, 11.6, 1.0)])
        cap = loft([Plane.XY.offset(z0 + 6.6) * RectangleRounded(18.0, 18.0, 1.5),
                    Plane.XY.offset(z0 + 14.0) * RectangleRounded(12.5, 13.5, 2.0)])
        out += [at * lower, at * upper, at * cap]
    return Compound(out)


def layers():
    """[(名前, 形, 下の部品との間の高さ, 名前を置く側)]（下から積む順。形は組み付けた位置）。収納部の天井はボトムケースに
    接着するので、ボトムケースと一緒にする（分けると上のフォームに隠れる）。ボトムケースは大きく、底蓋をほとんど隠すので、
    その下だけ間を広げる。"""
    board, xiao, usb, battery = bay.dummies()
    matrix, _ = M.matrix_mock()
    floor = F.floor_foams(F.case_cuts())["l"][0]
    z = L.Z_OFFSET
    return [
        ("Lid, controller PCB (XIAO nRF52840) and battery", Compound([bay.lid(), board, xiao, usb, battery]), 0, "right"),
        ("Bottom case (3D-printed PLA) with the bay ceiling", Compound([B.build(), bay.ceiling()]), 60, "left"),
        ("Floor foam (PORON 2 mm)", Pos(0, 0, B.FLOOR_Z) * extrude(F.to_face(floor), F.FLOOR_FOAM_T), 45, "right"),
        ("Matrix PCB with hot-swap sockets", Pos(0, 0, M.PCB_TOP_Z) * matrix, 25, "left"),
        ("Plate foam (PORON 3 mm)", Pos(0, 0, M.PCB_TOP_Z) * extrude(F.to_face(F.plate_foam()), F.PLATE_FOAM_T), 25,
         "right"),
        ("FR-4 switch plate with PORON gaskets", Compound([Pos(0, 0, G.PLATE_Z + z) * P.solid()]
                                                      + [Pos(0, 0, z) * g for _, g in G.gaskets()]), 25, "left"),
        ("Top case (3D-printed PLA)", Pos(0, 0, z) * T.build(), 30, "right"),
        ("Keycaps and MX switches", keycaps(), 30, "left"),
    ]


def camera():
    ax = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*VIEW))
    ax.SetYDirection(gp_Dir(*UP))
    return ax


def to_2d(ax, p):
    """3D の点を、投影した面の座標（x は右、y は上）へ。"""
    v = Vector(p)
    xd, yd = ax.XDirection(), ax.YDirection()
    return (v.X * xd.X() + v.Y * xd.Y() + v.Z * xd.Z(), v.X * yd.X() + v.Y * yd.Y() + v.Z * yd.Z())


def hidden_line(shape, ax):
    """隠れ線を消した線: (稜線と輪郭線, なめらかにつながる面の境目の線)。どちらも投影した面の座標の Edge。"""
    algo = HLRBRep_Algo()
    algo.Add(shape.wrapped)
    algo.Projector(HLRAlgo_Projector(ax))
    algo.Update()
    algo.Hide()
    hlr = HLRBRep_HLRToShape(algo)

    def edges(c):
        out = []
        if c.IsNull():
            return out
        ex = TopExp_Explorer(c, TopAbs_EDGE)
        while ex.More():
            out.append(Edge(ex.Current()))
            ex.Next()
        return out

    return edges(hlr.VCompound()) + edges(hlr.OutLineVCompound()), edges(hlr.Rg1LineVCompound())


def polyline(e, tol=0.05):
    if e.geom_type.name == "LINE":
        return [e @ 0, e @ 1]
    n = max(2, min(200, int(e.length / 0.5) + 1))
    return [e @ (i / (n - 1)) for i in range(n)]


def anchor(shape, ax, side):
    """引き出し線の先: 部品の上の方の頂点のうち、名前を置く側にいちばん寄ったもの（投影した面の座標）。"""
    bb = shape.bounding_box()
    pts = [to_2d(ax, v) for v in shape.vertices() if v.Z >= bb.max.Z - 0.4 * (bb.max.Z - bb.min.Z)]
    return min(pts) if side == "left" else max(pts)


def main():
    ax = camera()
    parts, top = [], None
    for name, shape, clear, side in layers():
        bb = shape.bounding_box()
        dz = 0.0 if top is None else top + clear - bb.min.Z
        parts.append((name, Pos(0, 0, dz) * shape, side))
        top = bb.max.Z + dz
    sharp, smooth = hidden_line(Compound([s for _, s, _ in parts]), ax)
    paths = [(polyline(e), LINE_W) for e in sharp] + [(polyline(e), LINE_W * 0.5) for e in smooth]
    xs = [p.X for pl, _ in paths for p in pl]
    ys = [p.Y for pl, _ in paths for p in pl]
    # 名前は図の左右の外に置き、部品の頂点から水平な引き出し線を引く
    gx0, gx1 = min(xs), max(xs)
    label_gap = 8.0

    def label_w(side):   # 文字の幅は大まかに 1 文字 FONT * 0.55
        return max(len(n) for n, _, s in parts if s == side) * FONT * 0.5 + 2.0

    m = 6.0
    x0, x1 = gx0 - label_gap - label_w("left") - m, gx1 + label_gap + label_w("right") + m
    y0, y1 = min(ys) - m, max(ys) + m
    w, h = x1 - x0, y1 - y0

    def sx(x):
        return x - x0

    def sy(y):
        return y1 - y

    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w * SCALE:.0f}" height="{h * SCALE:.0f}" '
           f'viewBox="0 0 {w:.2f} {h:.2f}">', '<rect width="100%" height="100%" fill="white"/>',
           '<g fill="none" stroke="#222" stroke-linecap="round" stroke-linejoin="round">']
    for pl, lw in paths:
        d = "M" + " L".join(f"{sx(p.X):.2f},{sy(p.Y):.2f}" for p in pl)
        svg.append(f'<path d="{d}" stroke-width="{lw}"/>')
    svg.append("</g>")
    svg.append(f'<g font-family="Helvetica, Arial, sans-serif" font-size="{FONT}" fill="#222">')
    for name, shape, side in parts:
        ax_, ay = anchor(shape, ax, side)
        if side == "left":
            lx, anchor_text = gx0 - label_gap, "end"
        else:
            lx, anchor_text = gx1 + label_gap, "start"
        svg.append(f'<circle cx="{sx(ax_):.2f}" cy="{sy(ay):.2f}" r="0.8" fill="#222"/>')
        svg.append(f'<path d="M{sx(ax_):.2f},{sy(ay):.2f} L{sx(lx):.2f},{sy(ay):.2f}" stroke="#222" stroke-width="0.3"/>')
        tx = lx - 1.5 if side == "left" else lx + 1.5
        svg.append(f'<text x="{sx(tx):.2f}" y="{sy(ay) + 1.5:.2f}" text-anchor="{anchor_text}">{name}</text>')
    svg += ["</g>", "</svg>"]
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(svg))
    print(f"{OUT}: {len(sharp)} + {len(smooth)} edges, {w:.0f} x {h:.0f} mm")


if __name__ == "__main__":
    main()
