# スイッチプレート（左手）。ケースの平面形はすべてこのプレートの外形から決める。
# - 座標はプレート DXF のまま（mm、y は上向き＝奥が正、原点はキー配置の原点）
# - 外形のうちタブを除いた部分は辺の定義として持ち、タブとキーの位置は DXF から読む
import math
from dataclasses import dataclass
from pathlib import Path

import ezdxf
from build123d import CenterArc, Face, Line, ThreePointArc, Vector, Wire

from outline import plan_face

PLATE_DXF = Path(__file__).parent.parent / "plate" / "plate_left.dxf"
OUTLINE_LAYER = "10 OUTLINE"
CUTOUT_LAYER = "00 CUTOUT"  # 14 mm 角のスイッチ穴（1U の枠と違って隣どうしが接しない）

# 外形（タブを除く）の辺
RIGHT = ("line", (133.355, 0.0), (0.0, 1.0))
BACK = ("arc", (66.68, -295.896), 295.891)               # 奥の辺
LEFT = ("line", (0.005, 0.0), (0.0, 1.0))
FRONT_LEFT = ("arc", (61.7143, 71.295), 149.4)           # 手前左の辺
THUMB = ("arc", (61.6719, -276.728), 198.623)            # 親指の下の辺（凹）。手前左の辺とは接線つながり
THUMB_LINE = ("line", (118.045, -86.2728), (9.133, -2.7052))  # 親指の下に続く直線（R3C6 の下）。親指の下の辺とは接線つながり
SLANT = ("line", (127.178, -88.978), (6.177, 20.8548))   # 右下の斜めの辺

# 外形: [(辺, 外側へずらすときの符号, 辺の種類, 始点の目安), ...]
# 辺の種類: "edge" は奥と手前の辺（ボトムケースに壁がある）、"side" は左右と右下の斜めの辺（ボトムケースに壁がない）
PLAN = [
    (RIGHT, -1, "side", (133.355, -68.1232)),
    (BACK, 1, "edge", (133.355, -7.615)),
    (LEFT, 1, "side", (0.005, -7.615)),
    (FRONT_LEFT, 1, "edge", (0.005, -64.765)),
    (THUMB, -1, "edge", (61.72, -78.105)),
    (THUMB_LINE, -1, "edge", (118.045, -86.2728)),
    (SLANT, -1, "side", (127.178, -88.978)),
]
I_RIGHT, I_BACK, I_LEFT, I_FRONT_LEFT, I_THUMB, I_THUMB_LINE, I_SLANT = range(len(PLAN))


def base(grow=0.0, side_grow=None):
    """タブを除いた外形を外へずらした Face。side_grow は左右と斜めの辺（省略時は grow）。"""
    return plan_face(PLAN, {"edge": grow, "side": grow if side_grow is None else side_grow})


def _dxf_edges(layer, snap=0.01):
    """レイヤーの線と円弧。端点どうしが snap 以内なら同じ点にそろえる（手描きの DXF で端点がわずかにずれているため）。"""
    raw = []
    for e in ezdxf.readfile(PLATE_DXF).modelspace():
        if e.dxf.layer != layer:
            continue
        if e.dxftype() == "LINE":
            raw.append((Vector(*tuple(e.dxf.start)[:2]), Vector(*tuple(e.dxf.end)[:2]), None))
        elif e.dxftype() == "ARC":
            a0, a1 = e.dxf.start_angle, e.dxf.end_angle
            arc = CenterArc(tuple(e.dxf.center)[:2], e.dxf.radius, a0, (a1 - a0) % 360)
            raw.append((arc.position_at(0), arc.position_at(1), arc.position_at(0.5)))
    points = []

    def snapped(p):
        for q in points:
            if (p - q).length < snap:
                return q
        points.append(p)
        return p

    out = []
    for p0, p1, mid in raw:
        p0, p1 = snapped(p0), snapped(p1)
        out.append(Line(p0, p1) if mid is None else ThreePointArc(p0, mid, p1))
    return out


def outline():
    """DXF の外形（タブ込み）。"""
    face = Face(Wire.combine(_dxf_edges(OUTLINE_LAYER))[0])
    return face if face.normal_at().Z > 0 else -face


@dataclass
class Key:
    center: Vector
    angle: float  # 度（反時計回り）


def keys():
    """スイッチ穴から、キーの中心と回転角を読む。"""
    out = []
    for w in Wire.combine(_dxf_edges(CUTOUT_LAYER)):
        vs = [Vector(v.X, v.Y) for v in w.vertices()]
        c = sum(vs, Vector(0, 0)) * (1 / len(vs))
        e = w.edges()[0]
        d = e.position_at(1) - e.position_at(0)
        out.append(Key(c, math.degrees(math.atan2(d.Y, d.X)) % 90))
    return sorted(out, key=lambda k: (k.center.X, -k.center.Y))


@dataclass
class Tab:
    x0: float
    x1: float
    edge: int        # 根元がある辺（PLAN の番号）
    y0: float
    y1: float

    @property
    def width(self):
        return self.x1 - self.x0

    @property
    def cx(self):
        return (self.x0 + self.x1) / 2


def tabs():
    """ガスケットタブ。DXF の外形からタブを除いた外形を引いた残りとして見つける。タブの両側は y 軸に平行な辺とする。"""
    # 本体を少し広げて引く（DXF の円弧と辺の定義のわずかな差が、細い切れ端として残らないように）
    rest = outline() - base(0.05)
    out = []
    for f in rest.faces():
        if f.area < 1.0:
            continue
        bb = f.bounding_box()
        c = f.center()
        # 根元の辺: タブの中心から最も近い、奥か手前の辺
        edge = min((i for i, (_, _, kind, _) in enumerate(PLAN) if kind == "edge"),
                   key=lambda i: abs(_dist(c, PLAN[i][0])))
        out.append(Tab(bb.min.X, bb.max.X, edge, bb.min.Y, bb.max.Y))
    return sorted(out, key=lambda t: (-t.y1, t.x0))


def _dist(p, curve):
    if curve[0] == "arc":
        return (Vector(p.X, p.Y) - Vector(*curve[1])).length - curve[2]
    q, d = Vector(*curve[1]), Vector(*curve[2]).normalized()
    v = Vector(p.X, p.Y) - q
    return v.X * d.Y - v.Y * d.X

