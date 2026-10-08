# 左手ケースの平面形の共通部分。ボトムケースの外形を基準にして、各部の輪郭はそこからのオフセットで作る。
# - 座標: XY がケースの平面（y は奥が正）。単位は mm
# - 辺（曲線）は直線 ("line", 点, 向き) か円弧 ("arc", 中心, 半径) で表す
import math

from build123d import Face, Line, Pos, ThreePointArc, Vector, Wire, Circle

# ボトムケースの外形の辺
HALF_W = 67.825
RIGHT = ("line", (HALF_W, 0.0), (0.0, 1.0))
LEFT = ("line", (-HALF_W, 0.0), (0.0, 1.0))
BACK_ARC = ("arc", (0.0, -266.243), 308.896)        # 奥の辺
LEFT_ARC = ("arc", (7.3122, 159.531), 203.5)        # 手前左の辺
FRONT = ("line", (0.0, -42.6534), (1.0, 0.0))       # 手前中央の水平な辺
THUMB_ARC = ("arc", (-6.2286, -222.398), 179.745)   # 親指の下の辺（凹）
THUMB_LINE = ("line", (48.6409, -51.2326), (6.9706, -2.2649))  # 親指の下の辺に続く直線
SLANT = ("line", (67.8250, -27.7716), (-8.7269, -26.8587))     # 右下の斜めの辺

# M2 ネジの位置
SCREWS = [
    (45.2441, 38.5128),
    (-10.6426, 41.6691),
    (-65.8622, 34.7310),
    (-65.8264, -29.5140),
    (-10.5735, -41.8534),
    (57.6020, -53.3030),
]

# ボスを除いた外形: (辺, 外側へ広げるときの shifted() の符号, 左右の辺か, 始点の目安)。
# 始点は前の辺との交点のうち、目安に近い方
PLAN = [
    (RIGHT, -1, True, (67.825, -27.7716)),
    (BACK_ARC, 1, False, (67.825, 35.1148)),
    (LEFT, 1, True, (-67.825, 35.1148)),
    (LEFT_ARC, 1, False, (-67.825, -29.5897)),
    (FRONT, -1, False, (-15.7901, -42.6534)),
    (THUMB_ARC, -1, False, (-5.8494, -42.6534)),
    (THUMB_LINE, -1, False, (48.6409, -51.2326)),
    (SLANT, 1, True, (58.4, -53.9)),
]


def intersect(c1, c2, near):
    """2 つの辺の交点のうち、near に近い方。"""
    near = Vector(*near)
    if c1[0] == "arc" and c2[0] == "line":
        c1, c2 = c2, c1
    if c1[0] == "line" and c2[0] == "line":
        p, d, q, e = (Vector(*v) for v in (*c1[1:], *c2[1:]))
        t = ((q - p).X * e.Y - (q - p).Y * e.X) / (d.X * e.Y - d.Y * e.X)
        return p + d * t
    if c1[0] == "line":
        p, d = Vector(*c1[1]), Vector(*c1[2]).normalized()
        c, r = Vector(*c2[1]), c2[2]
        b = (p - c).dot(d)
        s = math.sqrt(b * b - ((p - c).dot(p - c) - r * r))
        pts = [p + d * (-b + s), p + d * (-b - s)]
    else:
        c, r, k, q = Vector(*c1[1]), c1[2], Vector(*c2[1]), c2[2]
        dist = (k - c).length
        a = (r * r - q * q + dist * dist) / (2 * dist)
        h = math.sqrt(r * r - a * a)
        u = (k - c).normalized()
        m, n = c + u * a, Vector(-u.Y, u.X)
        pts = [m + n * h, m - n * h]
    return min(pts, key=lambda v: (v - near).length)


def arc_mid(p0, p1, center, r):
    """円弧（短い方）上で p0 と p1 の中間の点。"""
    (cx, cy) = center
    a0 = math.atan2(p0.Y - cy, p0.X - cx)
    a1 = math.atan2(p1.Y - cy, p1.X - cx)
    a = a0 + ((a1 - a0 + math.pi) % (2 * math.pi) - math.pi) / 2
    return (cx + r * math.cos(a), cy + r * math.sin(a))


def edges_face(curves, pts):
    """curves[i] が pts[i] から pts[i+1] へ向かう閉じた輪郭の Face。法線は常に +Z（時計回りの輪郭は向きを反転する）。"""
    n = len(curves)
    edges = []
    for i, curve in enumerate(curves):
        p0, p1 = Vector(*pts[i]), Vector(*pts[(i + 1) % n])
        edges.append(Line(p0, p1) if curve[0] == "line" else ThreePointArc(p0, arc_mid(p0, p1, *curve[1:]), p1))
    face = Face(Wire(edges))
    if face.normal_at().Z < 0:
        return edges_face([curves[(n - 2 - j) % n] for j in range(n)], pts[::-1])
    return face


def chain_face(chain):
    """[(辺, 始点の目安), ...] から閉じた Face を作る。"""
    pts = [intersect(chain[i - 1][0], chain[i][0], chain[i][1]) for i in range(len(chain))]
    return edges_face([c for c, _ in chain], pts)


def path_face(path):
    """[(点, 次の点までの辺), ...] から閉じた Face を作る。辺は None（直線）か ("arc", 中心, 半径)。"""
    return edges_face([c or ("line",) for _, c in path], [p for p, _ in path])


def shifted(curve, d):
    """辺を d だけずらす。円弧は半径を、直線は左手側の法線方向へ。"""
    if curve[0] == "arc":
        return ("arc", curve[1], curve[2] + d)
    (px, py), (dx, dy) = curve[1:]
    k = d / math.hypot(dx, dy)
    return ("line", (px - dy * k, py + dx * k), (dx, dy))


def plan_curves(grow=0.0, side_grow=None):
    """ボトムケースの外形（ボスなし）の辺を grow だけ外へずらしたもの。
    左右の辺と右下の斜めの辺は side_grow だけずらす（省略時は grow）。"""
    side_grow = grow if side_grow is None else side_grow
    return [shifted(c, s * (side_grow if side else grow)) for c, s, side, _ in PLAN]


def plan(grow=0.0, side_grow=None):
    """plan_curves() の辺で囲まれた Face。"""
    curves = plan_curves(grow, side_grow)
    hints = []
    for i, (_, _, _, hint) in enumerate(PLAN):
        # 直線と交わる角は、目安をずらした直線の上へ移す（親指の下の円弧と直線はほぼ接していて、
        # 交点が 1 mm ほどの間隔で 2 つあるので、目安が離れていると選び方がぶれる）
        for c in (curves[i - 1], curves[i]):
            if c[0] == "line":
                hint = project(hint, c)
        hints.append(hint)
    return chain_face(list(zip(curves, hints)))


def project(p, line):
    """点 p から直線へ下ろした垂線の足。"""
    p, q, d = Vector(*p), Vector(*line[1]), Vector(*line[2]).normalized()
    return q + d * (p - q).dot(d)


def with_bosses(face, r):
    """ネジ位置に半径 r の円を足す。"""
    return face + [Pos(x, y) * Circle(r) for x, y in SCREWS]


def on_curve(p, curve, tol=1e-3):
    """点 p（XY）が辺の上にあるか。"""
    if curve[0] == "arc":
        (cx, cy), r = curve[1:]
        return abs(math.hypot(p.X - cx, p.Y - cy) - r) < tol
    (px, py), (dx, dy) = curve[1:]
    return abs(((p.X - px) * dy - (p.Y - py) * dx) / math.hypot(dx, dy)) < tol
