# 平面形を作るための 2D の道具。
# - 辺（曲線）は直線 ("line", 点, 向き) か円弧 ("arc", 中心, 半径) で表す
# - 輪郭は「辺の並び」で表し、頂点は隣り合う辺の交点として計算する。オフセットした輪郭も同じ辺の並びから作れる
import math

from build123d import Face, Line, ThreePointArc, Vector, Wire


def intersect(c1, c2, near):
    """2 つの辺の交点のうち、near に近い方。接している（交わらない）ときは接点。"""
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
        s = math.sqrt(max(0.0, b * b - ((p - c).dot(p - c) - r * r)))
        pts = [p + d * (-b + s), p + d * (-b - s)]
    else:
        c, r, k, q = Vector(*c1[1]), c1[2], Vector(*c2[1]), c2[2]
        dist = (k - c).length
        a = (r * r - q * q + dist * dist) / (2 * dist)
        h = math.sqrt(max(0.0, r * r - a * a))
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
    """[(辺, 始点の目安), ...] から閉じた Face を作る。始点は前の辺との交点のうち、目安に近い方。"""
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


def project(p, line):
    """点 p から直線へ下ろした垂線の足。"""
    p, q, d = Vector(*p), Vector(*line[1]), Vector(*line[2]).normalized()
    return q + d * (p - q).dot(d)


def on_curve(p, curve, tol=1e-3):
    """点 p（XY）が辺の上にあるか。"""
    if curve[0] == "arc":
        (cx, cy), r = curve[1:]
        return abs(math.hypot(p.X - cx, p.Y - cy) - r) < tol
    (px, py), (dx, dy) = curve[1:]
    return abs(((p.X - px) * dy - (p.Y - py) * dx) / math.hypot(dx, dy)) < tol


def tangent(curve, p):
    """辺の p における向き（単位ベクトル、向きの正負は問わない）。"""
    if curve[0] == "line":
        return Vector(*curve[2]).normalized()
    c = Vector(*curve[1])
    r = Vector(p.X, p.Y) - c
    return Vector(-r.Y, r.X).normalized()


# ---- 輪郭（plan）: [(辺, 外側へずらすときの shifted() の符号, 辺の種類, 始点の目安), ...] ----

def plan_curves(plan, grow):
    """輪郭の各辺を外へずらす。grow は {辺の種類: ずらす量}。"""
    return [shifted(c, s * grow[kind]) for c, s, kind, _ in plan]


def plan_points(plan, grow):
    """ずらした輪郭の頂点。"""
    curves = plan_curves(plan, grow)
    pts = []
    for i, (_, _, _, hint) in enumerate(plan):
        # 直線と交わる角は、目安をずらした直線の上へ移す（ほぼ接している円弧と直線で、交点の選び方がぶれないように）
        for c in (curves[i - 1], curves[i]):
            if c[0] == "line":
                hint = project(hint, c)
        pts.append(intersect(curves[i - 1], curves[i], hint))
    return pts


def plan_face(plan, grow):
    """ずらした輪郭の Face。"""
    return edges_face(plan_curves(plan, grow), plan_points(plan, grow))


def plan_corners(plan, grow, min_angle=1.0):
    """ずらした輪郭の頂点のうち、辺が接線つながりでない（角になっている）もの。"""
    curves = plan_curves(plan, grow)
    out = []
    for i, p in enumerate(plan_points(plan, grow)):
        t0, t1 = tangent(curves[i - 1], p), tangent(curves[i], p)
        if math.degrees(math.asin(min(1.0, abs(t0.cross(t1).Z)))) > min_angle:
            out.append(p)
    return out


def outward_normal(plan, i, p):
    """輪郭の i 番目の辺の、点 p における外向きの単位法線。"""
    curve, s, _, _ = plan[i]
    if curve[0] == "arc":
        r = (Vector(p.X, p.Y) - Vector(*curve[1])).normalized()
        return r * s
    dx, dy = curve[2]
    return Vector(-dy, dx).normalized() * s
