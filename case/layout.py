# ボトムケースとトップケースで共有する配置。すべてプレートの外形からのオフセットで決める（v0.1.0 で使っていた寸法関係）。
# - 「edge」はプレートの奥と手前の辺、「side」は左右と右下の斜めの辺（plate.PLAN の辺の種類）
from dataclasses import dataclass

from build123d import Pos, Rectangle, Vector

import plate as P
from outline import intersect, outward_normal, plan_face, shifted

# ---- プレートからのオフセット ----
PLATE_GAP = 0.7       # プレートの外周とケースの内面（トップケースの内側、ボトムケースの壁の内側）のすき間
BOTTOM_WALL = 2.8     # ボトムケースの奥と手前の壁。左右と斜めの辺には壁がなく、外形はプレートの辺と同じ位置
TOP_FIT = 0.5         # トップケースの外壁の内面と、ボトムケースの外形のすき間（奥と手前。左右は PLATE_GAP）
TOP_WALL = 2.0        # トップケースの外壁の厚さ
Z_OFFSET = 6.5        # ボトムケースの z = トップケースの z + Z_OFFSET（ボトムケースの下げた壁の上端がトップケースの段差に当たる）

BOTTOM_OUTER = {"edge": PLATE_GAP + BOTTOM_WALL, "side": 0.0}           # ボトムケースの外形
BOTTOM_FLOOR = {"edge": PLATE_GAP, "side": 1.0}                         # ボトムケースの床（左右は外形より外まで）
TOP_CAVITY = {"edge": PLATE_GAP, "side": PLATE_GAP}                     # トップケースの内側（ボトムケースの床の上の空間）
TOP_INNER = {"edge": BOTTOM_OUTER["edge"] + TOP_FIT, "side": PLATE_GAP}  # トップケースの外壁の内面（下側）
TOP_OUTER = {"edge": TOP_INNER["edge"] + TOP_WALL, "side": PLATE_GAP + TOP_WALL}  # トップケースの外形


def face(grow):
    return plan_face(P.PLAN, grow)


# ---- ネジ（M2）----
SCREW_FROM_PLATE = 2.7   # ネジの中心: プレートの辺から外へ（ボトムケースの壁の中、外形から 0.8 mm 内側）
SCREW_SIDE_INSET = 3.2   # 四隅のネジ: 左右と斜めの辺から内側へ（ボトムケースのネジ頭の穴と側面の間に 1.2 mm の肉を残す）
BOSS_R = 2.0             # ボトムケースのボス（= ネジ頭の穴）
HOLE_R = 1.2             # ネジ穴


@dataclass
class Screw:
    pos: Vector
    normal: Vector   # 外向きの法線（ネジが乗っている辺の）
    back: bool       # 奥の辺のネジか


def screws():
    """四隅に 4 本と、奥と手前の辺の中央に 1 本ずつ。"""
    def on(edge_i, other, near):
        line = shifted(P.PLAN[edge_i][0], P.PLAN[edge_i][1] * SCREW_FROM_PLATE)
        p = intersect(line, other, near)
        return Screw(Vector(p.X, p.Y), outward_normal(P.PLAN, edge_i, p), edge_i == P.I_BACK)

    def inset(side_i):
        c, s, _, _ = P.PLAN[side_i]
        return shifted(c, -s * SCREW_SIDE_INSET)

    left, right = P.LEFT[1][0], P.RIGHT[1][0]
    middle = ("line", ((left + right) / 2, 0.0), (0.0, 1.0))
    return [
        on(P.I_BACK, inset(P.I_LEFT), (left, 0)),
        on(P.I_BACK, middle, ((left + right) / 2, 0)),
        on(P.I_BACK, inset(P.I_RIGHT), (right, 0)),
        on(P.I_FRONT_LEFT, inset(P.I_LEFT), (left, -70)),
        on(P.I_THUMB, middle, ((left + right) / 2, -80)),
        on(P.I_THUMB_LINE, inset(P.I_SLANT), (127, -92)),
    ]


# ---- ガスケットタブ ----
TAB_SLOT_CLEARANCE = 0.15  # トップケースのポケットとタブの横のすき間（片側）。タブの x の位置決めを兼ねる
PILLAR_CLEARANCE = 0.2     # ボトムケースの柱（タブが乗る張り出し）と、トップケースのポケットの横のすき間（片側）


def tab_strip(tab, half_width):
    """タブの中心から ±half_width の帯（タブのある辺の付近だけ）。"""
    y0, y1 = tab.y0 - 8, tab.y1 + 8
    return Pos(tab.cx, (y0 + y1) / 2) * Rectangle(2 * half_width, y1 - y0)


def slot_half_width(tab):
    return tab.width / 2 + TAB_SLOT_CLEARANCE


def pillar_half_width(tab):
    return slot_half_width(tab) - PILLAR_CLEARANCE
