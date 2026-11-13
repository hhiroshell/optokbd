# ボトムケースとトップケースで共有する配置。すべてプレートの外形からのオフセットで決める（v0.1.0 で使っていた寸法関係）。
# - 「edge」はプレートの奥と手前の辺、「side」は左右と右下の斜めの辺（plate.PLAN の辺の種類）
from dataclasses import dataclass

from build123d import Compound, Mesher, Plane, Pos, Rectangle, Vector, export_step, export_stl, mirror

import plate as P
from outline import intersect, outward_normal, plan_face, shifted

# ---- プレートからのオフセット ----
PLATE_GAP = 0.7       # プレートの外周とケースの内面（トップケースの内側、ボトムケースの壁の内側）のすき間
BOTTOM_WALL = 2.8     # ボトムケースの奥と手前の壁。左右と斜めの辺には壁がなく、外形はプレートの辺と同じ位置
TOP_FIT = 0.25        # トップケースの外壁の内面と、ボトムケースの外形のすき間。合わせ目を細く見せるため 0.5（奥と手前）、
                      # 0.7（左右）から詰めた（タブのポケットの横のすき間 TAB_SLOT_CLEARANCE で、0.2 ではきつかったのに合わせる）
TOP_SIDE_GAP = 0.6    # 左右と右下の斜めの辺: プレートの外周とトップケースの外壁の内面のすき間。外へ出たマトリクス基板
                      # （matrix_mock.MATRIX_SIDE_GROW 0.3）と 0.3、プレートのフォーム（foam.PLATE_FOAM_GROW 0.4）と 0.2 残る所まで、
                      # PLATE_GAP から詰めた（どちらも上からかぶせるときに内面のそばを通る）
TOP_WALL = 2.0        # トップケースの外壁の厚さ（左右。内面を PLATE_GAP から TOP_SIDE_GAP に詰める前の）
Z_OFFSET = 7.0        # ボトムケースの z = トップケースの z + Z_OFFSET（ボトムケースの壁の上端がトップケースの段差に当たる）。
                      # ボトムケースの壁を 0.5 上げたのに合わせて 6.5 から上げた（トップケースの外壁の下端の、机からの高さは変わらない）
REVEAL = 1.5          # トップケースの下端を、ボトムケースの傾斜した底面から上げる量（手前でもボトムケースの側面が帯状に見える）

BOTTOM_OUTER = {"edge": PLATE_GAP + BOTTOM_WALL, "side": 0.0}           # ボトムケースの外形の基準（底面の傾斜や収納部の位置を決める）
# ボトムケースの実際の外形: 左右と右下の斜めの辺は、トップケースの外壁の内面（TOP_SIDE_GAP）から TOP_FIT 内側まで外へ出す。
# 外へ出るのは床より下の側面だけ（床より上の左右には壁がない）。トップケースの内面は高さによらず TOP_SIDE_GAP なので、
# かぶせるときに外へ出たマトリクス基板とプレートのフォームが通る
BOTTOM_SIDE = {"edge": BOTTOM_OUTER["edge"], "side": TOP_SIDE_GAP - TOP_FIT}
BOTTOM_FLOOR = {"edge": PLATE_GAP, "side": 1.0}                         # ボトムケースの床（左右は外形より外まで）
TOP_CAVITY = {"edge": PLATE_GAP, "side": TOP_SIDE_GAP}                     # トップケースの内側（ボトムケースの床の上の空間）
TOP_INNER = {"edge": BOTTOM_OUTER["edge"] + TOP_FIT, "side": TOP_SIDE_GAP}  # トップケースの外壁の内面（下側）
# トップケースの外形。すき間を詰める前（奥と手前の TOP_FIT 0.5、左右の PLATE_GAP）の位置のまま、壁を内側へ厚くした
TOP_OUTER = {"edge": BOTTOM_OUTER["edge"] + 0.5 + TOP_WALL, "side": PLATE_GAP + TOP_WALL}


def face(grow):
    return plan_face(P.PLAN, grow)


# ---- STL / 3MF の書き出し ----
MESH_LINEAR = 0.0001    # メッシュの細かさ。線の許容差は辺の大きさに対する比（130 mm の辺で 0.013 mm）、
MESH_ANGULAR = 0.02     # 角度は隣り合う三角形の向きの差（rad、約 1.1°）。build123d の既定（0.001, 0.1）では、フィレットや
                        # 大きな円弧が 5.7° ごとの平らな面になる


def export(shape_or_parts, path):
    """STEP / STL / 3MF に書き出す。部品のリスト [(名前, 形)] なら、3MF では部品ごとに名前をつける。"""
    parts = shape_or_parts if isinstance(shape_or_parts, list) else [("part", shape_or_parts)]
    export_step(Compound([s for _, s in parts]), path.with_suffix(".step"))
    export_stl(Compound([s for _, s in parts]), path.with_suffix(".stl"), MESH_LINEAR, MESH_ANGULAR)
    m = Mesher()  # 3D プリンタのスライサー用。単位は mm
    for name, s in parts:
        m.add_shape(s, MESH_LINEAR, MESH_ANGULAR, part_number=name)
    m.write(path.with_suffix(".3mf"))


# ---- 右手 ----
# 右手のケースは、左手のケースを左右に反転した形。プレートは同じ板を裏返して使い、右手のマトリクス基板はキー・J1・外形が
# 左手の鏡像の配置になっている。コントローラー基板は左右対称で共通。反転の面はケースの左右の中央（奥と手前の辺の中央のネジの x）
MIRROR_X = (P.LEFT[1][0] + P.RIGHT[1][0]) / 2


def right_hand(shape):
    return mirror(shape, Plane.YZ.offset(MIRROR_X))


def export_hands(name, shape, out):
    """左手の形を out/<name>-l.*、左右に反転した右手の形を out/<name>-r.* に書き出す。"""
    export([(f"{name}-l", shape)], out / f"{name}-l")
    export([(f"{name}-r", right_hand(shape))], out / f"{name}-r")


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
TAB_SLOT_CLEARANCE = 0.25  # トップケースのポケットとタブの横のすき間（片側）。タブの x の位置決めを兼ねる。0.15 では
                           # 届いたプレートのタブがポケットに当たってスムーズに入らず、0.2 でもまだきつかった


def tab_strip(tab, half_width):
    """タブの中心から ±half_width の帯（タブのある辺の付近だけ）。"""
    y0, y1 = tab.y0 - 8, tab.y1 + 8
    return Pos(tab.cx, (y0 + y1) / 2) * Rectangle(2 * half_width, y1 - y0)


def slot_half_width(tab):
    return tab.width / 2 + TAB_SLOT_CLEARANCE
