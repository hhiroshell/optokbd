# 遊舎工房のレーザー加工サービス（その2、フォーム = PORON）に出す、フォームとガスケットのカットデータ（左右 2 台分）。
# - 床のフォーム（2 mm）: マトリクス基板とボトムケースの床の間（2.5 mm）。ふだんは基板に触れず、基板が大きくたわんだときだけ
#   受け止める。基板の裏の部品（ソケット、ダイオード、J4 = マトリクスのコネクタ）、スイッチの軸と突起（基板の裏へ出る）、
#   収納部の天井の開口、床から出ているもの（奥のネジのまわりの台）を切り抜く
# - プレートのフォーム（3 mm）: プレートとマトリクス基板の間（3.4 mm）。スイッチの 14 mm 角の穴
# - ガスケット（3 mm、接着シート付き）: タブ 1 つにつき上下 2 枚。上下とも、シールの面をタブに向けて貼る。シールはデザインを
#   上から見た面の裏に付くので、上のガスケットは組み付けた形を上から見た形、下のガスケットはその鏡像。右手は左手の鏡像なので、
#   右手の上は左手の下と、右手の下は左手の上と同じ形。先端と側面はタブにそろえ、内側の辺の違う 2 案（GASKET_VARIANTS）を
#   並べて、組み立てるときに選ぶ（目印の丸のある側が上限案）
# - 枚数: 床のフォームは左右それぞれ FOAM_SETS 枚、プレートのフォーム（左右共用、右手には裏返す）は 2 × FOAM_SETS 枚、
#   ガスケットは案ごと・貼る所ごとに GASKET_SETS 枚。フォームは 500 × 200、ガスケットは 297 × 190（接着シート付き）
# - フォームの外形は、奥と手前の辺がプレートの外形の少し外（ボトムケースの壁の内面の手前）、左右の辺は床のフォームが基板の外形、
#   プレートのフォームがプレートの外形の少し外
# - 右手の基板は、配置スクリプトの原点 (185, 50) で、プレートの座標を x で反転した向き。基板のフットプリントは鏡像にしないので、
#   裏の部品の切り抜きは右手の基板から求める。ケース（床の切り抜き）は左手の鏡像
# - 2D の形は shapely で作る（曲線は CHORD 以下の折れ線）。build123d の 2D のオフセットは、輪郭が複数に分かれると作れない
# - カットのパス: 穴（内周）はカット 1（赤）、外形はカット 2（青）。線幅 0.1 mm、塗りなし。上から見た向き（SVG の y は下向き）
# - テンプレート: 遊舎工房の Laser_Poron_template.svg（デザインルールのページのリンクから。Inkscape 用）を TEMPLATE に置く
#   （リポジトリには入れない）。注文するサイズのレイヤーだけを残し、サイズの文字（黒の塗り = 彫刻になる）と注意書きを消す
# - 実行: uv run python foam.py [--show]   （kicad-cli と、pcbnew の使える /usr/bin/python3 が必要。out/ に入稿用の SVG と
#   確認用の PNG を書き出す。--show でケース・基板のモックと組み付けた状態を表示）
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from build123d import Face, GeomType, Pos, Vector, Wire, extrude
from shapely import affinity
from shapely.geometry import MultiPolygon, Point, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

import bottom as B
import layout as L
import matrix_mock as M
import plate as P

ROOT = Path(__file__).resolve().parent
PCBS = ROOT.parent / "pcbs"
TEMPLATE = ROOT / "vendor" / "Laser_Poron_template.svg"

FLOOR_FOAM_T = M.CASE_FOAM_T   # 2.0
PLATE_FOAM_T = 3.0             # プレートと基板の間 3.4 mm に、つぶさずに入れる
GASKET_T = 3.0                 # 上下 2 枚とプレートで 7.6 → 6.5 mm に挟む（1 枚 0.55、約 18% つぶれる）

OUTLINE_INSET = 0.2            # 床のフォームの左右の辺: マトリクス基板の外形から内側へ
FLOOR_FOAM_GROW = 0.4          # 床のフォームの奥と手前の辺: プレートの外形から外へ
EDGE_MERGE = 0.75              # 切り抜きと外周の間が 2 × EDGE_MERGE より狭いところは、外へ開く
PART_MARGIN = 0.5              # 基板の裏の部品の外形（ソケットは matrix_mock の形、ほかはコートヤード）から外へ
HOLE_MARGIN = 0.5              # スイッチの軸と突起の穴の半径に足す量
MIN_HOLE_D = 1.5               # これより小さい穴（ビア）は切り抜かない
CASE_MARGIN = 0.5              # 床から出ているものの外形から外へ
MERGE = 1.0                    # 同じキーの切り抜きどうし、切り抜きと外周の間が 2 × MERGE より狭いところは、つなげる
MIN_WEB = 0.4                  # 残るフォームのうち、幅 2 × MIN_WEB より細いところは切り落とす
MAX_SCRAP = 100.0              # 捨てる切れ端（いちばん大きな 1 枚以外。カットしないのでシートの残りの側に残る）の面積の上限 [mm^2]
PLATE_HOLE = 14.0              # プレートのフォームのスイッチの穴（プレートと同じ）
PLATE_FOAM_GROW = 0.4          # プレートのフォームの外形: プレートの外形（タブを除く）から外へ
# プレートのフォームの外形が、壁の内面、ネジのボス（どちらもプレートから L.PLATE_GAP 外）、ガスケットから 0.2 mm 以上離れること
assert PLATE_FOAM_GROW + 0.2 <= L.PLATE_GAP and L.SCREW_FROM_PLATE - L.BOSS_R >= L.PLATE_GAP

BOARDS = {"l": ("matrix-left", (50.0, 50.0), 1), "r": ("matrix-right", (185.0, 50.0), -1)}   # (基板, 原点, x の向き)

K = 1543.3132 / 500            # テンプレートの単位（1 mm あたり）
SHEET_MARGIN = 3.0             # テンプレートの端から 3 mm にはパスを置かない
PART_GAP = 4.0                 # 部品どうしの間（ルールは 2 mm 以上）
GASKET_GAP = 2.5               # ガスケットどうしの間（貼る所が同じもの）
FOAM_SETS = 3                  # フォームの枚数: 床のフォームは左右それぞれ、プレートのフォーム（左右共用）は 2 倍
GASKET_SETS = 6                # ガスケットの枚数: 案ごと、貼る所ごと
# ガスケットの案（名前, 内側の辺: プレートの外形から外へ [mm]）。組み立てるときにどちらかを選ぶ。上限はケースの内面（L.PLATE_GAP）
# 目印つきの案は、まとまりの右上に目印のダミーのパーツ（直径 MARKER_D の丸）を置く（台紙に付いたまま届くので、目印のある側が
# その案とわかる。片方だけに置けば、シートの向きによらない）
GASKET_VARIANTS = [("recommended", 0.8, False), ("max", 0.7, True)]
VARIANT_GAP = 15.0             # ガスケットの案どうしの間
MARKER_D = 10.0                # 目印の丸の直径
MARKER_GAP = 5.0               # 目印とガスケットのまとまりの間
GROUP_GAP = 6.0                # ガスケットの、貼る所ごとのまとまりの間
CUT1, CUT2 = "#ff0000", "#0000ff"
STROKE_W = 0.1                 # 線幅 [mm]（ルールは 0.001 mm か 0.1 mm）
CHORD = 0.2                    # 曲線を折れ線にするときの 1 本の長さの上限 [mm]
QUAD_SEGS = 16                 # 円弧（オフセットの角や円）を 90° あたり何本の線にするか
SIMPLIFY = 0.01                # 床のフォームの輪郭の点を、この誤差 [mm] の範囲で間引く


# ---- build123d ↔ shapely ----

def wire_points(w):
    pts = []
    for e in w.order_edges():
        n = 1 if e.geom_type == GeomType.LINE else max(2, math.ceil(e.length / CHORD))
        pts += [e @ (i / n) for i in range(n)]
    return [(p.X, p.Y) for p in pts]


def to_poly(face):
    """build123d の Face（複数でも）→ shapely。"""
    polys = [Polygon(wire_points(f.outer_wire()), [wire_points(w) for w in f.inner_wires()]) for f in face.faces()]
    return unary_union(polys)


def to_face(poly):
    """shapely の Polygon → build123d の Face（法線は +Z）。"""
    poly = orient(poly, sign=1.0)   # 外周を反時計回りに（時計回りだと面の法線が下を向き、押し出しが下向きになる）

    def wire(ring):
        return Wire.make_polygon([Vector(x, y) for x, y in list(ring.coords)[:-1]], close=True)
    face = Face(wire(poly.exterior), [wire(r) for r in poly.interiors])
    assert face.normal_at().Z > 0.999
    return face


def grow(geom, d):
    return geom.buffer(d, quad_segs=QUAD_SEGS)


def mirror_x(geom):
    return affinity.scale(geom, xfact=-1, yfact=1, origin=(0, 0))


def polygons(geom):
    return list(geom.geoms) if isinstance(geom, MultiPolygon) else [geom]


def floor_outline(sx):
    """床のフォームの外形。奥と手前は壁の内面（プレートから L.PLATE_GAP 外）の手前まで、左右は基板の外形まで（ボトムケースの
    左右には壁がなく、床の端はプレートの辺の位置）。奥のネジのまわりの台は、床から出ているものとして切り抜く。"""
    poly = to_poly(P.base(FLOOR_FOAM_GROW, M.MATRIX_SIDE_GROW - OUTLINE_INSET))
    return poly if sx > 0 else mirror_x(poly)


# ---- 基板の裏 ----

_COURTYARDS = r"""
import json, sys
import pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
out = []
for fp in b.GetFootprints():
    if not fp.IsFlipped():
        continue
    ps = fp.GetCourtyard(pcbnew.B_CrtYd)
    for i in range(ps.OutlineCount()):
        o = ps.Outline(i)
        out.append([fp.GetReference(), [pcbnew.ToMM(o.CPoint(j)) for j in range(o.PointCount())]])
print(json.dumps(out))
"""


def courtyards(pcb, origin):
    """裏面の部品（ダイオード、J4）のコートヤード: [(参照, Polygon)]。座標は左右それぞれの手の平面（右手は x が負）。"""
    res = subprocess.run(["/usr/bin/python3", "-c", _COURTYARDS, str(pcb)], check=True, capture_output=True, text=True)
    return [(ref, Polygon([(x - origin[0], origin[1] - y) for x, y in pts]))
            for ref, pts in json.loads(res.stdout.strip().splitlines()[-1])]


def socket_poly(x, y, angle):
    """ホットスワップソケットの大まかな外形（matrix_mock と同じ。本体と両端の端子）。"""
    shape = unary_union([box(*r) for r in M.SOCKET_BODY + M.SOCKET_TABS])
    return affinity.translate(affinity.rotate(shape, angle, origin=(0, 0)), x, y)


def board_cuts(hand):
    """基板の裏で避けるもの（左右それぞれの手の平面）。"""
    name, origin, sx = BOARDS[hand]
    pcb = PCBS / name / f"{name}.kicad_pcb"
    M.KICAD_OFFSET = origin
    holes = M.kicad_holes(pcb)
    sw = M.switches(holes)
    # スイッチの中心が、プレートのスイッチ穴（右手は x を反転）と一致すること
    keys = [(sx * k.center.X, k.center.Y) for k in P.keys()]
    assert len(sw) == len(keys)
    for x, y, _ in sw:
        assert min(math.hypot(x - a, y - b) for a, b in keys) < 0.01, (hand, x, y)
    parts = courtyards(pcb, origin)
    refs = [r for r, _ in parts]   # ダイオード 25 個と、マトリクスのコネクタ（参照は J4）
    assert sum(r.startswith("D") for r in refs) == 25 and sum(r.startswith("J") for r in refs) == 1, refs
    # キーごとに、ソケット、スイッチの軸と突起の穴、いちばん近いダイオードをまとめ、間の細いところをつなぐ
    groups = [[grow(socket_poly(x, y, a), PART_MARGIN)] for x, y, a in sw]

    def nearest(px, py):
        return min(range(len(sw)), key=lambda i: math.hypot(sw[i][0] - px, sw[i][1] - py))

    for x, y, d in holes:
        if d >= MIN_HOLE_D:
            groups[nearest(x, y)].append(Point(x, y).buffer(d / 2 + HOLE_MARGIN, quad_segs=QUAD_SEGS))
    others = []
    for ref, poly in parts:
        if ref.startswith("D"):
            groups[nearest(poly.centroid.x, poly.centroid.y)].append(grow(poly, PART_MARGIN))
        else:
            others.append(grow(poly, PART_MARGIN))
    return [close(unary_union(g), MERGE) for g in groups] + others


# ---- ケースの床 ----

def case_cuts():
    """床で避けるもの（左手の平面）: 収納部の天井の開口と、床の上面より上に出ているもの。"""
    import bay
    cuts = [to_poly(f) for f in bay.floor_openings()]
    # 床の上面より上でフォームの範囲に入るもの（奥のネジのまわりの台）。台の中のネジ穴は埋める
    z0 = B.FLOOR_Z + 0.05
    slab = Pos(0, 0, z0) * extrude(to_face(floor_outline(1)), FLOOR_FOAM_T + 0.3)
    inter = B.build() & slab
    for f in inter.faces() if inter is not None else []:
        if f.geom_type == GeomType.PLANE and f.normal_at().Z < -0.999 and abs(f.center().Z - z0) < 1e-4:
            cuts.append(grow(Polygon(to_poly(f).exterior), CASE_MARGIN))
    return cuts


# ---- フォーム ----

def close(geom, r):
    """幅 2 × r より狭いすき間を埋める。"""
    return grow(grow(geom, r), -r)


def clean_foam(outline, groups):
    """外形から切り抜きを引く。groups は切り抜きのまとまり（キーごと、J4 と収納部の天井の開口）。まとまりのうち、外周との間が
    2 × EDGE_MERGE より狭いものは外へ開く（まとまりどうしはつながない）。幅 2 × MIN_WEB より細いところは落とす。
    いちばん大きな 1 枚と、捨てる切れ端を返す。"""
    outside = box(*grow(outline, 20).bounds).difference(outline)
    cuts = []
    for g in groups:
        if g.distance(outside) < 2 * EDGE_MERGE:
            g = unary_union([g, close(unary_union([g, outside]), EDGE_MERGE).intersection(outline)])
        cuts.append(g)
    foam = outline.difference(unary_union(cuts))
    foam = grow(grow(foam, -MIN_WEB), MIN_WEB).intersection(outline)
    parts = sorted(polygons(foam), key=lambda p: p.area, reverse=True)
    return parts[0].simplify(SIMPLIFY, preserve_topology=True), parts[1:]


def floor_groups(hand, case):
    """床のフォームの切り抜きのまとまり（左右それぞれの手の平面）。"""
    _, _, sx = BOARDS[hand]
    case = case if sx > 0 else [mirror_x(c) for c in case]
    keys_and_j4 = board_cuts(hand)
    # J4 と収納部の天井の開口は重なるので 1 つのまとまりに
    j4 = keys_and_j4[-1]
    groups = keys_and_j4[:-1] + [close(unary_union([j4] + [c for c in case if c.intersects(j4)]), MERGE)]
    return groups + [c for c in case if not c.intersects(j4)]


def floor_foams(case):
    """床のフォーム: {"l": 左手, "r": 右手}、それぞれ (Polygon, 切れ端)。右手の基板はダイオードの向きを反転していないので、
    左右で切り抜きが違う（裏返しても使えない）。"""
    return {hand: clean_foam(floor_outline(sx), floor_groups(hand, case)) for hand, (_, _, sx) in BOARDS.items()}


def plate_foam():
    """プレートのフォーム（左手の向き。右手には裏返して使う。プレートと同じく、裏返すと x を反転した右手の形になる）。
    この高さ（基板の上面から上）では、奥と手前の壁の内面はプレートから L.PLATE_GAP 外、ガスケットはプレートから
    gasket.INNER_CLEARANCE + L.PLATE_GAP 外、ネジのボスの縁もプレートから L.PLATE_GAP 外にあるので、外形をプレートの外形の
    PLATE_FOAM_GROW 外まで広げる（穴と外周の間を広げる）。穴は外へ開かない。"""
    h = PLATE_HOLE / 2
    holes = [affinity.translate(affinity.rotate(box(-h, -h, h, h), k.angle, origin=(0, 0)), k.center.X, k.center.Y)
             for k in P.keys()]
    foam = to_poly(P.base(PLATE_FOAM_GROW)).difference(unary_union(holes))
    assert isinstance(foam, Polygon) and len(foam.interiors) == len(holes)
    return foam


def gasket_footprint(tab, inner):
    """入稿するガスケットの平面形（組み付けた位置）: タブのうち、プレートの外形から inner より外の部分（先端と側面はタブに
    そろえ、先端の角もタブの丸みのまま）。モックのガスケット（gasket.footprint）より広い: 上のガスケットの内側の辺が
    ケースの内面（L.PLATE_GAP。ここから内側はベゼルの下で、プレートの上面との間が 0.5 mm しかない）を越えなければよい。
    タブに貼るのでプレートと一緒に動き、レーザーで切ると焼けて無くなる幅の分だけタブより小さくなる。"""
    tab_shape = to_poly(P.outline()).difference(to_poly(P.base(inner)))
    shape = tab_shape.intersection(to_poly(L.tab_strip(tab, tab.width / 2)))
    return max(polygons(shape), key=lambda q: q.area)


def gaskets(inner):
    """[(タブの名前, 貼る所, Polygon)]。貼る所は L-upper、L-lower、R-upper、R-lower。組み付けたときの角度のまま、上から見て
    シールが裏に付く向き。"""
    names = {P.I_BACK: "back", P.I_FRONT_LEFT: "front-left", P.I_THUMB: "thumb"}
    out = []
    for tab in P.tabs():
        f = gasket_footprint(tab, inner)
        n = f"{names.get(tab.edge, tab.edge)}-x{tab.cx:.0f}"
        out += [(n, "L-upper", f), (n, "L-lower", mirror_x(f)), (n, "R-upper", mirror_x(f)), (n, "R-lower", f)]
    return out


def stack_pitch(poly, gap):
    """同じ形を縦に重ねるときの、間が gap 以上になる最小の間隔 [mm]（ななめの帯は外接矩形の高さより詰められる）。"""
    lo, hi = 0.0, height(poly) + gap
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if poly.distance(affinity.translate(poly, 0, mid)) < gap else (lo, mid)
    return hi + 0.01


def min_web(poly):
    """フォームのいちばん細いところ（穴どうし、穴と外周の距離の最小）。"""
    rings = [poly.exterior] + list(poly.interiors)
    return min(a.distance(b) for i, a in enumerate(rings) for b in rings[i + 1:])


# ---- SVG ----

def ring_d(ring, ox, oy, k=K):
    """平面の輪郭（y は上向き）→ パス（y は下向き）。(ox, oy) は平面の (0, 0) を置く位置 [mm]（oy は上端からの距離）。"""
    pts = list(ring.coords)[:-1]
    return "M " + " L ".join(f"{(ox + x) * k:.4f},{(oy - y) * k:.4f}" for x, y in pts) + " Z"


def placed_paths(poly, x, y):
    """Polygon の外接矩形の左上を (x, y) [mm] に置いたパス: [(色, d)]。外形はカット 2、穴はカット 1。"""
    x0, _, _, y1 = poly.bounds
    ox, oy = x - x0, y + y1
    return [(CUT2, ring_d(poly.exterior, ox, oy))] + [(CUT1, ring_d(r, ox, oy)) for r in poly.interiors]


NS = {"svg": "http://www.w3.org/2000/svg", "inkscape": "http://www.inkscape.org/namespaces/inkscape",
      "sodipodi": "http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"}
for _p, _u in NS.items():
    ET.register_namespace("" if _p == "svg" else _p, _u)


def write_sheet(path, layer_label, paths, guides=()):
    """テンプレートの layer_label のレイヤーだけを残し、パスを入れて書き出す。guides は水色の案内の長方形 [(x, y, w, h)] [mm]。"""
    tree = ET.parse(TEMPLATE)
    root = tree.getroot()
    label = f"{{{NS['inkscape']}}}label"
    layer = None
    for g in list(root.findall("svg:g", NS)):
        if g.get(label) == layer_label:
            layer = g
        else:
            root.remove(g)
    assert layer is not None, layer_label
    for g in list(layer.findall("svg:g", NS)):   # サイズの文字（黒の塗り）
        layer.remove(g)
    for i, (x, y, w, h) in enumerate(guides):
        ET.SubElement(layer, f"{{{NS['svg']}}}rect", {
            "id": f"guide{i}", "x": f"{x * K:.4f}", "y": f"{y * K:.4f}", "width": f"{w * K:.4f}", "height": f"{h * K:.4f}",
            "style": "fill:none;stroke:#00ffff;stroke-width:0.5"})
    for i, (color, d) in enumerate(paths):
        ET.SubElement(layer, f"{{{NS['svg']}}}path", {
            "id": f"cut{i}", "d": d, "style": f"fill:none;stroke:{color};stroke-width:{STROKE_W * K:.4f}"})
    tree.write(path, xml_declaration=True, encoding="UTF-8")


def layout_row(polys, x0, y0):
    """左から順に並べる: [(Polygon, x, y)]（外接矩形の左上の位置 [mm]）。"""
    out, x = [], x0
    for p in polys:
        bx0, by0, bx1, by1 = p.bounds
        out.append((p, x, y0))
        x += bx1 - bx0 + PART_GAP
    return out


def check_layout(placed, sheet_w, sheet_h, area=None):
    """部品がシート（または area = (x, y, w, h) の中）の端から SHEET_MARGIN 以上内側にあり、互いに 2 mm 以上離れていること。"""
    ax, ay, aw, ah = area or (0, 0, sheet_w, sheet_h)
    moved = []
    for p, x, y in placed:
        x0, _, _, y1 = p.bounds
        q = affinity.scale(affinity.translate(p, -x0, -y1), 1, -1, origin=(0, 0))   # シートの座標（y は下向き）
        q = affinity.translate(q, x, y)
        bx0, by0, bx1, by1 = q.bounds
        assert bx0 >= ax + SHEET_MARGIN and by0 >= ay + SHEET_MARGIN, (bx0, by0)
        assert bx1 <= ax + aw - SHEET_MARGIN and by1 <= ay + ah - SHEET_MARGIN, (bx1, by1)
        moved.append(q)
    for i, a in enumerate(moved):
        for b in moved[i + 1:]:
            assert a.distance(b) >= 2.0
    return moved


def sheet(path, layer_label, size, placed, area=None):
    check_layout(placed, *size, area=area)
    paths = [pd for p, x, y in placed for pd in placed_paths(p, x, y)]
    write_sheet(path, layer_label, paths, guides=[area] if area else [])


def preview_png(png_path, size, placed, labels, area=None, regions=()):
    """確認用の PNG（入稿用ではない）: 部品を塗り、名前を書く。regions は [(名前, x, y, w, h)] の囲み。"""
    w, h = size
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w * 4}" height="{h * 4}" viewBox="0 0 {w} {h}">',
             f'<rect width="{w}" height="{h}" fill="white" stroke="#888" stroke-width="0.3"/>']
    if area:
        parts.append(f'<rect x="{area[0]}" y="{area[1]}" width="{area[2]}" height="{area[3]}" fill="none" stroke="#0aa" '
                     f'stroke-width="0.3" stroke-dasharray="2 1"/>')
    for text, rx, ry, rw, rh in regions:
        parts.append(f'<rect x="{rx - 2}" y="{ry - 2}" width="{rw + 4}" height="{rh + 4}" fill="none" stroke="#36c" '
                     f'stroke-width="0.4"/>')
        parts.append(f'<text x="{rx + rw / 2:.2f}" y="{ry + rh + 7:.2f}" font-size="5" text-anchor="middle" '
                     f'font-family="sans-serif" fill="#36c">{text}</text>')
    for (p, x, y), text in zip(placed, labels):
        x0, y0, x1, y1 = p.bounds
        d = " ".join(ring_d(r, x - x0, y + y1, k=1) for r in [p.exterior] + list(p.interiors))
        parts.append(f'<path d="{d}" fill="#f2c94c" fill-rule="evenodd" stroke="#333" stroke-width="0.15"/>')
        size_ = min(5, (y1 - y0) * 0.8, (x1 - x0) / 8)
        parts.append(f'<text x="{x + (x1 - x0) / 2:.2f}" y="{y + (y1 - y0) / 2 + size_ * 0.35:.2f}" font-size="{size_:.2f}" '
                     f'text-anchor="middle" font-family="sans-serif" fill="#a33">{text}</text>')
    parts.append("</svg>")
    tmp = png_path.with_suffix(".preview.svg")
    tmp.write_text("\n".join(parts))
    subprocess.run(["convert", "-density", "100", str(tmp), str(png_path)], check=True)
    tmp.unlink()


def cut_length(placed):
    """カットのパスの長さの合計 [m]。"""
    return sum(r.length for p, _, _ in placed for r in [p.exterior] + list(p.interiors)) / 1000


def width(p):
    return p.bounds[2] - p.bounds[0]


def height(p):
    return p.bounds[3] - p.bounds[1]


def grid(polys, cols, x0, y0):
    """外接矩形の大きさをそろえた格子に、左上から横へ並べる: [(Polygon, x, y)]。"""
    w = max(width(p) for p in polys)
    h = max(height(p) for p in polys)
    return [(p, x0 + (i % cols) * (w + PART_GAP), y0 + (i // cols) * (h + PART_GAP)) for i, p in enumerate(polys)]


def main():
    out = ROOT / "out"
    out.mkdir(exist_ok=True)
    floor = floor_foams(case_cuts())
    for hand, (f, scraps) in floor.items():
        print(f"floor foam {hand}: {width(f):.2f} x {height(f):.2f} mm, holes {len(f.interiors)}, "
              f"min web {min_web(f):.2f}, scraps {[round(s.area, 1) for s in scraps]}")
        assert all(s.area <= MAX_SCRAP for s in scraps)
    plate = plate_foam()
    print(f"plate foam: {width(plate):.2f} x {height(plate):.2f} mm, holes {len(plate.interiors)}, min web {min_web(plate):.2f}")
    x0 = y0 = SHEET_MARGIN + 1.0

    # 床のフォーム（2 mm、500 × 200）: 上の段に左手、下の段に右手を FOAM_SETS 枚ずつ
    placed = grid([floor["l"][0]] * FOAM_SETS + [floor["r"][0]] * FOAM_SETS, FOAM_SETS, x0, y0)
    sheet(out / "foam-floor-2mm.svg", "500x200", (500, 200), placed)
    preview_png(out / "foam-floor-2mm.png", (500, 200), placed, ["L"] * FOAM_SETS + ["R"] * FOAM_SETS)
    print(f"foam-floor-2mm: {len(placed)} pieces, cut {cut_length(placed):.2f} m")

    # プレートのフォーム（3 mm、500 × 200）: 左右共用を 2 × FOAM_SETS 枚
    placed = grid([plate] * (2 * FOAM_SETS), FOAM_SETS, x0, y0)
    sheet(out / "foam-plate-3mm.svg", "500x200", (500, 200), placed)
    preview_png(out / "foam-plate-3mm.png", (500, 200), placed, ["L/R"] * len(placed))
    print(f"foam-plate-3mm: {len(placed)} pieces, cut {cut_length(placed):.2f} m")

    # ガスケット（3 mm、接着シート付き）: 500 × 300 のテンプレートに 297 × 190 の長方形を置き、その中に並べる。
    # 内側の辺の違う 2 つの案（GASKET_VARIANTS）を、左右に VARIANT_GAP あけて並べる。それぞれの案の中では、貼る所
    # （タブ × L/R × upper/lower）ごとに GASKET_SETS 枚を、組み付けたときの角度のまま縦に重ねる。1 行が 1 つのタブ
    # （L-upper、L-lower、R-upper、R-lower の順）。まとまりの間は GROUP_GAP
    area = (SHEET_MARGIN + 2.0, SHEET_MARGIN + 2.0, 297.0, 190.0)
    x, y0 = area[0] + SHEET_MARGIN + 1.0, area[1] + SHEET_MARGIN + 1.0
    placed, labels, regions = [], [], []
    for vname, inner, marker in GASKET_VARIANTS:
        gs = gaskets(inner)
        tabs = list(dict.fromkeys(t for t, _, _ in gs))
        spots = list(dict.fromkeys(s for _, s, _ in gs))
        gw = max(width(p) for _, _, p in gs)
        y = y0
        for tab in tabs:
            polys = [p for t, _, p in gs if t == tab]
            pitch = max(stack_pitch(p, GASKET_GAP) for p in polys)
            for c, spot in enumerate(spots):
                p = next(q for t, s, q in gs if t == tab and s == spot)
                for i in range(GASKET_SETS):
                    placed.append((p, x + c * (gw + GROUP_GAP), y + i * pitch))
                    labels.append(f"{tab} {spot}" if i == 0 else "")
            y += max(height(p) for p in polys) + (GASKET_SETS - 1) * pitch + GROUP_GAP
        w = len(spots) * gw + (len(spots) - 1) * GROUP_GAP
        regions.append((vname, inner, x, y0, w, y - GROUP_GAP - y0))
        if marker:
            placed.append((Point(0, 0).buffer(MARKER_D / 2, quad_segs=QUAD_SEGS), x + w + MARKER_GAP, y0))
            labels.append("mark")
        print(f"gasket {vname} (inner +{inner}): {len(gs)} spots x {GASKET_SETS}, "
              + ", ".join(f"{t} {s} {width(p):.2f} x {height(p):.2f} mm, {p.area:.1f} mm2" for t, s, p in gs if s == "L-upper"))
        x += w + VARIANT_GAP
    sheet(out / "gasket-3mm-adhesive.svg", "500x300", (500, 300), placed, area=area)
    preview_png(out / "gasket-3mm-adhesive.png", (310, 200), placed, labels, area=area,
                regions=[(f"{v}: inner +{i} mm", rx, ry, rw, rh) for v, i, rx, ry, rw, rh in regions])
    print(f"gasket-3mm-adhesive: {len(placed)} pieces, cut {cut_length(placed):.2f} m")
    return floor, plate


if __name__ == "__main__":
    floor, plate = main()
    if "--show" in sys.argv:
        from ocp_vscode import show
        board, _ = M.matrix_mock()
        show(B.build(), Pos(0, 0, M.PCB_TOP_Z) * board,
             Pos(0, 0, B.FLOOR_Z) * extrude(to_face(floor["l"][0]), FLOOR_FOAM_T),
             Pos(0, 0, M.PCB_TOP_Z) * extrude(to_face(plate), PLATE_FOAM_T),
             names=["bottom-l", "matrix-mock", "foam-floor-l", "foam-plate-l"],
             colors=["#5b8fd6", "#2f7d4f", "#f2c94c", "#e07b39"], alphas=[0.4, 0.9, 1, 1])
