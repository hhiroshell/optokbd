# place_matrix.py と同じ座標表から、プレート用の DXF を作る。
# - DXF の原点 (0,0) = レイアウト座標の原点（place_matrix.py の ORIGIN に置いた点）
# - DXF は Y 軸が上向きなので、レイアウトの y を反転して書き出す
# レイヤー:
#   CUTOUT    : 14.0mm 角のスイッチ穴（プレートのカット線）
#   KEY_1U    : 19.05mm 角のキー外枠（参考）
#   KEYCAP_18 : 18mm 角のキーキャップ外形（参考）
#   CENTER    : スイッチ中心の十字（参考）
import math, re, sys
import ezdxf

src = open(sys.argv[1] if len(sys.argv) > 1 else "place_matrix.py").read()
KEYS = {}
for m in re.finditer(r"\((\d), (\d)\): \(([\d.]+), ([\d.]+), ([\d.]+)\)", src):
    KEYS[(int(m[1]), int(m[2]))] = (float(m[3]), float(m[4]), float(m[5]))
assert len(KEYS) == 25, len(KEYS)

def square(cx, cy, r_cw, size):
    h = size / 2
    a = math.radians(r_cw)
    pts = []
    for px, py in [(-h, -h), (h, -h), (h, h), (-h, h)]:
        x = cx + px * math.cos(a) - py * math.sin(a)
        y = cy + px * math.sin(a) + py * math.cos(a)
        pts.append((x, -y))  # y 反転
    return pts

def add_square(msp, pts, layer):
    # LibreCAD の平行線（オフセット）ツールはポリラインを対象にできないため、4本の LINE で描く
    for i in range(4):
        msp.add_line(pts[i], pts[(i + 1) % 4], dxfattribs={"layer": layer})

def build(mirror, path):
    doc = ezdxf.new("R2010", setup=False)
    doc.units = ezdxf.units.MM
    doc.header["$INSUNITS"] = 4
    for name, color in [("CUTOUT", 7), ("KEY_1U", 8), ("KEYCAP_18", 9), ("CENTER", 1)]:
        doc.layers.add(name, color=color)
    msp = doc.modelspace()
    for (row, col), (x, y, r) in sorted(KEYS.items()):
        if mirror:
            x, r = -x, -r
        add_square(msp, square(x, y, r, 14.0), "CUTOUT")
        add_square(msp, square(x, y, r, 19.05), "KEY_1U")
        add_square(msp, square(x, y, r, 18.0), "KEYCAP_18")
        msp.add_line((x - 1, -y), (x + 1, -y), dxfattribs={"layer": "CENTER"})
        msp.add_line((x, -y - 1), (x, -y + 1), dxfattribs={"layer": "CENTER"})
    doc.saveas(path)

build(False, "plate_left.dxf")
build(True, "plate_right.dxf")
print("done")
