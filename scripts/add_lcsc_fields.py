# 回路図の部品に、JLCPCB の部品実装（PCBA）で使う LCSC の部品番号のフィールド（LCSC、非表示）を入れる。
# 番号はフットプリントで決める（LCSC_BY_FOOTPRINT）。すでに LCSC のフィールドがある部品は値を書き換える。
# KiCad の外で実行する（回路図エディタで開いている回路図は書き換えない。ロックファイルがあれば止まる）:
#   python3 scripts/add_lcsc_fields.py              # 既定の回路図（DEFAULT_FILES）
#   python3 scripts/add_lcsc_fields.py a.kicad_sch  # 指定した回路図
# 回路図を書き換えたあと、PCB エディタで「回路図から PCB を更新」（F8）すると、フットプリントにも LCSC のフィールドが入る。

import re
import sys
from pathlib import Path

# ===== 設定 =====
# フットプリント: LCSC の部品番号（2026-10-23 に JLCPCB の部品ページで在庫を確かめたもの）
LCSC_BY_FOOTPRINT = {
    "kbd_SW:CherryMX_Hotswap_1u": "C41430893",   # HanElectricity CPG151101S11-16（Kailh の C2803348 は在庫切れ）
    "Diode_SMD:D_SOD-123": "C81598",             # 1N4148W（Basic 部品）
    "Connector_JST:JST_SH_SM11B-SRSS-TB_1x11-1MP_P1.00mm_Horizontal": "C515956",   # JST SM11B-SRSS-TB（在庫が少ない）
}
FIELD = "LCSC"
ROOT = Path(__file__).resolve().parent.parent
DEFAULT_FILES = [
    ROOT / "pcbs/matrix-left/matrix-left.kicad_sch",
    ROOT / "pcbs/matrix-right/matrix-right.kicad_sch",
    ROOT / "pcbs/root/matrix-left.kicad_sch",
    ROOT / "pcbs/root/matrix-right.kicad_sch",
]
# ================

FOOTPRINT_RE = re.compile(r'^\t\t\(property "Footprint" "([^"]*)"$')
FIELD_RE = re.compile(rf'^\t\t\(property "{FIELD}" "([^"]*)"$')


def field_block(value, at_line):
    """非表示のフィールドの行。位置はフットプリントのフィールドと同じ。"""
    return [
        f'\t\t(property "{FIELD}" "{value}"',
        at_line,
        "\t\t\t(hide yes)",
        "\t\t\t(show_name no)",
        "\t\t\t(do_not_autoplace no)",
        "\t\t\t(effects",
        "\t\t\t\t(font",
        "\t\t\t\t\t(size 1.27 1.27)",
        "\t\t\t\t)",
        "\t\t\t)",
        "\t\t)",
    ]


def process(path):
    lock = path.with_name("~" + path.name + ".lck")
    if lock.exists():
        raise SystemExit(f"{path}: 回路図エディタで開いている（{lock.name} がある）。閉じてから実行する")
    lines = path.read_text().split("\n")
    out, added, updated, unknown = [], 0, 0, set()
    i = 0
    while i < len(lines):
        line = lines[i]
        if line != "\t(symbol":   # 配置した部品（lib_symbols の中の定義ではない）
            out.append(line)
            i += 1
            continue
        # 部品のブロック（"\t)" まで）を取り出して処理する
        end = lines.index("\t)", i)
        block = lines[i:end + 1]
        i = end + 1
        fp_idx = next((k for k, l in enumerate(block) if FOOTPRINT_RE.match(l)), None)
        footprint = FOOTPRINT_RE.match(block[fp_idx]).group(1) if fp_idx is not None else ""
        value = LCSC_BY_FOOTPRINT.get(footprint)
        if value is None:
            if footprint:
                unknown.add(footprint)
            out.extend(block)
            continue
        f_idx = next((k for k, l in enumerate(block) if FIELD_RE.match(l)), None)
        if f_idx is not None:
            block[f_idx] = f'\t\t(property "{FIELD}" "{value}"'
            updated += 1
        else:
            fp_end = block.index("\t\t)", fp_idx)
            block[fp_end + 1:fp_end + 1] = field_block(value, block[fp_idx + 1])
            added += 1
        out.extend(block)
    path.write_text("\n".join(out))
    name = path.relative_to(ROOT) if path.is_relative_to(ROOT) else path
    print(f"{name}: {added} 個に追加、{updated} 個を更新"
          + (f"（番号のないフットプリント: {sorted(unknown)}）" if unknown else ""))


for p in [Path(a).resolve() for a in sys.argv[1:]] or DEFAULT_FILES:
    process(p)
