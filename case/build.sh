#!/usr/bin/env bash
# out/ を消して、印刷する部品（out/print/）と発注データ（out/order/）をすべて書き出し直す。
# - 必要なもの: uv、kicad-cli と pcbnew の使える /usr/bin/python3（KiCad。matrix_mock.py、foam.py、jig.py が使う）、
#   vendor/Laser_Poron_template.svg（foam.py。入手先は foam.py の先頭）
# - 実行: ./build.sh
set -euo pipefail
cd "$(dirname "$0")"

rm -rf out
for script in bottom top bay reset_pin palmrest jig plate matrix_mock foam; do
    echo "== $script.py"
    uv run python "$script.py"
done
