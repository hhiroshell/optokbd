# DIY Keyboard "Opto" ⌨

XIAO nRF52840 と ZMK で動く、左右分割のワイヤレスキーボード（片手 25 キー、7 列、MX ホットスワップ、ガスケットマウント）。
ケースは [build123d](https://github.com/gumyr/build123d) で、基板は KiCad 10 で設計している。

- 今の版は v0.2.0。左手は組み上げて動作を確かめた。右手のケースは左手を左右反転して作る。設計の決めごとと部品は
  [docs/v0.2.0.md](docs/v0.2.0.md)
- v0.1.0（FreeCAD のケースと有線の基板）はタグ `v0.1.0` にある
- ファームウェアは [hhiroshell/zmk-config](https://github.com/hhiroshell/zmk-config)

## ディレクトリ

| ディレクトリ | 中身 |
|---|---|
| `case/` | ケースとその周りの 3D モデル（build123d、uv のプロジェクト）。トップケース、ボトムケース、底蓋と収納部、パームレスト、はんだ付けの治具。フォームとガスケットのカットデータ、基板とプレートの外形もここで作る |
| `pcbs/` | KiCad のプロジェクト。`matrix-left`、`matrix-right`（マトリクス基板）、`controller`（コントローラー基板）、`plate`（FR-4 のスイッチプレート）、`root`（全体の回路図） |
| `plate/` | スイッチプレートの DXF（ケースの平面形の基準）。`plate_left.dxf` を使う |
| `layout/` | キー配列（Keyboard Layout Editor の JSON） |
| `lib/` | KiCad のフットプリント（XIAO nRF52840） |
| `scripts/` | KiCad のスクリプト（部品の配置、外形、シルク、JLCPCB の発注データ）と、Bambu Studio のプリセットの同期 |
| `print/bambu/` | Bambu Studio のプリセット（A1、0.2 mm ノズル） |
| `docs/` | 設計メモ |

## ケースのモデルを作る

[uv](https://docs.astral.sh/uv/) と KiCad（`kicad-cli` と、`pcbnew` の使える `/usr/bin/python3`）が必要。

```bash
cd case
./build.sh                          # out/ を消して、すべて作り直す
uv run python top.py --show         # 1 つだけ作り、OCP CAD Viewer に表示する
```

- `case/out/print/`: 印刷する部品（STEP / STL / 3MF）。ケース、底蓋、収納部の天井、パームレストは、左手（`-l`）と、それを左右に
  反転した右手（`-r`）を書き出す
- `case/out/order/`: 発注データ（遊舎工房に入稿するフォームとガスケットの SVG）と、KiCad の Edge.Cuts に入れる外形

フォームのカットデータ（`foam.py`）には、遊舎工房のテンプレート `Laser_Poron_template.svg` が要る。リポジトリには入れていないので、
[デザインルールのページ](https://yushakobo.jp/lasercut/)のリンクから入手して `case/vendor/` に置く。
