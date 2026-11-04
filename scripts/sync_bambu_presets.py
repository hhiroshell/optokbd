# Bambu Studio のユーザープリセット（プロセス・フィラメント・プリンタ）を、リポジトリの print/bambu/ に写す（履歴管理用）。
# - プリセットの JSON は、元にしたシステムのプリセット（inherits）から変えた項目だけを持つので、そのまま git の差分で読める
# - 写すのは *.json だけ。*.info（クラウド同期の情報: アカウント、日時）は写さない
# - Bambu Studio で消したプリセットは、print/bambu/ からも消す（鏡像にする）
# - Bambu Studio の設定の場所は、Flatpak 版（~/.var/app/com.bambulab.BambuStudio/config/BambuStudio）か、ふつうの版
#   （~/.config/BambuStudio）。環境変数 BAMBU_STUDIO_DIR で指定もできる。プリセットはその下の user/<アカウントの番号>/ にある
#   （ログインしていないときは user/default/。アカウントが複数あるときは --account で選ぶ）
# 実行:
#   python3 scripts/sync_bambu_presets.py            # 写して、変わったファイルを表示する
#   python3 scripts/sync_bambu_presets.py --dry-run  # 変わるファイルを表示するだけ
# 写したあと git diff print/bambu で中身を確かめてコミットする。

import argparse
import filecmp
import os
import shutil
import sys
from pathlib import Path

# ===== 設定 =====
CANDIDATES = [
    Path.home() / ".var/app/com.bambulab.BambuStudio/config/BambuStudio",   # Flatpak 版
    Path.home() / ".config/BambuStudio",
]
KINDS = ["process", "filament", "machine"]
ROOT = Path(__file__).resolve().parent.parent
DEST = ROOT / "print" / "bambu"


def studio_dir():
    """Bambu Studio の設定のディレクトリ。"""
    if env := os.environ.get("BAMBU_STUDIO_DIR"):
        return Path(env).expanduser()
    found = [d for d in CANDIDATES if (d / "user").is_dir()]
    if not found:
        sys.exit("Bambu Studio の設定が見つからない: " + ", ".join(str(d) for d in CANDIDATES))
    return found[0]


def user_dir(base, account=None):
    """プリセットのあるディレクトリ: user/<アカウントの番号>/。番号を指定しないときは、1 つだけあればそれ、
    なければ user/default/。"""
    if account:
        d = base / "user" / account
        if not d.is_dir():
            sys.exit(f"アカウントのディレクトリがない: {d}")
        return d
    accounts = [d for d in (base / "user").iterdir() if d.is_dir() and d.name.isdigit()]
    if len(accounts) > 1:
        sys.exit("アカウントが複数ある。--account で選ぶ: " + ", ".join(d.name for d in accounts))
    return accounts[0] if accounts else base / "user" / "default"


def main():
    parser = argparse.ArgumentParser(description="Bambu Studio のユーザープリセットを print/bambu/ に写す")
    parser.add_argument("--dry-run", action="store_true", help="変わるファイルを表示するだけで、写さない")
    parser.add_argument("--account", help="user/ の下のアカウントの番号（複数あるとき）")
    args = parser.parse_args()

    src = user_dir(studio_dir(), args.account)
    print(f"from: {src}")
    print(f"to:   {DEST.relative_to(ROOT)}")
    changes = 0
    for kind in KINDS:
        s, d = src / kind, DEST / kind
        srcs = {p.name: p for p in s.glob("*.json")} if s.is_dir() else {}
        dests = {p.name: p for p in d.glob("*.json")} if d.is_dir() else {}
        for name, p in sorted(srcs.items()):
            if name not in dests:
                mark = "A"
            elif not filecmp.cmp(p, dests[name], shallow=False):
                mark = "M"
            else:
                continue
            print(f"  {mark} {kind}/{name}")
            changes += 1
            if not args.dry_run:
                d.mkdir(parents=True, exist_ok=True)
                shutil.copy2(p, d / name)
        for name in sorted(set(dests) - set(srcs)):
            print(f"  D {kind}/{name}")
            changes += 1
            if not args.dry_run:
                dests[name].unlink()
    print("no changes" if changes == 0 else f"{changes} file(s) {'would change' if args.dry_run else 'changed'}")


if __name__ == "__main__":
    main()
