#!/usr/bin/env python3
"""run_init_linux.py -- 2026-08-18 臨時腳本，在這台 Linux 機器上直接呼叫
web/init_flow.py 的 run_init()（跟網頁「Initialize System」按鈕做的事
完全一樣），不透過 Flask/app.py。

用法：
  cd host && python3 run_init_linux.py
"""
import sys, json
sys.path.insert(0, "web")

from awg_common import open_board
from init_flow import run_init


def main():
    dev, fp = open_board()
    print(f"[master board opened]")

    def on_phase(entry):
        status = "OK" if entry["ok"] else "FAIL"
        print(f"  [{entry['name']}] {status}  {entry.get('detail', '')}  {entry.get('error', '')}")

    result = run_init(fp, on_phase=on_phase)
    print("\n=== 結果 ===")
    print(f"ok={result['ok']}  total_boards={result['total_boards']}")
    for dest_id, b in result.get("boards", {}).items():
        print(f"  dest_id={dest_id}: {b}")


if __name__ == "__main__":
    main()
