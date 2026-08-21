#!/usr/bin/env python3
"""
si5332_configure.py — 設定 Si5332-6EX-EVB 輸出時脈（分階段寫入，避開已知的 I2C lockup bug）

背景（詳見 PROJECT.md「根本原因已定位」一節，2026-07-13）：
用 `CBProDeviceWrite --project <file>` 一次性寫入（Ready → 設定暫存器 → Active
全部在同一個 process/USB session 裡連續完成）會讓裝置在寫入完成、剛進入 ACTIVE
狀態之後，緊接著的下一個 I2C 操作必定失敗（`0xFA general failure`），且會讓
RAM 內的設定被清空回到 blank 狀態（因為 Si5332 是 RAM-based 配置，斷電/bus
lockup 都會讓設定消失）。已用對照測試確認：跟等待時間無關，純粹是「同一個
session 連續做完全部步驟」才會觸發；改成三次完全獨立的 process 呼叫（各自開
關 USB/I2C 連線），即使中間零延遲緊接著執行也完全正常。

本腳本把 CBProProjectRegistersExport 匯出的暫存器腳本（含 pre/post-amble）自動
拆成 Ready / 設定 / Active 三段，各自用獨立的 CBProDeviceWrite.exe process 呼叫
寫入，重現這個已驗證可行的流程。

用法：
  python si5332_configure.py --project <path>.slabtimeproj
      先用 CBProProjectRegistersExport 匯出暫存器腳本，再分階段寫入

  python si5332_configure.py --registers <path>.csv
      直接使用已經匯出好的暫存器腳本（CBProProjectRegistersExport
      --include-load-writes --format csv 產生的檔案）

前提：CBPro GUI（ClockBuilder Pro.exe）必須先關閉，否則 CLI 會回報
"Failed to create an IPC Port: 存取被拒" ——這是資源被 GUI 佔用，不是硬體問題。
"""
import sys
import subprocess
import argparse
import tempfile
import os

from si5332_registers import split_stages

CBPRO_BIN = r"C:\Program Files (x86)\Skyworks\ClockBuilder Pro\Bin"
CBPRO_EXPORT = os.path.join(CBPRO_BIN, "CBProProjectRegistersExport.exe")
CBPRO_WRITE = os.path.join(CBPRO_BIN, "CBProDeviceWrite.exe")
CBPRO_READ = os.path.join(CBPRO_BIN, "CBProDeviceRead.exe")

USYS_STAT_ACTIVE = 0x02


def export_registers(project_path: str) -> str:
    """用 CBProProjectRegistersExport 把專案檔匯出成暫存器腳本，回傳暫存檔路徑"""
    out_fd, out_path = tempfile.mkstemp(suffix=".csv", prefix="si5332_regs_")
    os.close(out_fd)
    print(f"Exporting registers from project: {project_path}")
    result = subprocess.run(
        [CBPRO_EXPORT, "--project", project_path,
         "--include-load-writes", "--format", "csv", "--outfile", out_path],
        capture_output=True, text=True
    )
    print(result.stdout)
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr)
        sys.exit(f"[X] Export failed (exit {result.returncode})")
    return out_path


def read_back(addresses):
    """讀回一批 register 目前的實際值，回傳 {int(addr): int(value)}"""
    result = subprocess.run(
        [CBPRO_READ, "--quiet", "--format", "csv", "--registers"] + addresses,
        capture_output=True, text=True
    )
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
        sys.exit(f"[X] CBProDeviceRead failed (exit {result.returncode})")

    values = {}
    for line in result.stdout.strip().splitlines():
        if line.startswith("Address") or not line.strip():
            continue
        parts = line.split(",")
        if len(parts) < 3:
            continue
        addr, hexval = parts[0].strip(), parts[2].strip()
        values[int(addr, 16)] = int(hexval, 16)
    return values


def read_usys_stat() -> int:
    result = subprocess.run(
        [CBPRO_READ, "--quiet", "--format", "csv", "--settings", "USYS_STAT"],
        capture_output=True, text=True
    )
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
        sys.exit(f"[X] CBProDeviceRead failed (exit {result.returncode})")
    line = result.stdout.strip().splitlines()[-1]
    return int(line.split(",")[-1].strip(), 16)


def already_configured(config_lines) -> bool:
    """先讀回目前狀態，判斷是否已經跟目標 config 完全一致（不需要重寫）"""
    expected = {}
    for line in config_lines:
        addr, data = [p.strip() for p in line.split(",")]
        expected[int(addr, 16)] = int(data, 16)

    actual = read_back([f"0x{addr:04X}" for addr in expected.keys()])
    if any(actual.get(addr) != val for addr, val in expected.items()):
        return False

    return read_usys_stat() == USYS_STAT_ACTIVE


def write_stage_file(lines) -> str:
    fd, path = tempfile.mkstemp(suffix=".txt", prefix="si5332_stage_")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return path


def run_stage(name: str, lines) -> bool:
    path = write_stage_file(lines)
    try:
        print(f"=== Stage: {name} ({len(lines)} register write(s)) ===")
        result = subprocess.run([CBPRO_WRITE, "--registers", path],
                                 capture_output=True, text=True)
        print(result.stdout)
        if result.returncode != 0:
            print(result.stderr, file=sys.stderr)
            print(f"[X] Stage '{name}' failed (exit {result.returncode})")
            return False
        return True
    finally:
        os.unlink(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--project", help="CBPro 專案檔 (.slabtimeproj)")
    group.add_argument("--registers", help="已匯出的暫存器腳本 (.csv)")
    args = parser.parse_args()

    csv_path = args.registers or export_registers(args.project)

    preamble, config, postamble = split_stages(csv_path)

    print(f"檢查目前狀態是否已符合目標設定（{len(config)} 筆暫存器 + USYS_STAT）...")
    if already_configured(config):
        print("[OK] Si5332 目前設定已跟目標一致且為 ACTIVE，不需要重寫，跳過。")
        return

    print("設定不符或未啟動，開始分階段寫入...")
    if not run_stage("Ready (preamble)", preamble):
        sys.exit(1)
    if not run_stage("Config", config):
        sys.exit(1)
    if not run_stage("Active (postamble)", postamble):
        sys.exit(1)

    print("\n[OK] Si5332 configured successfully (staged write, all 3 stages passed).")


if __name__ == "__main__":
    main()
