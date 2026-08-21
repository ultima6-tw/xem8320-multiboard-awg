#!/usr/bin/env python3
"""
si5332_configure_linux.py -- Si5332-6EX-EVB Linux 版設定工具（pyusb 直接控制）

跟 si5332_configure.py（Windows 版，靠 subprocess 呼叫 CBPro CLI .exe）的差異：
  - 不支援 --project：.slabtimeproj -> CSV 的匯出（CBProProjectRegistersExport.exe）
    純粹是離線格式轉換，完全不碰硬體，這台機器沒有這支 exe，一樣要先在 Windows
    上跑一次 CBProProjectRegistersExport --project <file> --include-load-writes
    --format csv --outfile <out>.csv，再把這個 csv 複製過來這台機器用。
  - 讀寫暫存器改成直接跟裝置的 USB bulk endpoint 溝通（pyusb），protocol 細節
    是從 Windows 側錄 CBProDeviceWrite.exe/CBProDeviceRead.exe 逆向出來的，
    見 si5332_usb_linux.py 開頭註解 / NOTES.md「Si5332-6EX-EVB USB 協定側錄 +
    逆向」章節（2026-07-28）。
  - 沿用同一個「Ready/Config/Active 三階段各自獨立 session」的防呆設計
    （已知 I2C lockup bug，見 si5332_configure.py docstring）：每個 stage 都
    重新 open()/close() 裝置（各自重跑一次握手），不共用同一條連線。

用法：
  python si5332_configure_linux.py --registers <path>.csv
      csv 格式跟 si5332_configure.py 一樣：CBProProjectRegistersExport
      --include-load-writes --format csv 產生的檔案（含 preamble/config/
      postamble 三段註解分隔）。
"""
import sys
import argparse

from si5332_usb_linux import Si5332USB, Si5332USBError, USYS_STAT_ACTIVE
from si5332_registers import split_stages, parse_lines


def already_configured(config_lines) -> bool:
    expected = parse_lines(config_lines)
    with Si5332USB() as si:
        actual = si.read_back(expected.keys())
        if any(actual.get(addr) != val for addr, val in expected.items()):
            return False
        return si.read_usys_stat() == USYS_STAT_ACTIVE


def run_stage(name: str, lines) -> bool:
    parsed = parse_lines(lines)
    print(f"=== Stage: {name} ({len(parsed)} register write(s)) ===")
    try:
        with Si5332USB() as si:
            for addr, data in parsed.items():
                si.write_register(addr, data)
    except Si5332USBError as e:
        print(f"[X] Stage '{name}' failed: {e}", file=sys.stderr)
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--registers", required=True,
                         help="已匯出的暫存器腳本 (CBProProjectRegistersExport 產生的 csv，從 Windows 複製過來)")
    args = parser.parse_args()

    preamble, config, postamble = split_stages(args.registers)

    print(f"檢查目前設定是否已符合目標（{len(config)} 筆暫存器 + USYS_STAT）...")
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

    print("\n[OK] Si5332 configured successfully (staged write via pyusb, all 3 stages passed).")


if __name__ == "__main__":
    main()
