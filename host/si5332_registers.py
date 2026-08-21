#!/usr/bin/env python3
"""
si5332_registers.py -- Si5332 暫存器腳本 (CBProProjectRegistersExport CSV) 共用解析邏輯

純字串/數值解析，不含任何 OS-specific 呼叫，Windows 版 (si5332_configure.py)
跟 Linux 版 (si5332_configure_linux.py) 都 import 這份共用邏輯，避免兩邊
各自維護一份、日後改格式時漏改其中一邊。
"""
import sys


def split_stages(csv_path: str):
    """把匯出的暫存器腳本拆成 (preamble, config, postamble) 三段內容（不含註解行）"""
    with open(csv_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    preamble, config, postamble = [], [], []
    section = None
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("# Start configuration preamble"):
            section = "pre"
            continue
        if stripped.startswith("# End configuration preamble"):
            section = None
            continue
        if stripped.startswith("# Start configuration registers"):
            section = "cfg"
            continue
        if stripped.startswith("# End configuration registers"):
            section = None
            continue
        if stripped.startswith("# Start configuration postamble"):
            section = "post"
            continue
        if stripped.startswith("# End configuration postamble"):
            section = None
            continue
        if not stripped or stripped.startswith("#") or stripped == "Address,Data":
            continue
        if section == "pre":
            preamble.append(stripped)
        elif section == "cfg":
            config.append(stripped)
        elif section == "post":
            postamble.append(stripped)

    if not preamble or not config or not postamble:
        sys.exit("[X] 無法從匯出檔解析出 preamble/config/postamble 三段，"
                  "格式可能跟預期不符（需要 --include-load-writes 產生的格式）")
    return preamble, config, postamble


def parse_lines(lines):
    """把 'addr,data' 這種 hex 字串列表轉成 {int(addr): int(data)}"""
    parsed = {}
    for line in lines:
        addr, data = [p.strip() for p in line.split(",")]
        parsed[int(addr, 16)] = int(data, 16)
    return parsed
