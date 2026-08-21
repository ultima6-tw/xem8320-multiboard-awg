#!/usr/bin/env python3
"""
flash_and_reload.py — XEM8320 一鍵燒錄 + reload（Step 14.3）

流程：
  1. 偵測 USB 上的板子數量與 serial
  2. 對所有（或指定）板子寫入 bitstream 到 SPI flash
  3. 燒錄完成後，對所有板子執行 ConfigureFPGAFromFlash（不需 power cycle）

用法：
  python flash_and_reload.py path/to/x.bit                     # 燒錄所有連接板子
  python flash_and_reload.py path/to/x.bit --serial BOARD_A_SERIAL  # 燒錄指定板
  python flash_and_reload.py path/to/x.bit --parallel           # 平行燒錄所有連接板子
  python flash_and_reload.py --list                             # 只列出板子

bitfile 必須明確指定（無預設路徑，避免燒到舊專案的 bitstream）。

--parallel（2026-07-31 新增）：每片板子各自用獨立 subprocess 執行
（等於遞迴呼叫自己的單板模式，本來就是完整的 flash+reload），刻意
不在同一個 process 內同時開多個 ok.FrontPanelDevices()——這個專案
2026-07-24/25 已經證實這樣做會讓 process 直接消失（exit code 127，
無例外、無 log），見 NOTES.md「除錯工具附帶記錄」。輸出導向暫存檔
而不是 PIPE，避免燒錄進度輸出量大（13MB bitfile ≈ 13000 個
progress bar 更新）把 PIPE 塞滿造成 deadlock。
"""
import sys, time, argparse, subprocess, tempfile
from pathlib import Path

sys.path.insert(0, r"C:\Program Files\Opal Kelly\FrontPanelUSB\API\Python\3")
import ok

# 2026-08-18：改指向 step-16 本地的 flash/flashloader.bit（原本指向
# awg-test-step-14/flash/，但該資料夾已在 2026-08-10 搬進
# Projects/FPGA/_archive/，路徑因此失效——跟 create_bd.tcl 的
# src_dir6/7/14 是同一類問題，使用者確認 step-16 是最後版本、需要
# 整合起來，所以複製一份進本地 flash/，不再依賴外部/已封存資料夾）。
FLASHLOADER_BIT    = Path(__file__).resolve().parents[1] / "flash" / "flashloader.bit"
FLASH_SECTOR_SIZE  = 65536
FLASH_PAGE_SIZE    = 256
MAX_TRANSFER_SIZE  = 1024

# ── 裝置列舉 ─────────────────────────────────────────────────────────────────

def get_connected_serials():
    devs = ok.FrontPanelDevices()
    n = devs.GetCount()
    return [devs.GetSerial(i) for i in range(n)]

def print_devices(serials):
    if not serials:
        print("No FrontPanel devices found.")
        return
    print(f"Connected devices ({len(serials)}):")
    for i, s in enumerate(serials):
        print(f"  [{i}] {s}")

# ── Flash write（單板）────────────────────────────────────────────────────────

def _wait_done(fp, timeout_ms=5000):
    deadline = time.time() + timeout_ms / 1000
    while time.time() < deadline:
        fp.UpdateTriggerOuts()
        if fp.IsTriggered(0x60, 0x0001):
            return True
        time.sleep(0.005)
    return False

def flash_one(serial, bitfile_path):
    devs = ok.FrontPanelDevices()
    dev = devs.Open(serial)
    if dev is None:
        print(f"[X] {serial}: cannot open")
        return False

    info = ok.okTDeviceInfo()
    dev.GetDeviceInfo(info)
    print(f"\n── {serial} ({info.productName}) ──")

    print(f"  Loading flashloader.bit...")
    ret = dev.ConfigureFPGA(str(FLASHLOADER_BIT))
    if ret != ok.ErrorCode.NoError:
        print(f"  [X] ConfigureFPGA failed: {ret}")
        return False
    if not dev.IsFrontPanelEnabled():
        print(f"  [X] FrontPanel not enabled")
        return False

    fp = dev.GetFPGADataPortClassic()

    raw = Path(bitfile_path).read_bytes()
    sync_pos = 0
    for i in range(len(raw) - 4):
        if raw[i:i+4] == b'\xff\xff\xff\xff':
            sync_pos = max(0, i - 2)
            break
    data = raw[sync_pos:]
    file_size = len(data)
    print(f"  Bitfile: {len(raw)//1024} kB raw → {file_size//1024} kB trimmed")

    # Erase
    n_sectors = (file_size + FLASH_SECTOR_SIZE - 1) // FLASH_SECTOR_SIZE
    print(f"  Erasing {n_sectors} sectors...", end="", flush=True)
    fp.SetWireInValue(0x00, 0x0000)
    fp.SetWireInValue(0x01, n_sectors)
    fp.UpdateWireIns()
    fp.UpdateTriggerOuts()
    fp.ActivateTriggerIn(0x40, 3)
    for _ in range(600):
        time.sleep(0.2)
        fp.UpdateTriggerOuts()
        if fp.IsTriggered(0x60, 0x0001):
            break
    else:
        print(" TIMEOUT")
        return False
    print(" done")

    # Write
    n_chunks = (file_size + MAX_TRANSFER_SIZE - 1) // MAX_TRANSFER_SIZE
    print(f"  Writing {file_size//1024} kB...")
    buf = bytearray(MAX_TRANSFER_SIZE)
    page_addr = 0
    BAR_W = 40
    for i in range(n_chunks):
        offset = i * MAX_TRANSFER_SIZE
        chunk = data[offset:offset + MAX_TRANSFER_SIZE]
        buf[:] = b'\xff' * MAX_TRANSFER_SIZE
        buf[:len(chunk)] = chunk
        fp.WriteToPipeIn(0x80, buf)
        fp.SetWireInValue(0x00, page_addr)
        fp.SetWireInValue(0x01, (MAX_TRANSFER_SIZE // FLASH_PAGE_SIZE) - 1)
        fp.UpdateWireIns()
        fp.UpdateTriggerOuts()
        fp.ActivateTriggerIn(0x40, 5)
        if not _wait_done(fp, timeout_ms=5000):
            print(f"\r  TIMEOUT at chunk {i}")
            return False
        page_addr += MAX_TRANSFER_SIZE // FLASH_PAGE_SIZE
        pct = (i + 1) / n_chunks
        done = int(BAR_W * pct)
        bar = "#" * done + "-" * (BAR_W - done)
        kb_done = (i + 1) * MAX_TRANSFER_SIZE // 1024
        print(f"\r  [{bar}] {pct*100:5.1f}%  {kb_done}/{file_size//1024} kB", end="", flush=True)
    print(f"\r  [{'#'*BAR_W}] 100.0%  {file_size//1024}/{file_size//1024} kB  done")

    # Quad Enable
    if info.hasQuadConfigFlash:
        fp.ActivateTriggerIn(0x40, 6)
        time.sleep(0.3)
        print(f"  Quad Enable set")

    print(f"  [OK] Flash write complete")
    return True

# ── Reload from flash（單板）────────────────────────────────────────────────

def reload_one(serial):
    devs = ok.FrontPanelDevices()
    dev = devs.Open(serial)
    if dev is None:
        print(f"  [X] {serial}: cannot open for reload")
        return False
    print(f"  Reloading {serial} from flash...", end="", flush=True)
    ret = dev.ConfigureFPGAFromFlash(0)
    if ret == 0:
        print(" OK")
        return True
    else:
        print(f" FAIL (err={ret})")
        return False

# ── 平行燒錄（多板，各自獨立 subprocess）────────────────────────────────────

def flash_reload_parallel(targets, bitfile):
    """每片板子各自用獨立 subprocess 執行完整的
    `flash_and_reload.py --serial <serial> <bitfile>`（遞迴呼叫自己，
    單板模式本來就是完整的 flash+reload），刻意不在同一個 process 內
    同時開多個 ok.FrontPanelDevices()（見檔頭說明的已知 crash 問題）。
    輸出導向暫存檔而不是 subprocess.PIPE，避免燒錄進度輸出量大時把
    PIPE 塞滿造成 deadlock（parent 依序 wait，PIPE 版會卡死）。"""
    print(f"\n== 平行燒錄 {len(targets)} 片板子（各自獨立 process）==")
    script_path = str(Path(__file__).resolve())
    procs = {}
    for serial in targets:
        log_f = tempfile.TemporaryFile(mode="w+", encoding="utf-8", errors="replace")
        p = subprocess.Popen(
            [sys.executable, script_path, "--serial", serial, str(bitfile)],
            stdout=log_f, stderr=subprocess.STDOUT)
        procs[serial] = (p, log_f)

    ok_list = []
    for serial, (p, log_f) in procs.items():
        p.wait()
        log_f.seek(0)
        output = log_f.read()
        log_f.close()
        status = "[OK]" if p.returncode == 0 else f"[X] FAIL (exit={p.returncode})"
        print(f"\n── {serial}: {status} ──")
        print(output)
        if p.returncode == 0:
            ok_list.append(serial)
    return ok_list

# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="XEM8320 flash + reload (Step 14.3)")
    parser.add_argument("--list",   action="store_true", help="List connected devices and exit")
    parser.add_argument("--serial", default=None, help="Only flash this serial number")
    parser.add_argument("--parallel", action="store_true",
                         help="平行燒錄多片板子（各自獨立 subprocess，避開 OpalKelly binding 同一 process 內開多個裝置會 crash 的已知問題）")
    parser.add_argument("bitfile",  nargs="?", default=None, help="Bitfile (.bit) to flash (required unless --list)")
    args = parser.parse_args()

    serials = get_connected_serials()
    print_devices(serials)

    if args.list:
        return

    if not serials:
        sys.exit("[X] No devices found")

    if not args.bitfile:
        sys.exit("[X] bitfile argument required, e.g.\n"
                  "    python flash_and_reload.py D:/Vivado/awg_step15a/awg_step15a.runs/impl_1/awg_step15a_bd_wrapper.bit")

    targets = [args.serial] if args.serial else serials
    bitfile = Path(args.bitfile)
    if not bitfile.exists():
        sys.exit(f"[X] Bitfile not found: {bitfile}")
    print(f"\nBitfile: {bitfile}")

    if args.parallel:
        ok_list = flash_reload_parallel(targets, bitfile)
        if not ok_list:
            sys.exit("[X] No boards flashed successfully")
        print(f"\nDone. {len(ok_list)} board(s) updated (parallel).")
        return

    # Flash
    flashed = []
    for serial in targets:
        if serial not in serials:
            print(f"[X] {serial} not connected, skipping")
            continue
        ok_flag = flash_one(serial, bitfile)
        if ok_flag:
            flashed.append(serial)

    if not flashed:
        sys.exit("[X] No boards flashed successfully")

    # Reload
    print(f"\n── Reloading from flash ──")
    for serial in flashed:
        reload_one(serial)

    print(f"\nDone. {len(flashed)} board(s) updated.")

if __name__ == "__main__":
    main()
