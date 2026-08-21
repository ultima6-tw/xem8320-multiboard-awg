"""
query_board_status.py -- 統一讀取/寫入架構的查詢工具（2026-07-27 新增）

送 T_QUERY(0x12) 帶 query_type，讀 PO_STATUS_REPLY(0xA1) BTPipeOut 拿
回覆。五種 query_type（QT_BOARD_INFO/QT_SINE_STATUS/QT_CALIB_STATUS/
QT_TRIGGER_GROUP/QT_DDR_STATUS）都能問，dest_id 可以是 --serial 那片
板子自己（本機 loopback，跟遠端查詢走同一套機制，見 PROJECT.md「統一
讀取/寫入架構」一致性優先的設計決定），也可以是別片板子。

**2026-07-30 新增 QT_DDR_STATUS**：跨板讀 DDR 播放狀態
（current_idx/next_idx/mux_sel/play_pos，`dac_clk`→`sys_clk` 走
`level_cdc`）。跟既有 `read_port_status.py`（讀 WO 0x30-0x33，本機
USB 直讀、無 CDC）並存，兩條路徑格式一致但用途不同：本機/USB 已連線
時用 `read_port_status.py`；需要跨板（USB 沒接在那片板子上）時用這裡
的 `--query ddr_status`。

用法：
  # 查自己的板身份資訊
  python query_board_status.py --serial BOARD_A_SERIAL --dest-id 0x0000 --query board_info

  # 從 board A 的 USB，查 board B 的 sine_gen 即時狀態
  python query_board_status.py --serial BOARD_A_SERIAL --dest-id 0x0001 --query sine_status

  # 查校正狀態 / trigger 分組 / DDR 播放狀態
  python query_board_status.py --serial BOARD_A_SERIAL --dest-id 0x0000 --query calib_status
  python query_board_status.py --serial BOARD_A_SERIAL --dest-id 0x0000 --query trigger_group
  python query_board_status.py --serial BOARD_A_SERIAL --dest-id 0x0001 --query ddr_status
"""
import sys
import time
import argparse

sys.path.insert(0, r"C:\path\to\awg-test-step-16\host")
from awg_common import open_board, send_query, read_status_reply, QT_DECODERS, \
    QT_NAMES, QT_BOARD_INFO, QT_SINE_STATUS, QT_CALIB_STATUS, QT_TRIGGER_GROUP, \
    QT_DDR_STATUS, TI_CMD, TI_BIT_FLUSH_STANDBY

QUERY_NAME_TO_TYPE = {
    "board_info":    QT_BOARD_INFO,
    "sine_status":   QT_SINE_STATUS,
    "calib_status":  QT_CALIB_STATUS,
    "trigger_group": QT_TRIGGER_GROUP,
    "ddr_status":    QT_DDR_STATUS,
}


def print_board_info(d):
    print(f"  board_id      = 0x{d['board_id']:04X}")
    print(f"  is_master     = {d['is_master']}")
    print(f"  total_boards  = {d['total_boards']}")
    print(f"  board_index   = {d['board_index']}")
    print(f"  channel_up_0  = {d['channel_up_0']}")
    print(f"  channel_up_1  = {d['channel_up_1']}")
    print(f"  init_ok       = {d['init_ok']}")
    print(f"  ext_clk_sel   = {d['ext_clk_sel']}")
    print(f"  per_hop_value = {d['per_hop_value']}")
    print(f"  trig_delay          = {d['trig_delay']}")
    print(f"  manual_delay_active = {d['manual_delay_active']}")


def print_sine_status(d):
    print(f"  mux_sel_sync = 0x{d['mux_sel_sync']:02X} (bit=0 -> a side active, bit=1 -> b side active)")
    for ch, params in enumerate(d['channels']):
        print(f"  channel {ch}: phase_acc=0x{d['phase_acc'][ch]:08X}")
        for name, val in params.items():
            print(f"      {name:17s} = 0x{val:08X}")


def print_calib_status(d):
    print(f"  scale_cfg = 0x{d['scale_cfg']:02X}")
    for ch, v in enumerate(d['amp_ctrl']):
        print(f"  amp_ctrl[{ch}]  = 0x{v:05X}")
    for i, v in enumerate(d['calib_coef']):
        print(f"  calib_coef[{i:2d}] = 0x{v:05X}")


def print_trigger_group(d):
    print(f"  group_id_a = {d['group_id_a']}")
    print(f"  group_id_b = {d['group_id_b']}")
    print(f"  group_id_c = {d['group_id_c']}")
    print(f"  group_id_d = {d['group_id_d']}")


def print_ddr_status(d):
    for ch, c in enumerate(d['channels']):
        print(f"  channel {ch}: current_idx={c['current_idx']}  next_idx={c['next_idx']}  "
              f"mux_sel={c['mux_sel']}  play_pos={c['play_pos']}")


PRINTERS = {
    QT_BOARD_INFO:    print_board_info,
    QT_SINE_STATUS:   print_sine_status,
    QT_CALIB_STATUS:  print_calib_status,
    QT_TRIGGER_GROUP: print_trigger_group,
    QT_DDR_STATUS:    print_ddr_status,
}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--serial', required=True, help='送封包用哪片板子的 USB 連線（也是讀 PO_STATUS_REPLY 的那片）')
    p.add_argument('--dest-id', type=lambda x: int(x, 0), required=True,
                    help='要查詢的目標板子 board_id（本機查自己就填自己的 board_id）')
    p.add_argument('--query', required=True, choices=list(QUERY_NAME_TO_TYPE),
                    help='查詢種類')
    p.add_argument('--wait', type=float, default=0.05,
                    help='送出 T_QUERY 後等多久再讀 PO_STATUS_REPLY（秒，預設 0.05；跨板環路較大時可能要加大）')
    args = p.parse_args()

    query_type = QUERY_NAME_TO_TYPE[args.query]

    print(f"Opening {args.serial}...")
    dev, fp = open_board(args.serial)
    print(f"OK  (query={args.query} query_type={query_type}, dest_id=0x{args.dest_id:04X})\n")

    fp.ActivateTriggerIn(TI_CMD, TI_BIT_FLUSH_STANDBY)
    time.sleep(0.01)

    send_query(fp, args.dest_id, query_type)
    print(f"[1] T_QUERY sent (query_type={QT_NAMES[query_type]})")

    time.sleep(args.wait)

    src, got_query_type, data_words = read_status_reply(fp)
    print(f"[2] PO_STATUS_REPLY: src=0x{src:04X} query_type={got_query_type} "
          f"({QT_NAMES.get(got_query_type, '?')})")

    if got_query_type != query_type:
        sys.exit(f"[X] 收到的 query_type (0x{got_query_type:02X}) 跟送出的 (0x{query_type:02X}) 不符，"
                  f"可能是舊回覆還沒被沖掉，或封包還沒跑完一圈——加大 --wait 再試一次")

    result = QT_DECODERS[query_type](data_words)
    print(f"\n=== {args.query} (dest_id=0x{args.dest_id:04X}) ===")
    PRINTERS[query_type](result)


if __name__ == "__main__":
    main()
