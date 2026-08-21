"""
board_ctrl.py — 單板操作面板用的 host 邏輯（2026-07-31 新增）

波形上膛（arm）+ trigger + DDR 播放狀態讀回，全部走 master 這條 USB，
可以對任何 dest_id（包含 master 自己 loopback）操作，不需要開該板自己
的 USB——沿用 `test_awg_arm_2slot_fpddr4.py --dest-id` 已經驗證過的
Aurora 遠端上膛機制（`T_WAVEFORM_STREAM`/`T_LIST_WRITE`/`T_PLAY_CTRL`
都支援 dest_id 轉送，示波器驗證過，見 PROJECT.md「真正上膛+播放驗證
完成」章節）。

跟 `test_awg_arm_2slot_fpddr4.py`/`test_awg_arm_nslot_fpddr4.py` 的
差異：這裡把「上膛任意 slot 數」跟「dest_id 可以是別片板子」兩個既有
能力合併成一個通用函式，slot 數/頻率完全由呼叫端（web API 的 request
body）決定，不是寫死 2 個 slot。

Trigger 用 `T_TRIG_START` 廣播——協定本身就是 broadcast + group_select，
不是 per-board 定址，只有 `is_master` 的板子會真的動作，跟 `test_
aurora_trigger.py` 完全一致，所以這裡沒有 dest_id 參數。

DDR 播放狀態讀回用既有的 `T_QUERY(QT_DDR_STATUS)`（`awg_common.py`
已有 `decode_qt_ddr_status`，跟 `query_board_status.py` 同一套機制）。

失敗時直接 raise（不像 `init_flow.py` 用結構化 phases——這裡每個操作
都是單一動作，不是多步驟流程，呼叫端 Flask endpoint 自己 try/except
轉成 JSON error response 即可，不需要 phases 的複雜度）。
"""
import sys
import os
import time
import struct
import math

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from multitone_scan import (
    precompute_full_sum, load_gain_correction,
    N as MULTITONE_N, N_TONES as MULTITONE_N_TONES,
    compute_comb_bins, render_comb_waveform,
)
from awg_common import (
    send_pkt, send_query, read_status_reply,
    T_LIST_WRITE, T_PLAY_CTRL, T_TRIG_START, T_WAVEFORM_STREAM, T_TRIG_MASK,
    T_TRIG_DELAY_CFG, T_SINE_CTRL, T_TIMER_CTRL, T_GROUP_SCHED_CTRL,
    T_SINE_LIST_CTRL,
    DEST_BCAST, QT_DDR_STATUS, decode_qt_ddr_status,
    QT_SINE_STATUS, decode_qt_sine_status,
    QT_TRIGGER_GROUP, decode_qt_trigger_group,
    DAC_FS, wave_len_for_freq, wave_len_for_two_freqs, wave_len_for_comb_plus_freq, pad_to_burst,
    TI_CMD, TI_BIT_FLUSH_STANDBY,
    PARAM_TUNING_WORD, PARAM_PHASE, PARAM_START_AMP, PARAM_STEP,
    PARAM_DURATION_CYCLES, PARAM_LOOP_MODE, PARAM_MODE, PARAM_RAMP_EN,
    SINE_LIST_PARAM_COMMIT_DEPTH, SINE_N_SLOT,
)

PI_WAVE_DATA = 0x81
BTPIPE_BLOCK = 1024
SLOT0_ADDR = 0x00002000
SLOT_MARGIN = 0x1000  # slot 間位址留白，避免不同長度波形重疊（沿用既有腳本慣例）
MAX_SLOTS = 8          # list_depth 是 3-bit port（waveform_controller.v）
DEFAULT_AMP = 8000

# 2026-08-18 新增：每個 module 一個固定、互不重疊的 16MB 位址區段
# （遠大於這個專案目前任何一次上膛實際會用到的量——最大的
# arm_multitone_pattern() 單一 slot 也才 ~800KB）。修這個之前，四個
# arm_*() 函式全部共用同一個 SLOT0_ADDR，每次呼叫都從頭往上排，只要
# 兩次呼叫涵蓋不同的 module 子集合（例如一次 arm 全部 4 個 module 的
# Reset to Zero，接著另一次只 arm module 0 的大波形），後面那次的
# 位址範圍完全可能蓋過前一次其他 module 已經寫好的資料——排查
# 900kHz/600kHz 示範時發現這個問題，見 NOTES.md 2026-08-18「續：
# `_pack_ch1_ch2` 修法上機後仍不 work」章節。改成每個 module 呼叫
# _module_base_addr(m) 各自獨立起算，不同 module 的位址區段從此
# 保證不重疊，不管呼叫順序/涵蓋的 module 子集合怎麼變化。
MODULE_ADDR_STRIDE = 0x01000000  # 16MB per module


def _module_base_addr(m):
    return SLOT0_ADDR + m * MODULE_ADDR_STRIDE


def _check_slot_fits(m, slot_addr, n_samples):
    """固定分區只保證『起點』不會撞到別的 module，同一個 module 自己
    的 slot 長度仍然是不固定的（使用者上傳內容/頻率算出來的長度都
    可能很大）——這裡在實際寫入前擋一次，slot 寫完後的結束位址如果
    超出這個 module 自己的 16MB 區段，就直接報錯，不要讓它靜默溢位
    寫壞下一個 module 的資料。"""
    region_start = _module_base_addr(m)
    region_end = region_start + MODULE_ADDR_STRIDE
    end_addr = slot_addr + n_samples * 4
    if end_addr > region_end:
        raise ValueError(
            f"module {m}: slot content too large — ends at 0x{end_addr:08X}, "
            f"exceeds this module's {MODULE_ADDR_STRIDE // (1024*1024)}MB region "
            f"(0x{region_start:08X}-0x{region_end:08X})"
        )


def _flush_standby(fp):
    fp.ActivateTriggerIn(TI_CMD, TI_BIT_FLUSH_STANDBY)
    time.sleep(0.01)


def make_waveform_stream_pkt(dest_id, addr, samples32):
    """組 T_WAVEFORM_STREAM(0x13) 封包，跟既有腳本完全一致的格式：
    beat0=header，beat1={total_bytes[63:32],addr[31:0]}，beat2..N=波形
    資料（64-bit/beat=2 個 32-bit sample）。"""
    total_bytes = len(samples32) * 4
    assert total_bytes % 16 == 0, "wave_total_bytes must be a multiple of 16"
    n_data_beats = total_bytes // 8
    pkt_len = 2 + n_data_beats
    b0 = struct.pack('<II', (0 << 16) | (dest_id & 0xFFFF), (pkt_len << 8) | T_WAVEFORM_STREAM)
    b1 = struct.pack('<II', addr & 0xFFFFFFFF, total_bytes & 0xFFFFFFFF)
    data = bytearray()
    for i in range(0, len(samples32), 2):
        lo = samples32[i]
        hi = samples32[i + 1] if i + 1 < len(samples32) else 0
        data += struct.pack('<II', lo, hi)
    pkt = bytearray(b0 + b1) + data
    rem = len(pkt) % BTPIPE_BLOCK
    if rem != 0:
        pkt += b'\x00' * (BTPIPE_BLOCK - rem)
    return pkt


def _pack_ch1_only(val):
    return (int(val) & 0x3FFF) << 18 & 0xFFFFFFFF


def _gen_sine_ch1(n, amp, period):
    return [_pack_ch1_only(amp * math.sin(2 * math.pi * i / period)) for i in range(n)]


def _gen_from_samples(amp, values):
    """values：使用者上傳檔案解析出的振幅值清單（-1.0~1.0），跟
    _gen_sine_ch1() 用同一種 amp 縮放 + _pack_ch1_only 編碼，唯一差異是
    波形樣本來源是任意上傳資料，不是 host 端算出的正弦波（2026-08-03
    新增，見 arm_waveform_from_files()）。"""
    return [_pack_ch1_only(amp * v) for v in values]


def write_wave(fp, dest_id, addr, samples):
    pkt = make_waveform_stream_pkt(dest_id, addr, samples)
    _flush_standby(fp)
    ret = fp.WriteToBlockPipeIn(PI_WAVE_DATA, BTPIPE_BLOCK, pkt)
    if ret != len(pkt):
        raise RuntimeError(f"T_WAVEFORM_STREAM WriteToBlockPipeIn short: {ret} != {len(pkt)}")


def send_list_write(fp, dest_id, ch, slot, addr, wave_len, repeat=1):
    """repeat>1（2026-08-10，零成本診斷用，見 NOTES.md「T_LIST_WRITE CDC
    缺陷」章節）：重送同一組值 N 次，中間夾延遲，讓 sys_clk/dac_clk 的
    相位飄移在每次重送時都不一樣——如果 aurora_ctrl_mux.v 的 list write
    路徑真的缺 CDC（懷疑對象），單次送出偶爾被跨時脈域吃掉/撕裂的機率
    會隨重送次數指數下降，藉此在不動 RTL 的情況下驗證假設方向對不對。
    重送同一組值本身是冪等的，不會因為送第二次而改變結果。"""
    sel = ((slot & 0x7) << 2) | (ch & 0x3)
    for _ in range(repeat):
        _flush_standby(fp)
        send_pkt(fp, dest_id, T_LIST_WRITE, beat1_lo=sel,
                  extra_beats=[(addr & 0xFFFFFFFF, 0), (wave_len & 0xFFFFFFFF, 0)])
        if repeat > 1:
            time.sleep(0.005)


# ── trig_timer（自動排程觸發，2026-08-04 host 端首次接上）───────────────
# rtl/trig_timer.v：每個 module（port 0-3 = z0-z3，跟 wctrl_$ch 同一組
# 編號，見 create_bd.tcl 876/4426 行）各自一個硬體排程器，armed 後收到
# 一次 first_trigger（接的就是既有 dac_trig_queue_0/native_trig_out，
# 也就是 /api/trigger 這個現有機制本身，不需要新的啟動指令）就會照
# intervals 自動連續觸發，不用 host 介入。這條路徑 RTL/Aurora 協定層
# 早就支援（local_reg_handler.v 的 0x0C 解碼），但在這之前完全沒有任何
# host 腳本（local 或 remote）碰過，是全新、未上機驗證過的整合。
#
# 2026-08-04（同一天）：list_depth 原本只有 3-bit（合法值 0-7，8 這個
# 數字本身無法編碼），已修好——RTL 端（trig_timer.v/aurora_ctrl_mux.v/
# local_reg_handler.v/create_bd.tcl）改成 16 slot、5-bit depth（1-16 都
# 能正確編碼，不再有死格），beat1 欄位版面也跟著重排，見下方
# send_timer_ctrl()。完整推導見 PROJECT.md trigger group 排程功能
# 三部曲「B」條目、rtl/trig_timer.v 檔頭註解。
def send_timer_ctrl(fp, dest_id, port, slot, depth, loop_en, run, interval_cycles):
    """T_TIMER_CTRL(0x0C)，3-beat：beat1[3:0]=slot(0-15) [8:4]=depth(0-16)
    [9]=loop_en [10]=run [12:11]=port(0-3) beat2[31:0]=interval_cycles
    （dac_clk cycles，100MHz=10ns 解析度）。每次呼叫寫入一個 slot 的
    interval 值，同時（重新）宣告 depth/loop_en/run/port——寫入完整
    排程要呼叫多次（每個 slot 一次），run 通常只在最後一次呼叫才設 1。
    """
    if not (0 <= port <= 3):
        raise ValueError("port must be 0-3 (module z0-z3)")
    if not (0 <= slot <= 15):
        raise ValueError("slot must be 0-15")
    if not (0 <= depth <= 16):
        raise ValueError("depth must be 0-16 (5-bit field — see rtl/trig_timer.v)")
    beat1 = ((port & 0x3) << 11) | ((1 if run else 0) << 10) | ((1 if loop_en else 0) << 9) \
        | ((depth & 0x1F) << 4) | (slot & 0xF)
    send_pkt(fp, dest_id, T_TIMER_CTRL, beat1_lo=beat1,
              extra_beats=[(interval_cycles & 0xFFFFFFFF, 0)])
    time.sleep(0.01)  # 比照 send_list_write() 呼叫端在多筆連續寫入之間留的
    # settling time（arm_waveform_from_files() 的 send_list_write 迴圈也是
    # 每筆之間 sleep(0.01)）——run/loop_en/depth 是 level_cdc（quasi-static
    # 2-flop synchronizer），list_wr_en 是 trigger_cdc（單週期 pulse），
    # 連續送太快有 CDC pulse 被下一筆覆蓋、或 host 端在 level 訊號真正
    # 穿越到 dac_clk domain 前就送下一筆的風險。


def set_trig_timer(fp, dest_id, port, intervals_cycles, loop_en, run):
    """幫一個 module（port=0-3，對應 z0-z3）的 trig_timer 寫入完整排程。
    intervals_cycles：1-7 個 dac_clk cycle 數的清單（每個 module 自己的
    trig_timer 是獨立的，不是共用暫存器，不像 dac_mode_ramp 那樣需要
    覆寫全部 4 個 module）。逐一送 T_TIMER_CTRL 寫進對應 slot，只有最後
    一次呼叫的封包才真正把 run bit 設成呼叫端要的值——這樣期間 run 保持
    False，避免拿還沒寫完的排程去 arm。run=True 時會讓 trig_timer 進入
    armed 狀態，等下一次對這個 module 所屬 trigger group 的 Trigger
    （既有 /api/trigger）當 first_trigger 才真正開始跑；run=False 會
    立即 stop/disarm（不管當下在跑到哪個 slot）。intervals_cycles 為空
    只送一個 depth=0 的 control-only 封包（單純拿來下 run=False 停止用）。
    """
    depth = len(intervals_cycles)
    if run and not (1 <= depth <= 16):
        raise ValueError("intervals_cycles must have 1-16 entries when run=True")
    if depth == 0:
        send_timer_ctrl(fp, dest_id, port, slot=0, depth=0, loop_en=loop_en, run=run, interval_cycles=0)
        return
    for slot, intv in enumerate(intervals_cycles):
        is_last = (slot == depth - 1)
        send_timer_ctrl(fp, dest_id, port, slot=slot, depth=depth, loop_en=loop_en,
                         run=(run if is_last else False), interval_cycles=intv)


# ── sine N-slot（Sine mode 排程表，2026-08 新增，host 端首次接上）──────
# rtl/sine_ctrl_regs.v：每個 physical channel（0-7）各自一個 N_SLOT=4
# 深的排程表，跟既有 T_SINE_CTRL 的單值立即生效寫入是完全獨立的儲存區
# ——這裡寫的內容要等 commit（arm）之後，靠 trig_start（既有 /api/trigger
# 機制）才會依序輪播，不是寫了就馬上生效。跟 set_trig_timer()/
# set_group_sched() 同一種「逐筆寫入+最後一筆才真正 commit」慣例，但
# commit 的語意不是「run bit」而是「commit list_depth 並 arm」（見
# rtl/sine_ctrl_regs.v 檔頭「host 端寫入順序 hard contract」）。
def send_sine_list_ctrl(fp, dest_id, ch, slot_idx, param_sel, value):
    """T_SINE_LIST_CTRL(0x2D)，2-beat：beat1[31:0]=data
    beat1[39:32]=sel={slot_idx[1:0],param_sel[2:0],ch_sel[2:0]}。
    param_sel 0-5 用 PARAM_TUNING_WORD 等既有常數（跟 T_SINE_CTRL 共用
    同一組編碼）；param_sel=SINE_LIST_PARAM_COMMIT_DEPTH(6) 時語意是
    「commit list_depth 並 arm」，slot_idx 這時被忽略、value 是新的
    list_depth（不是參數值）。
    """
    if not (0 <= ch <= 7):
        raise ValueError("ch must be 0-7 (physical channel)")
    if not (0 <= slot_idx < SINE_N_SLOT):
        raise ValueError(f"slot_idx must be 0-{SINE_N_SLOT - 1} (N_SLOT — see rtl/sine_ctrl_regs.v)")
    if not (0 <= param_sel <= 7):
        raise ValueError("param_sel must be 0-7")
    sel = ((slot_idx & 0x3) << 6) | ((param_sel & 0x7) << 3) | (ch & 0x7)
    send_pkt(fp, dest_id, T_SINE_LIST_CTRL, beat1_lo=value, beat1_hi=sel)
    time.sleep(0.01)  # 比照 send_timer_ctrl()/send_group_sched_ctrl() 留的
    # CDC settling time（sel/data 是 level_cdc，wr 是 trigger_cdc，見
    # create_bd.tcl sine_list_*_cdc_0 對應章節）。


def set_sine_slot(fp, dest_id, ch, slots):
    """幫一個 physical channel（ch=0-7）寫入完整的 N-slot 排程表並 commit
    （arm）。

    slots：list of dict，1-SINE_N_SLOT 個元素，每個 dict 是一個 slot 的
    6 個參數：{'tuning_word', 'phase', 'start_amp', 'step',
    'duration_cycles', 'loop_mode'}（值的單位/編碼跟既有 T_SINE_CTRL
    單值寫入完全一樣，沒有另外定義新格式）。

    寫入順序完全比照 rtl/sine_ctrl_regs.v 檔頭的 hard contract：先把
    每個 slot 的 6 個參數都寫完，最後才送 commit——呼叫端不用自己排
    順序，這裡已經處理好。commit 之後，第一次 trigger 就會立刻讓
    active 側顯示 slot 0（不需要额外的「暖機」trigger），之後每次
    trigger 依序揭露 slot 1,2,...,N-1,0,1,...循環，見
    rtl/sine_ctrl_regs.v 檔頭「2. 新增 per-channel N-slot table」段落。

    （2026-08-05 曾經在這裡加過一個 depth==1 host-side padding
    workaround——當時 RTL 的 commit 邏輯對 depth==1 處理不完整，會讓
    下一次 trigger 洩漏內部 slot table 殘留的舊值，見 NOTES.md
    2026-08-05「depth==1 bug 精確定位」章節。**2026-08-06 RTL 修法
    上機驗證通過後已移除**——`test_sine_nslot_depth1_rtl.py` 直接
    繞過這層（曾經有的）workaround，重現原始受控測試步驟，確認
    depth==1 commit 之後不管觸發幾次都不會再洩漏殘留值。）
    """
    if not (1 <= len(slots) <= SINE_N_SLOT):
        raise ValueError(f"slots must have 1-{SINE_N_SLOT} entries (N_SLOT — see rtl/sine_ctrl_regs.v)")

    param_keys = [
        ('tuning_word', PARAM_TUNING_WORD),
        ('phase', PARAM_PHASE),
        ('start_amp', PARAM_START_AMP),
        ('step', PARAM_STEP),
        ('duration_cycles', PARAM_DURATION_CYCLES),
        ('loop_mode', PARAM_LOOP_MODE),
    ]
    for slot_idx, slot in enumerate(slots):
        for key, param_sel in param_keys:
            if key not in slot:
                raise ValueError(f"slot {slot_idx} missing '{key}'")
            send_sine_list_ctrl(fp, dest_id, ch, slot_idx, param_sel, slot[key])

    # commit / arm——必須是最後一步（見上方 docstring/rtl 檔頭 hard
    # contract），slot_idx 這時無意義，固定傳 0。
    send_sine_list_ctrl(fp, dest_id, ch, 0, SINE_LIST_PARAM_COMMIT_DEPTH, len(slots))


# ── group_trig_scheduler（trigger group 排程，2026-08-04 新增）─────────
# rtl/group_trig_scheduler.v：每片板子一個（不像 trig_timer 有 4 個
# port），排程「在什麼時間廣播哪一個 trigger group」——跟 trig_timer
# 最主要的差異：(1) run=1 直接開始，不用等 first_trigger（這個模組
# 本身就是要「自己當觸發源」）；(2) 每個 slot 多帶一個 group_sel，
# fire 那一拍會廣播對應的 trigger group（重用既有 T_TRIG_START 那套
# 已驗證的多板 PAUSE/ACK/GO 握手機制，不需要新的 Aurora 傳送層）。
def send_group_sched_ctrl(fp, dest_id, slot, depth, loop_en, run, group_sel, interval_cycles,
                           arm_mode=False):
    """T_GROUP_SCHED_CTRL(0x2C)，3-beat：beat1[3:0]=slot(0-15)
    [8:4]=depth(0-16) [9]=loop_en [10]=run [14:11]=group_sel(0-15)
    [15]=arm_mode（2026-08-20 新增，見下方）beat2[31:0]=interval_cycles
    （dac_clk cycles）。每次呼叫寫入一個 slot，同時（重新）宣告
    depth/loop_en/run/arm_mode——寫入完整排程要呼叫多次（每個 slot
    一次），run 通常只在最後一次呼叫才設 1。

    arm_mode（2026-08-20 新增，多板同步輪播 Architecture B，見
    rtl/group_trig_scheduler.v 檔頭說明/PROJECT.md「test2/test3 多板
    同步」章節）：False（預設，行為完全不變）＝ run 上升緣直接開始倒數
    （Architecture A，每次切換都走跨板 PAUSE/ACK/GO 廣播，上機實測過
    有 trigger-to-trigger jitter，±10-60ns 量級，改版前實測值）；
    True＝ run 上升緣只進 armed，等下一次 `/api/trigger` 廣播（跟現有
    Manual Trigger/test1 Run 同一條路徑）才真正開始倒數，之後每片板子
    各自在共用的 dac_clk 上倒數，不再繞 Aurora ring（Architecture B，
    誤差性質從「每次隨機」變成「整輪固定、可校準掉的 DC offset」）。
    """
    if not (0 <= slot <= 15):
        raise ValueError("slot must be 0-15")
    if not (0 <= depth <= 16):
        raise ValueError("depth must be 0-16 (5-bit field — see rtl/group_trig_scheduler.v)")
    if not (0 <= group_sel <= 15):
        raise ValueError("group_sel must be 0-15 (4-bit field)")
    beat1 = ((1 if arm_mode else 0) << 15) | ((group_sel & 0xF) << 11) \
        | ((1 if run else 0) << 10) | ((1 if loop_en else 0) << 9) \
        | ((depth & 0x1F) << 4) | (slot & 0xF)
    send_pkt(fp, dest_id, T_GROUP_SCHED_CTRL, beat1_lo=beat1,
              extra_beats=[(interval_cycles & 0xFFFFFFFF, 0)])
    time.sleep(0.01)  # 比照 send_timer_ctrl() 留的 settling time，同一類
    # level_cdc（run/loop_en/depth/arm_mode）+ trigger_cdc（list_wr_en pulse）組合


def set_group_sched(fp, dest_id, schedule, loop_en, run, arm_mode=False):
    """幫一片板子的 group_trig_scheduler 寫入完整排程。schedule：
    [(interval_cycles, group_sel), ...] 清單，1-16 筆。逐一送
    T_GROUP_SCHED_CTRL 寫進對應 slot，只有最後一次呼叫的封包才真正把
    run bit 設成呼叫端要的值——這樣期間 run 保持 False，避免拿還沒寫完
    的排程去 arm。

    arm_mode=False（預設，Architecture A，行為完全不變）：run=True 時
    group_trig_scheduler 立刻開始跑（不用等 /api/trigger，這是跟
    trig_timer 最主要的行為差異）。
    arm_mode=True（2026-08-20 新增，多板同步輪播 Architecture B，見
    send_group_sched_ctrl() docstring）：run=True 時只進 armed，呼叫端
    要另外送一次既有的廣播 trigger（如 trigger_playback()）才會真正
    開始倒數——那次廣播同時也是「真正開始播放」的那次觸發（DDR play_pos
    從未觸發狀態推進到 Set 1），所以第一組會立刻開始播放，沒有額外的
    開場靜音空檔（跟 arm_mode=False 那種「slot 0 身兼開場靜音+最後一組
    時長」的既有行為不同，見 PROJECT.md「test2/test3 多板同步」章節）。

    兩種模式 run=False 都會立即停止（不管當下在跑到哪個 slot）。
    schedule 為空只送一個 depth=0 的 control-only 封包（單純拿來下
    run=False 停止用）。

    depth==1 時 slot 0 同時是「唯一一筆」也是「is_last」，如果跟
    depth>=2 一樣把 run=True 併進同一個封包送，slot 0 的資料寫入
    （list_wr_en，走 trigger_cdc pulse CDC）跟 run 的上升緣（走
    level_cdc）會在 dac_clk domain 幾乎同時抵達，group_trig_scheduler.v
    的 `countdown <= mem_intv[0]` 可能搶在寫入生效前讀到舊值（接近 0），
    造成幾乎瞬間觸發，而不是等滿設定的 interval（2026-08-05 上機測試
    重現、經 Opus 覆核確認為必然發生、非邊緣時序 skew，見
    NOTES.md「Web UI 加入 trig_timer/group_trig_scheduler」章節）。
    depth>=2 沒有這個問題，因為 slot 0 一定在比 run=True 的最後封包更早
    的封包寫入，早就穩定。修法：depth==1 且 run=True 時拆成兩個封包送
    ——先寫 slot 0 資料（run=False），等 settle，再送第二個封包（同一份
    slot 0 資料，只是 run=True）。第二次重送同樣的 slot/interval/
    group_sel 不會讓 CDC bus 產生「值變化」，是安全的重寫。
    """
    depth = len(schedule)
    if run and not (1 <= depth <= 16):
        raise ValueError("schedule must have 1-16 entries when run=True")
    if depth == 0:
        send_group_sched_ctrl(fp, dest_id, slot=0, depth=0, loop_en=loop_en, run=run,
                               group_sel=0, interval_cycles=0, arm_mode=arm_mode)
        return
    if depth == 1 and run:
        # depth==1 race workaround（見上方 docstring）只在 arm_mode=False
        # 時真的有必要——arm_mode=True 的 run 上升緣只設 armed，不會在
        # 同一拍讀 mem_intv[0]（那個讀取延後到 first_trigger 那個完全
        # 獨立、之後才送的動作），沒有這個 race。但兩次送同樣的資料是
        # 無害的（CDC bus 沒有值變化），不特別為 arm_mode=True 分支省略
        # 這個保護，維持單一、簡單好懂的行為路徑。
        intv, group_sel = schedule[0]
        send_group_sched_ctrl(fp, dest_id, slot=0, depth=depth, loop_en=loop_en, run=False,
                               group_sel=group_sel, interval_cycles=intv, arm_mode=arm_mode)
        send_group_sched_ctrl(fp, dest_id, slot=0, depth=depth, loop_en=loop_en, run=True,
                               group_sel=group_sel, interval_cycles=intv, arm_mode=arm_mode)
        return
    for slot, (intv, group_sel) in enumerate(schedule):
        is_last = (slot == depth - 1)
        send_group_sched_ctrl(fp, dest_id, slot=slot, depth=depth, loop_en=loop_en,
                               run=(run if is_last else False), group_sel=group_sel,
                               interval_cycles=intv, arm_mode=arm_mode)


def arm_waveform(fp, dest_id, freqs_hz, channels, amp=DEFAULT_AMP):
    """上膛 len(freqs_hz) 個 slot（2~8），每個 slot 用 wave_len_for_freq()
    算出精確對應 freqs_hz[i] 的 buffer 長度，對 channels 逐一 LIST_WRITE，
    最後送 T_PLAY_CTRL 設 depth+play_en。回傳實際上膛結果（含 wave_len_
    for_freq 算出的實際頻率，可能跟輸入值有些微誤差）。停在 ARMED、還沒
    trigger。"""
    n_slots = len(freqs_hz)
    if not (2 <= n_slots <= MAX_SLOTS):
        raise ValueError(f"slot count must be between 2 and {MAX_SLOTS}, got {n_slots}")
    if not channels:
        raise ValueError("channels must not be empty")

    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=0)  # play_en <- 0 先關掉，避免上膛過程中撞到還在播放的舊資料

    slot_addr = SLOT0_ADDR
    slots = []
    for freq_hz in freqs_hz:
        n, k = wave_len_for_freq(freq_hz)
        samples = pad_to_burst(_gen_sine_ch1(n, amp, n / k))
        actual_hz = k * DAC_FS / n
        write_wave(fp, dest_id, slot_addr, samples)
        slots.append({"addr": slot_addr, "n_samples": len(samples),
                      "requested_hz": freq_hz, "actual_hz": actual_hz})
        slot_addr += len(samples) * 4 + SLOT_MARGIN

    for ch in channels:
        for i, s in enumerate(slots):
            send_list_write(fp, dest_id, ch, i, s["addr"], s["n_samples"])
            time.sleep(0.01)

    depth_val = sum((n_slots & 0x7) << (ch * 3) for ch in channels)
    play_val = sum(1 << ch for ch in channels)
    au_play_ctrl = (depth_val << 4) | play_val
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=au_play_ctrl)
    time.sleep(0.05)

    return {"dest_id": dest_id, "channels": channels, "slots": slots}


def arm_waveform_from_files(fp, dest_id, channel_slot_values, amp=DEFAULT_AMP):
    """channel_slot_values: {channel(0-3): [[v1, v2, ...], [v1, v2, ...], ...]}
    ——每個 channel 各自 2-8 個 slot，每個 slot 是使用者上傳檔案解析出的
    振幅值清單（-1.0~1.0，一行一個樣本）。DDR 播放的是使用者放進去的
    任意波形資料，不應該被迫是 host 端算出的特定頻率正弦波
    （2026-08-03，per 使用者回饋——取代先前 arm_waveform_per_channel()
    的「輸入頻率、host 算正弦波」設計，Sine 模式才有頻率概念）。

    跟 arm_waveform()/舊版 arm_waveform_per_channel() 同一套封包序列：
    T_PLAY_CTRL 關閉 -> 逐 slot 用 T_WAVEFORM_STREAM 寫 DDR4（pad_to_
    burst() 補靜音到 256 倍數——上傳資料是任意波形不是週期性正弦波，
    尾端補靜音是正確的做法，不用像 wave_len_for_freq() 那樣算精確週期
    長度）-> 逐 slot 用 T_LIST_WRITE 指到剛寫入的位址 -> T_PLAY_CTRL
    設 depth+play_en 開播放。DDR4 位址依序往後配置（across 全部
    channel 的全部 slot），確保不同 channel 的 buffer 不會互相重疊；
    depth 欄位本來就是 per-channel 3-bit，天生支援每個 channel 各自
    不同的 slot 數量。"""
    if not channel_slot_values:
        raise ValueError("channel_slot_values must not be empty")
    for ch, slots_values in channel_slot_values.items():
        if not (2 <= len(slots_values) <= MAX_SLOTS):
            raise ValueError(f"channel {ch}: slot count must be between 2 and {MAX_SLOTS}, got {len(slots_values)}")
        for values in slots_values:
            if not values:
                raise ValueError(f"channel {ch}: an uploaded slot file is empty")

    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=0)  # play_en <- 0 先關掉，避免上膛過程中撞到還在播放的舊資料

    channel_slots = {}
    for ch, slots_values in channel_slot_values.items():
        slot_addr = _module_base_addr(ch)  # 每個 module 自己的固定區段起算，不跟其他 module 共用/累加
        slots = []
        for values in slots_values:
            samples = pad_to_burst(_gen_from_samples(amp, values))
            _check_slot_fits(ch, slot_addr, len(samples))
            write_wave(fp, dest_id, slot_addr, samples)
            slots.append({"addr": slot_addr, "n_samples": len(samples), "n_uploaded": len(values)})
            slot_addr += len(samples) * 4 + SLOT_MARGIN
        channel_slots[ch] = slots

    for ch, slots in channel_slots.items():
        for i, s in enumerate(slots):
            send_list_write(fp, dest_id, ch, i, s["addr"], s["n_samples"])
            time.sleep(0.01)

    depth_val = sum((len(slots) & 0x7) << (ch * 3) for ch, slots in channel_slots.items())
    play_val = sum(1 << ch for ch in channel_slots.keys())
    au_play_ctrl = (depth_val << 4) | play_val
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=au_play_ctrl)
    time.sleep(0.05)

    return {"dest_id": dest_id, "channels": list(channel_slots.keys()), "channel_slots": channel_slots}


# ── Stereo (ch1/ch2 independent) test pattern（2026-08-07 新增）──────────
# dac_output_mux.v 檔頭已經記錄過：「DDR4 already supports independent
# ch1/ch2 content -- the host packs both channels into the sample stream
# it writes to DDR4 itself, no RTL change needed here.」（PORTS.md,
# user-confirmed 格式：ch1<-tdata[31:18], ch2<-tdata[15:2]）——但
# arm_waveform_from_files()/_gen_from_samples() 上面這條既有路徑只用
# _pack_ch1_only()，同一個 module 的 ch1/ch2 一直都被迫寫入一樣的內容，
# 從沒真的用過這個能力。這裡補上：N 階 DC 階梯測試圖案，每個 physical
# channel（0-7）依自己的 index 把階梯序列旋轉一階，讓同一時間點每個
# channel 剛好差一階（示波器可以一眼辨認哪條線是哪個 channel）。單一
# slot（depth=1）連續波形，不是多 slot 輪播——⚠️ waveform_controller.v
# 的 depth=1 路徑目前只有這次上機驗證過（board A/board B/board C 三片，2026-08-07），
# 不是像 sine_ctrl_regs.v 那個 2026-08-06 修過的 bug 一樣有長期回歸測試
# 覆蓋，之後如果行為異常要優先懷疑這裡。
def _pack_ch1_ch2(v1, v2):
    """dac_output_mux.v: ch1 <- tdata[31:18], ch2 <- tdata[15:2]（PORTS.md,
    user-confirmed）。v1/v2 是縮放過的整數（-8192~8191，跟 _pack_ch1_only
    同一種 14-bit 二補數編碼）。"""
    return ((int(v1) & 0x3FFF) << 18) | ((int(v2) & 0x3FFF) << 2)


# 2026-08-18 新增：arm_waveform_from_files() 的 channel_slot_values 其實是
# per-module（0-3，即 z0-z3）而不是 per physical channel——list_wr_slot 的
# ch 欄位只有 2 bit（send_list_write() 的 ch&0x3），硬體本來就沒有「指定
# 實體聲道」這回事，實體聲道是靠 _pack_ch1_only()/_pack_ch1_ch2() 決定
# 要把樣本塞進 32-bit word 的哪一半。arm_waveform_from_files() 固定用
# _pack_ch1_only()，該 module 的 ch2 永遠是 0V（已知限制，見 PROJECT.md
# 2026-08-14「Multitone Notch Scan」章節）。這裡補一個等效版本，用
# _pack_ch1_ch2(v, v) 讓同一 module 的 ch1/ch2 都輸出同一份內容——跟
# arm_multitone_pattern() 同一種「兩聲道灌一樣的東西」處理方式，只是
# 這裡接受任意 2-8 個 slot 的任意振幅序列，不是固定的梳狀波形。
def arm_waveform_stereo_same(fp, dest_id, module_slot_values, amp=DEFAULT_AMP):
    """module_slot_values: {module(0-3): [[v1, v2, ...], [v1, v2, ...], ...]}
    ——每個 module 各自 2-8 個 slot，每個 slot 是 -1.0~1.0 振幅值清單，
    跟 arm_waveform_from_files() 同一種輸入格式，唯一差異是 key 是
    module index（不是 physical channel），且該 module 的 ch1/ch2 兩個
    實體聲道會輸出同一份內容。"""
    if not module_slot_values:
        raise ValueError("module_slot_values must not be empty")
    for m, slots_values in module_slot_values.items():
        if not (0 <= m <= 3):
            raise ValueError(f"module must be 0-3, got {m}")
        if not (2 <= len(slots_values) <= MAX_SLOTS):
            raise ValueError(f"module {m}: slot count must be between 2 and {MAX_SLOTS}, got {len(slots_values)}")
        for values in slots_values:
            if not values:
                raise ValueError(f"module {m}: an uploaded slot is empty")

    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=0)  # play_en <- 0 先關掉，避免上膛過程中撞到還在播放的舊資料

    module_slots = {}
    for m, slots_values in module_slot_values.items():
        slot_addr = _module_base_addr(m)  # 每個 module 自己的固定區段起算，不跟其他 module 共用/累加
        slots = []
        for values in slots_values:
            samples = pad_to_burst([_pack_ch1_ch2(int(amp * v), int(amp * v)) for v in values])
            _check_slot_fits(m, slot_addr, len(samples))
            write_wave(fp, dest_id, slot_addr, samples)
            slots.append({"addr": slot_addr, "n_samples": len(samples), "n_uploaded": len(values)})
            slot_addr += len(samples) * 4 + SLOT_MARGIN
        module_slots[m] = slots

    for m, slots in module_slots.items():
        for i, s in enumerate(slots):
            send_list_write(fp, dest_id, ch=m, slot=i, addr=s["addr"], wave_len=s["n_samples"])
            time.sleep(0.01)

    # au_play_ctrl 是四個 module 共用的單一 32-bit 暫存器（見
    # rtl/aurora_ctrl_mux.v 的 play_ctrl_hold <= au_play_ctrl，整個覆蓋
    # 不是逐 bit 設定）——這裡只把 module_slots 有出現的 module 的
    # depth/play_en bit 設起來，其餘 module（不管原本在不在播）的
    # depth/play_en 會被這次寫入一併清成 0，是這顆暫存器架構本身的
    # 限制，呼叫端要注意這個副作用（跟 arm_waveform_from_files()/
    # arm_stereo_staircase()/arm_multitone_pattern() 完全一致，不是這個
    # 函式獨有）。
    depth_val = sum((len(slots) & 0x7) << (m * 3) for m, slots in module_slots.items())
    play_val = sum(1 << m for m in module_slots.keys())
    au_play_ctrl = (depth_val << 4) | play_val
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=au_play_ctrl)
    time.sleep(0.05)

    return {"dest_id": dest_id, "modules": list(module_slots.keys()), "module_slots": module_slots}


# 2026-08-18 新增：DDR Slot Setting 卡片本來只能上傳檔案（2026-08-04
# 拿掉了直接輸入頻率的介面，理由是「DDR 播的是任意波形，不是頻率」，
# 跟 Sine 模式的頻率概念分開）。但「輸入幾個頻率，搭配 Trigger Timer
# 依序自動切換」這種用途（跟這次 900kHz/600kHz 示範一樣）對使用者
# 來說比較接近 Sine 模式的操作方式，只是底層走 DDR list 播放——這裡
# 補一個「輸入頻率」入口，跟上傳檔案並列，不取代上傳檔案（任意波形
# 仍然只能靠上傳）。用跟 arm_waveform_stereo_same() 同一套 module 位址
# 配置/邊界檢查/ch1+ch2 雙聲道編碼，只是內容來源改成
# wave_len_for_freq() 精確算出的正弦波，不是使用者上傳的任意樣本。
def arm_ddr_freq_slots(fp, dest_id, module_freqs_hz, amp=DEFAULT_AMP):
    """module_freqs_hz: {module(0-3): [freq_hz, freq_hz, ...]} ——每個
    module 2-8 個頻率（Hz，必須 >0），各自用 wave_len_for_freq() 算出
    精確對應的 buffer 長度，ch1/ch2 兩個實體聲道輸出同一份正弦波
    （_pack_ch1_ch2）。回傳每個 slot 實際算出的 (requested_hz,
    actual_hz)（wave_len_for_freq() 是離散逼近，可能跟輸入值有些微
    誤差，跟 arm_waveform()/test_awg_arm_2slot_fpddr4.py 的既有回報
    方式一致）。停在 ARMED，還沒 trigger。"""
    if not module_freqs_hz:
        raise ValueError("module_freqs_hz must not be empty")
    for m, freqs in module_freqs_hz.items():
        if not (0 <= m <= 3):
            raise ValueError(f"module must be 0-3, got {m}")
        if not (2 <= len(freqs) <= MAX_SLOTS):
            raise ValueError(f"module {m}: freq count must be between 2 and {MAX_SLOTS}, got {len(freqs)}")
        for f in freqs:
            if not isinstance(f, (int, float)) or f <= 0:
                raise ValueError(f"module {m}: each frequency must be a positive number")

    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=0)  # play_en <- 0 先關掉，避免上膛過程中撞到還在播放的舊資料

    module_slots = {}
    for m, freqs in module_freqs_hz.items():
        slot_addr = _module_base_addr(m)  # 每個 module 自己的固定區段起算，不跟其他 module 共用/累加
        slots = []
        for freq_hz in freqs:
            n, k = wave_len_for_freq(freq_hz)
            actual_hz = k * DAC_FS / n
            samples = pad_to_burst([_pack_ch1_ch2(int(amp * math.sin(2 * math.pi * k * i / n)),
                                                   int(amp * math.sin(2 * math.pi * k * i / n)))
                                     for i in range(n)])
            _check_slot_fits(m, slot_addr, len(samples))
            write_wave(fp, dest_id, slot_addr, samples)
            slots.append({"addr": slot_addr, "n_samples": len(samples),
                          "requested_hz": freq_hz, "actual_hz": actual_hz})
            slot_addr += len(samples) * 4 + SLOT_MARGIN
        module_slots[m] = slots

    for m, slots in module_slots.items():
        for i, s in enumerate(slots):
            send_list_write(fp, dest_id, ch=m, slot=i, addr=s["addr"], wave_len=s["n_samples"])
            time.sleep(0.01)

    # au_play_ctrl 是四個 module 共用的單一暫存器，見 arm_waveform_
    # stereo_same() 上方同一則說明——只送這裡涵蓋的 module 一樣會把
    # 其餘 module 的 play_en 一併清成 0。
    depth_val = sum((len(slots) & 0x7) << (m * 3) for m, slots in module_slots.items())
    play_val = sum(1 << m for m in module_slots.keys())
    au_play_ctrl = (depth_val << 4) | play_val
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=au_play_ctrl)
    time.sleep(0.05)

    return {"dest_id": dest_id, "modules": list(module_slots.keys()), "module_slots": module_slots}


# 2026-08-20 新增（Control 分頁「test1」面板首個實際功能）：DC 固定電壓
# （6 個實體聲道，z0-z2）+ RF 各自獨立頻率（2 個實體聲道，z3）混合上膛。
# 跟上面 arm_waveform_stereo_same()/arm_ddr_freq_slots() 不同的地方只有
# 一個——那兩個函式都是「module 的 ch1/ch2 播同一份內容」（_pack_ch1_
# ch2(v,v)），這裡兩個實體聲道要各自獨立（DC 各自獨立電壓、RF 各自獨立
# 頻率），改用 _pack_ch1_ch2(v1,v2) 帶不同的 v1/v2——封包格式本身
# 2026-08-07 的階梯測試圖案已經上機驗證過兩聲道獨立內容沒問題，這裡只是
# 第一次在一般功能（不是一次性診斷腳本）用到這個能力。
#
# DC+RF 一定要同一次呼叫處理完：au_play_ctrl 是 4 個 module 共用的單一
# 暫存器，見 arm_waveform_stereo_same() 上方說明，分開呼叫會讓後一次
# 呼叫沒涵蓋到的 module 被清成 play_en=0。
#
# 固定用 2 個內容完全相同的 slot，不用 depth=1——depth=1 這條路徑目前
# 只有 2026-08-07 那次階梯測試圖案驗證過，測試覆蓋不足，見 PROJECT.md
# 2026-08-20「test1 面板」章節。內容完全相同的 2 個 slot，Trigger 切換
# 時電壓/頻率不會有任何變化，效果上等同穩定輸出，同時沿用已經大量
# 驗證過的多 slot 路徑。
TEST1_DC_SLOT_LEN = 256  # 已經是 BURST_SAMPLES 的倍數，pad_to_burst() 是 no-op


def _dc_ramp_cycle_length(spec, m, label):
    """算出一個 ramp DC channel 的 sawtooth 週期需要幾個 sample：從
    start_ratio 開始每個 sample 累加 slope_ratio_per_sample，直到下一步
    會超出 [-1.0, 1.0] 為止（不含超出的那一步）。回傳的長度直接當
    T_LIST_WRITE 的 wave_len 送出（不是 pad_to_burst() 補完後的長度）
    ——2026-08-21 查證確認 ddr4_stream_reader.v 本來就支援 wave_len 是
    任意值，會用 `fifo_we <= (samp_cnt < wave_len_r)` 擋掉超過 wave_len
    的補值樣本（該檔案 9-10/189-190 行），所以「碰到滿幅瞬間跳回起始
    值」這個 sawtooth 效果完全靠這個既有機制達成，不需要額外的迴圈/
    reset 邏輯，尾端也不會有 pad_to_burst() 補值造成的 0V 空隙（既有的
    arm_stereo_staircase()/arm_multitone_pattern() 都沒有利用這個能力，
    一律把補完後的長度當 wave_len 送出，這裡是這個專案第一次真正用到）。
    """
    start_ratio = spec["start_ratio"]
    slope_ratio_per_s = spec["slope_ratio_per_s"]
    if slope_ratio_per_s == 0:
        raise ValueError(f"module {m} {label}: ramp slope must not be 0 (use fixed mode instead)")
    slope_ratio_per_sample = slope_ratio_per_s / DAC_FS
    limit = 1.0 if slope_ratio_per_sample > 0 else -1.0
    remaining = limit - start_ratio
    if remaining / slope_ratio_per_sample <= 0:
        raise ValueError(
            f"module {m} {label}: start ratio {start_ratio:+.4f} is already at/beyond "
            f"the {limit:+.1f} limit for this slope direction")
    n = math.floor(remaining / slope_ratio_per_sample) + 1
    max_n = MODULE_ADDR_STRIDE // 4  # 16MB region / 4 bytes per sample
    if n > max_n:
        min_slope_ratio_per_s = abs(remaining) * DAC_FS / max_n
        raise ValueError(
            f"module {m} {label}: slope too small — ramp cycle needs {n} samples, "
            f"exceeds this module's {max_n}-sample (16MB) region; "
            f"|slope_ratio_per_s| must be at least {min_slope_ratio_per_s:.6f}")
    return n


def _dc_channel_codes(spec, amp, buf_len):
    """把一個 DC channel 的 spec 轉成長度 buf_len 的 DAC code 清單（未
    pack，14-bit 二補數範圍）。spec 是 float（fixed ratio，重複 buf_len
    次）或 {"mode": "ramp", "start_ratio": ..., "slope_ratio_per_s": ...}
    （ramp，buf_len 必須等於 _dc_ramp_cycle_length() 算出的值，呼叫端
    負責保證）。"""
    if isinstance(spec, dict):
        start_ratio = spec["start_ratio"]
        slope_ratio_per_sample = spec["slope_ratio_per_s"] / DAC_FS
        return [int(round(amp * (start_ratio + slope_ratio_per_sample * i))) for i in range(buf_len)]
    return [int(amp * spec)] * buf_len


def _slot_max_samples(n_slots):
    """一個 module 的 16MB 定址區段（MODULE_ADDR_STRIDE）要放 n_slots 個
    內容不同、長度相同的 slot（每個 slot 之間有 SLOT_MARGIN 間隔）時，
    單一 slot 最多能有幾個 sample——2026-08-21 新增，給 wave_len_for_
    two_freqs()/wave_len_for_comb_plus_freq() 當 max_samples 用（Opus
    分析抓到的既有 bug：先前這兩個函式的呼叫都直接用通用預設值，沒有
    照實際 slot 數換算，n_slots 較多時會讓 _check_slot_fits() 在寫入
    階段才報錯，不是在搜尋階段就用正確的上限）。公式來自 _module_
    base_addr()/_check_slot_fits() 的位址算法：第 i 個 slot（0-indexed）
    位址 = base + i*(4*L+SLOT_MARGIN)，最後一個 slot 結尾要 ≤ region_end
    ——即 n_slots*4*L + (n_slots-1)*SLOT_MARGIN ≤ MODULE_ADDR_STRIDE。"""
    return (MODULE_ADDR_STRIDE - (n_slots - 1) * SLOT_MARGIN) // (4 * n_slots)


def _rf_ifft_mixed_codes(m, ch1, ch2, amp, max_samples):
    """test1/test2/test3 共用的 RF「至少一個 channel 是 IFFT」邏輯
    （2026-08-21 從 arm_test1_pattern()/arm_rotation_pattern() 抽出來的
    共用 helper，Opus 建議——原本兩處各自維護一份幾乎相同的程式碼，
    這次順便修掉 IFFT+Single 混用的精確度問題，兩處只需要改一次）。

    兩個 channel 都是 IFFT 時走原本的 compute_comb_bins() 路徑（buffer
    長度純粹由 IFFT 頻率結構決定，不受這次改動影響）；混一個 Single
    頻率時改用 wave_len_for_comb_plus_freq()（見該函式說明——buffer
    長度不再要求是 IFFT 自然週期的整數倍，讓 Single 頻率也有精確度
    保證，取代原本「單次 round() 湊整數週期、沒有精確度保證」的做法）。

    回傳 (codes1, codes2, true_wave_len, info_extra)。"""
    ifft_specs = [c for c in (ch1, ch2) if c["mode"] == "ifft"]
    single_specs = [c for c in (ch1, ch2) if c["mode"] == "single"]
    ref = ifft_specs[0]
    if len(ifft_specs) == 2:
        other = ifft_specs[1]
        if (ref["start_hz"], ref["end_hz"], ref["step_hz"]) != \
           (other["start_hz"], other["end_hz"], other["step_hz"]):
            raise ValueError(
                f"module {m}: 兩個 channel 都選 IFFT 時 start_hz/end_hz/step_hz 必須完全相同"
                "（buffer 長度只能有一個，這是硬體限制）")

    info_extra = {}
    if single_specs:
        single_spec = single_specs[0]
        result = wave_len_for_comb_plus_freq(ref["step_hz"], single_spec["freq_hz"], max_samples)
        info_extra.update({
            "single_requested_hz": single_spec["freq_hz"],
            "single_actual_hz": result["single_actual_hz"],
            "single_ppm": result["single_ppm"],
            "comb_requested_step_hz": ref["step_hz"],
            "comb_actual_step_hz": result["step_actual_hz"],
            "comb_step_ppm": result["step_ppm"],
            "degraded": result["degraded"],
        })
        n, actual_step_hz, bins, w, phis, bin_hz = compute_comb_bins(
            ref["start_hz"], ref["end_hz"], ref["step_hz"],
            n=result["n"], bin_stride=result["bin_stride"])
        k_single = result["k_single"]
    else:
        n, actual_step_hz, bins, w, phis, bin_hz = compute_comb_bins(
            ref["start_hz"], ref["end_hz"], ref["step_hz"])
        k_single = None
        info_extra.update({"comb_n_bins": len(bins), "comb_actual_step_hz": actual_step_hz})

    def _channel_codes(spec_c):
        if spec_c["mode"] == "ifft":
            x = render_comb_waveform(n, bins, w, phis, bin_hz,
                                      spec_c.get("exclude_start_hz"),
                                      spec_c.get("exclude_end_hz"))
            peak = np.max(np.abs(x))
            scale = 0.95 / peak if peak > 0 else 1.0
            return np.clip(np.round(amp * x * scale), -8192, 8191).astype(np.int64)
        else:
            idx = np.arange(n)
            x = np.sin(2 * np.pi * k_single * idx / n)
            return np.clip(np.round(amp * spec_c["amp_ratio"] * x), -8192, 8191).astype(np.int64)

    codes1 = _channel_codes(ch1)
    codes2 = _channel_codes(ch2)
    return codes1, codes2, n, info_extra


def arm_test1_pattern(fp, dest_id, dc_module_volts, rf_module_values, amp=DEFAULT_AMP):
    """dc_module_volts: {module(0-3): (spec_ch1, spec_ch2)}——每個 spec
    是 float（fixed，振幅比例 -1.0~1.0）或 {"mode": "ramp",
    "start_ratio": -1.0~1.0, "slope_ratio_per_s": <非0>}（2026-08-21
    新增，見 _dc_ramp_cycle_length() 說明；斜率正負決定上升/下降方向，
    碰到 ±1.0 滿幅時透過 wave_len 精確對齊自動跳回 start_ratio，形成
    sawtooth）。呼叫端（app.py）已經用該 channel 目前的 scale_cfg 換算過
    電壓，這裡不再碰電壓/校準邏輯，只認識 ratio 單位。同一個 module 的
    兩個 channel 可以各自獨立選 fixed/ramp；**兩個都選 ramp 時，算出來
    的週期長度必須完全相同**（同一個 module 的 ch1/ch2 共用同一個 DDR
    buffer，跟 RF 的 IFFT 雙 channel 同一種硬體限制），不同就報錯。

    rf_module_values: {module(0-3): {"ch1": <spec>, "ch2": <spec>}}——
    2026-08-20 改版，每個 channel 各自獨立選模式：
      單頻：{"mode": "single", "freq_hz": ..., "amp_ratio": -1.0~1.0}
      IFFT 梳狀波：{"mode": "ifft", "start_hz": ..., "end_hz": ...,
                    "step_hz": ..., "amp_ratio": -1.0~1.0,
                    "exclude_start_hz": ...（可選）, "exclude_end_hz": ...（可選）}
    兩個 channel 都是 single 時走原本的 wave_len_for_two_freqs() 雙頻
    打包路徑，行為完全不變。只要有一個是 ifft，就用 ifft 那個 channel
    的頻率結構（multitone_scan.compute_comb_bins()）決定 buffer 長度，
    single 的那個 channel（如果有）在同一個長度下重新算自己的整數週期
    ——這樣兩個 channel 才能共用同一個 DDR buffer（呼叫端已經用今天
    `wave_len_for_two_freqs()` 同樣的道理驗證過可行）。**兩個 channel
    都是 ifft 時，start_hz/end_hz/step_hz 三個值必須完全相同**（buffer
    長度只能有一個，這是硬體限制；exclude_start_hz/exclude_end_hz 不
    受此限制，各自獨立），呼叫端（app.py）先驗證過，這裡再檢查一次
    防禦。

    同一個 module 不能同時出現在兩個 dict 裡。停在 ARMED，還沒 trigger；
    呼叫端另外送一次 trigger()。"""
    if not dc_module_volts and not rf_module_values:
        raise ValueError("dc_module_volts and rf_module_values must not both be empty")
    overlap = set(dc_module_volts) & set(rf_module_values)
    if overlap:
        raise ValueError(f"module(s) {sorted(overlap)} appear in both dc_module_volts and rf_module_values")

    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=0)  # play_en <- 0 先關掉，避免上膛過程中撞到還在播放的舊資料

    module_slots = {}

    for m, (spec1, spec2) in dc_module_volts.items():
        for label, spec in (("ch1", spec1), ("ch2", spec2)):
            if not isinstance(spec, dict) and not (-1.0 <= spec <= 1.0):
                raise ValueError(f"module {m} {label}: amplitude ratio out of range (-1.0~1.0): {spec}")

        ramp_lens = {label: _dc_ramp_cycle_length(spec, m, label)
                     for label, spec in (("ch1", spec1), ("ch2", spec2)) if isinstance(spec, dict)}
        if len(ramp_lens) == 2 and ramp_lens["ch1"] != ramp_lens["ch2"]:
            raise ValueError(
                f"module {m}: both channels in ramp mode must resolve to the same buffer length "
                f"(ch1 needs {ramp_lens['ch1']} samples, ch2 needs {ramp_lens['ch2']}) — "
                f"adjust start/slope so both match, or set one channel to fixed mode")
        wave_len = next(iter(ramp_lens.values()), TEST1_DC_SLOT_LEN)

        codes1 = _dc_channel_codes(spec1, amp, wave_len)
        codes2 = _dc_channel_codes(spec2, amp, wave_len)
        one_slot = pad_to_burst([_pack_ch1_ch2(c1, c2) for c1, c2 in zip(codes1, codes2)])
        slot_addr = _module_base_addr(m)
        slots = []
        for _ in range(2):  # 2 個內容相同的 slot，見上方模組說明
            _check_slot_fits(m, slot_addr, len(one_slot))
            write_wave(fp, dest_id, slot_addr, one_slot)
            slots.append({"addr": slot_addr, "n_samples": len(one_slot), "wave_len": wave_len})
            slot_addr += len(one_slot) * 4 + SLOT_MARGIN
        module_slots[m] = {"kind": "dc", "slots": slots, "spec_ch1": spec1, "spec_ch2": spec2}

    for m, spec in rf_module_values.items():
        ch1, ch2 = spec["ch1"], spec["ch2"]
        # amp_ratio 只有 single 模式需要（使用者填實際振幅）——ifft 模式
        # 不用填，固定用 0.95 滿幅自動正規化（跟 arm_multitone_pattern()
        # 同一套邏輯），2026-08-20 使用者確認「ifft 看起來也不用設定
        # 振幅」拿掉這個欄位。
        for label, ch in (("ch1", ch1), ("ch2", ch2)):
            if ch["mode"] == "single":
                ra = ch["amp_ratio"]
                if not (-1.0 <= ra <= 1.0):
                    raise ValueError(f"module {m} {label}: amplitude ratio out of range (-1.0~1.0): {ra}")

        if ch1["mode"] == "single" and ch2["mode"] == "single":
            # 兩個 channel 都是單頻：原本的雙頻打包路徑，wave_len_for_
            # two_freqs() 2026-08-21 改版後 n 不一定是 BURST_SAMPLES 的
            # 倍數（見該函式 BURST_TOLERANCE 說明），true_wave_len 記錄
            # 真正長度，跟 pad_to_burst() 補完的寫入長度分開。
            f1, f2 = ch1["freq_hz"], ch2["freq_hz"]
            ra1, ra2 = ch1["amp_ratio"], ch2["amp_ratio"]
            n, k1, k2 = wave_len_for_two_freqs(f1, f2, max_samples=_slot_max_samples(2))
            true_wave_len = n
            actual_f1 = k1 * DAC_FS / n
            actual_f2 = k2 * DAC_FS / n
            one_slot = pad_to_burst([
                _pack_ch1_ch2(int(amp * ra1 * math.sin(2 * math.pi * k1 * i / n)),
                              int(amp * ra2 * math.sin(2 * math.pi * k2 * i / n)))
                for i in range(n)])
            info_extra = {"requested_hz": [f1, f2], "actual_hz": [actual_f1, actual_f2]}
        else:
            # 至少一個 channel 是 IFFT：共用 helper（跟 arm_rotation_
            # pattern() 共用同一份邏輯，2026-08-21 抽出來，見該函式說明）。
            max_samples = _slot_max_samples(2)  # 這裡固定寫 2 個 slot
            codes1, codes2, true_wave_len, info_extra = _rf_ifft_mixed_codes(
                m, ch1, ch2, amp, max_samples)
            one_slot = pad_to_burst([_pack_ch1_ch2(int(c1), int(c2))
                                      for c1, c2 in zip(codes1, codes2)])

        slot_addr = _module_base_addr(m)
        slots = []
        for _ in range(2):  # 2 個內容相同的 slot，見上方模組說明
            _check_slot_fits(m, slot_addr, len(one_slot))
            write_wave(fp, dest_id, slot_addr, one_slot)
            # wave_len 跟 n_samples（pad_to_burst() 補完後的長度）2026-08-21
            # 起不一定相同：兩個 channel 都是 single 頻率時
            # wave_len_for_two_freqs()、IFFT 混 single 時
            # wave_len_for_comb_plus_freq() 都允許 N 落在 BURST_TOLERANCE
            # 範圍內不是剛好 256 倍數——一律送 true_wave_len（真正長度）
            # 當 wave_len，讓 RTL 的 fifo_we<=(samp_cnt<wave_len_r) 精確
            # 擋掉補值尾端，不會播出多餘的 0V 樣本。
            slots.append({"addr": slot_addr, "n_samples": len(one_slot), "wave_len": true_wave_len})
            slot_addr += len(one_slot) * 4 + SLOT_MARGIN
        module_slots[m] = {"kind": "rf", "slots": slots, "ch1": ch1, "ch2": ch2, **info_extra}

    for m, info in module_slots.items():
        for i, s in enumerate(info["slots"]):
            send_list_write(fp, dest_id, ch=m, slot=i, addr=s["addr"], wave_len=s["wave_len"])
            time.sleep(0.01)

    # au_play_ctrl 是四個 module 共用的單一暫存器，見 arm_waveform_
    # stereo_same() 上方同一則說明——只送這裡涵蓋的 module 一樣會把
    # 其餘 module 的 play_en 一併清成 0。
    depth_val = sum((len(info["slots"]) & 0x7) << (m * 3) for m, info in module_slots.items())
    play_val = sum(1 << m for m in module_slots.keys())
    au_play_ctrl = (depth_val << 4) | play_val
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=au_play_ctrl)
    time.sleep(0.05)

    return {"dest_id": dest_id, "modules": {str(m): info for m, info in module_slots.items()}}


STEREO_STAIRCASE_MAX_CODE = 8191  # 14-bit 二補數正向滿幅（-8192~8191），跟 _pack_ch1_ch2 同一套編碼


def arm_stereo_staircase(fp, dest_id, modules, steps=8, amp_ratio=1.0, step_duration_sec=None,
                          start_level="neg_max"):
    """modules: 這片板子要套用的 module index 清單（0-3 的子集）。每個
    module 的 ch1/ch2（physical channel = 2*m/2*m+1）各自依自己的
    physical channel index 把 `steps` 階 DC 階梯旋轉一階。

    amp_ratio：0.0-1.0 的比例（1.0=14-bit 滿幅），跟 apply_sine_channel()
    的 amp ramp 是同一種「比例不是絕對電壓」的精神——這個專案目前沒有
    任何地方記錄過「DAC code 對應多少實際伏特」的換算公式（2026-08-07
    查證：只有 scale_cfg 兩檔滿幅量程 ±1.25V/±5V 是上機實測確認過的，
    14-bit code 逐值換算沒有依據），所以不提供「輸入 V」的介面，避免
    給一個沒驗證過的數字。

    start_level：階梯的最低那一階從哪裡開始（2026-08-10，per 使用者
    要求「可選的 -max 或是 0V」），最高那一階永遠是 +amp，只有最低點
    不同：
      "neg_max"（預設，原本唯一的行為）：均分 -amp ~ +amp（雙極性）
      "zero"：均分 0 ~ +amp（單極性，不輸出負電壓）

    step_duration_sec：每一階要維持多久（秒），換算成 samples 數
    （DAC_FS=100MHz）。None 時預設每階 256 samples（~2.56µs，剛好對齊
    burst 邊界）。換算出來的總長度不一定是 256 的倍數，用 pad_to_burst()
    在陣列最後補靜音對齊（不會打斷階梯本身的規律，只在結尾多一小段
    0V），跟 arm_waveform_from_files() 同一種既有處理方式。

    強制把套用到的 module 切回 DDR(0)——不能依賴殘留的 dac_mode_ramp
    狀態（2026-08-07 上機時踩過：只寫 DDR4 波形沒有切 mode，播的其實是
    殘留的 Sine 設定）。"""
    if steps < 2 or steps > 8:
        raise ValueError("steps must be between 2 and 8 (list_depth is a 3-bit field)")
    if not (0.0 <= amp_ratio <= 1.0):
        raise ValueError("amp_ratio must be between 0.0 and 1.0")
    if start_level not in ("neg_max", "zero"):
        raise ValueError(f"start_level must be 'neg_max' or 'zero', got {start_level!r}")
    seg_len = 256 if step_duration_sec is None else max(1, round(step_duration_sec * DAC_FS))
    for m in modules:
        if not (0 <= m <= 3):
            raise ValueError(f"module must be 0-3, got {m}")
        set_dac_mode(fp, dest_id, m, 0)

    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=0)  # stop playback while arming

    amp = amp_ratio * STEREO_STAIRCASE_MAX_CODE

    def level_int(step_idx):
        frac = step_idx / (steps - 1)  # 0.0 (bottom step) .. 1.0 (top step)
        low = 0.0 if start_level == "zero" else -1.0
        return int(amp * (low + (1.0 - low) * frac))

    def channel_sequence(phys_ch):
        return [(step_idx + phys_ch) % steps for step_idx in range(steps)]

    depth_val = 0
    play_val = 0
    for m in modules:
        slot_addr = _module_base_addr(m)  # 每個 module 自己的固定區段，不跟其他 module 共用/累加
        ch1, ch2 = 2 * m, 2 * m + 1
        seq1, seq2 = channel_sequence(ch1), channel_sequence(ch2)
        samples = []
        for step in range(steps):
            samples.extend([_pack_ch1_ch2(level_int(seq1[step]), level_int(seq2[step]))] * seg_len)
        samples = pad_to_burst(samples)
        _check_slot_fits(m, slot_addr, len(samples))
        write_wave(fp, dest_id, slot_addr, samples)
        send_list_write(fp, dest_id, ch=m, slot=0, addr=slot_addr, wave_len=len(samples))
        depth_val |= (1 & 0x7) << (m * 3)  # depth=1，單一連續波形，不是多 slot 輪播
        play_val |= (1 << m)

    au_play_ctrl = (depth_val << 4) | play_val
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=au_play_ctrl)
    time.sleep(0.05)
    return {"dest_id": dest_id, "modules": list(modules), "steps": steps,
            "amp_ratio": amp_ratio, "step_duration_sec": seg_len / DAC_FS,
            "start_level": start_level}


# ── Multitone 測試圖案（2026-08-14 新增）─────────────────────────────
#
# 500Hz~500kHz、500Hz 間距、1000 音 Schroeder phase 梳狀波形，見
# multitone_scan.precompute_full_sum()。原本走動態 notch 掃描（背景執行緒
# 逐步排除單一頻率、2-slot 乒乓上傳），但批次大小算式在實測上傳速度下
# 無法收斂，Pi 5B 上被 OOM killer 砍掉 process（詳見 NOTES.md）。改成
# 這裡：跟 arm_stereo_staircase() 同一種「單一連續波形，depth=1，一次性
# 上膛」寫法，不需要背景執行緒/動態批次上傳。同一個 module 的 ch1/ch2
# 灌一樣的內容（跟 arm_stereo_staircase() 不同，這裡不需要區分兩個實體
# 通道），上機示波器已確認頻譜正確（FFT 驗證 + 頻譜儀比對，2026-08-14）。
def arm_multitone_pattern(fp, dest_id, modules, amp_ratio=1.0, exclude_hz=None, module_exclude_hz=None):
    """modules: 這片板子要套用的 module index 清單（0-3 的子集），每個
    module 的 ch1/ch2 都輸出同一份 500Hz~500kHz 梳狀波形。

    amp_ratio：0.0-1.0 的比例（同 arm_stereo_staircase()，不是絕對電壓）。

    exclude_hz：2026-08-19 新增，要完全排除（振幅設為 0）的頻率清單
    （Hz），會被四捨五入到最近的諧波，套用到沒有出現在 module_exclude_hz
    裡的 module。**這是「host 端算好再上膛」版的 IFFT notch 排除頻率
    功能**——取代原本 FPGA 即時扣除的 notch_bank.v 設計（見 PROJECT.md
    2026-08-19 對應章節：實際使用情境是「選好要排除的頻率、算好、上膛
    播放，之後偶爾才改」，不需要真正即時無縫切換，換一批排除頻率大約
    1.7~2 秒——重新算波形+上傳，等同重新按一次這個函式），不受聲道數/
    BRAM 資源限制，8 聲道都能直接用。

    module_exclude_hz：2026-08-19 新增，dict {module_index: [Hz, ...]}，
    讓同一次呼叫裡不同 module 各自排除不同頻率，覆蓋該 module 的
    exclude_hz 預設值。**必須用這個參數而不是分開呼叫兩次**——
    au_play_ctrl 是四個 module 共用的單一暫存器，這個函式每次呼叫一
    開始都會先整個歸零（見下方 T_PLAY_CTRL beat1_lo=0），分開呼叫會讓
    後一次洗掉前一次的 play-enable，只剩最後呼叫的 module 在播放（上機
    示波器驗證時踩過這個坑，見 NOTES.md 2026-08-19「示波器上機驗證」
    章節）。

    N=200,000 樣本不是 256 的倍數，跟 arm_waveform_from_files() 一樣用
    pad_to_burst() 補尾端靜音對齊——會讓實際週期從精確的 2.000ms 變成
    2.00192ms（頻率間距從 500.00Hz 略偏到 499.52Hz），2026-08-14 上機
    頻譜儀比對確認這個誤差量級不影響梳狀結構可辨識度。"""
    if not (0.0 <= amp_ratio <= 1.0):
        raise ValueError("amp_ratio must be between 0.0 and 1.0")
    for m in modules:
        if not (0 <= m <= 3):
            raise ValueError(f"module must be 0-3, got {m}")
        set_dac_mode(fp, dest_id, m, 0)

    fund_hz = DAC_FS / MULTITONE_N
    module_exclude_hz = module_exclude_hz or {}

    def _bins_for(hz_list):
        bins = set()
        for hz in (hz_list or []):
            k = max(1, min(MULTITONE_N_TONES, round(hz / fund_hz)))
            bins.add(k)
        return bins

    # 2026-08-19 新增：套用增益校正表（放大器對不同頻率增益不同，見
    # multitone_scan.load_gain_correction() 說明），沒有校正檔案時
    # 回傳全部 1.0，完全不改變原本的波形。同一次呼叫內每個 module 各自
    # 的排除頻率可能不同，所以波形要逐 module 分開算（precompute_full_
    # sum() 本身很快，逐 module 重算沒有效能顧慮）。
    gain_table = load_gain_correction()
    amp = amp_ratio * STEREO_STAIRCASE_MAX_CODE

    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=0)  # stop playback while arming

    depth_val = 0
    play_val = 0
    excluded_by_module = {}
    for m in modules:
        exclude_bins = _bins_for(module_exclude_hz.get(m, exclude_hz))
        x_full, _, _, _, _ = precompute_full_sum(gain_table, exclude_bins)
        scale = 0.95 / np.max(np.abs(x_full))
        codes = np.clip(np.round(amp * x_full * scale), -8192, 8191).astype(np.int64)
        samples = pad_to_burst([_pack_ch1_ch2(int(c), int(c)) for c in codes])

        slot_addr = _module_base_addr(m)  # 每個 module 自己的固定區段，不跟其他 module 共用/累加
        _check_slot_fits(m, slot_addr, len(samples))
        write_wave(fp, dest_id, slot_addr, samples)
        send_list_write(fp, dest_id, ch=m, slot=0, addr=slot_addr, wave_len=len(samples))
        depth_val |= (1 & 0x7) << (m * 3)  # depth=1，單一連續波形，不是多 slot 輪播
        play_val |= (1 << m)
        excluded_by_module[m] = sorted(k * fund_hz for k in exclude_bins)

    au_play_ctrl = (depth_val << 4) | play_val
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=au_play_ctrl)
    time.sleep(0.05)
    all_excluded = sorted({hz for lst in excluded_by_module.values() for hz in lst})
    return {"dest_id": dest_id, "modules": list(modules), "amp_ratio": amp_ratio,
            "n_tones": 1000, "freq_range_hz": [500, 500000], "spacing_hz": 500,
            "excluded_hz": all_excluded,
            "excluded_hz_by_module": {str(m): v for m, v in excluded_by_module.items()}}


# 2026-08-18/19：IFFT 梳狀波形排除頻率功能最早是做成 FPGA 即時扣除
# （notch_bank.v/notch_sine_gen.v，host 端曾經有 set_notch_target()/
# clear_notch_target()/send_notch_ctrl()/_notch_tuning_word()/
# _notch_phase_seed() 這些函式），上機驗證通過後使用者確認實際使用
# 情境是「選好要排除的頻率、算好、上膛播放，之後偶爾才改」，不需要
# 真正即時無縫切換——改成 host 端直接在 arm_multitone_pattern() 的
# exclude_hz 參數裡排除（見上方），完全不需要額外的 FPGA 硬體/BRAM
# 資源，8 聲道都能直接用。RTL 端的 notch_bank.v/notch_sine_gen.v +
# aurora_ctrl_mux.v/local_reg_handler.v 的 T_NOTCH_SWEEP_CTRL(0x2E)
# 解碼已經整批拆除、重新 build+燒錄，這裡的 host 函式也一併移除，
# 完整過程見 NOTES.md 2026-08-19「IFFT notch 改回 host 端算好上膛」
# 章節。


def trigger(fp, group_select=0xF):
    """廣播 T_TRIG_START。協定本身就是 broadcast，只有 is_master 的板子
    會真的動作（trig_start_pulse && is_master），不是 per-board 定址，
    所以沒有 dest_id 參數。"""
    _flush_standby(fp)
    send_pkt(fp, DEST_BCAST, T_TRIG_START, beat1_lo=group_select & 0xF)


def read_ddr_status(fp, dest_id, wait=0.05):
    """T_QUERY(QT_DDR_STATUS)。query_type 不符時回傳 None（呼叫端可能要重試，
    通常是舊回覆還沒沖掉）。"""
    _flush_standby(fp)
    send_query(fp, dest_id, QT_DDR_STATUS)
    time.sleep(wait)
    _, got_query_type, data_words = read_status_reply(fp)
    if got_query_type != QT_DDR_STATUS:
        return None
    return decode_qt_ddr_status(data_words)


def read_sine_status(fp, dest_id, wait=0.05):
    """T_QUERY(QT_SINE_STATUS)：8 個 physical channel 目前 active 側
    （依 mux_sel_sync 選好）的 tuning_word 等 6 個參數（2026-08-05 新增
    ——這之前完全沒有 web app wrapper，Sine 模式切到之後反而比 DDR 少
    了所有可讀回狀態，Opus 扮演首次使用者測試時抓到的落差）。回傳
    `decode_qt_sine_status()` 的原始結構（channels 8 筆 + mux_sel_sync
    + phase_acc），呼叫端自己把 tuning_word 換算回 Hz（跟
    apply_sine_channel()/apply_sine_slots() 同一條公式：
    `tuning_word * DAC_FS / 2**32`）。query_type 不符時回傳 None。"""
    _flush_standby(fp)
    send_query(fp, dest_id, QT_SINE_STATUS)
    time.sleep(wait)
    _, got_query_type, data_words = read_status_reply(fp)
    if got_query_type != QT_SINE_STATUS:
        return None
    return decode_qt_sine_status(data_words)


# ── Trigger group（T_TRIG_MASK/QT_TRIGGER_GROUP，2026-07-31 新增）──────────
# 每片板子 4 個 channel（A/B/C/D，對應 channel index 0-3）各自有一個
# 2-bit group_id（board_cfg_reg.v 的 group_id_a/b/c/d），送 T_TRIG_START
# 廣播時的 group_select（4-bit，bit 對應 group 0-3）決定「這次哪些
# group 要 fire」，兩者搭配才決定「這次哪些 channel 真的會觸發」。
#
# T_TRIG_MASK(0x0D) 封包格式（beat1_lo = {group_id[1:0], channel_mask[3:0]}）
# 一次只能設「某個 group_id 涵蓋哪些 channel」，且是部分更新——channel_mask
# 沒選到的 channel 保持原本的 group_id 不變（已核對 rtl/board_cfg_reg.v
# 212-215 行：`if (au_group_cfg_mask[3]) group_id_a <= au_group_cfg_
# group_id;`，其餘同理，沒被 mask 選中的完全不寫）。channel_mask 的 bit
# 對應：bit3=A(ch0)/bit2=B(ch1)/bit1=C(ch2)/bit0=D(ch3)，跟既有
# set_group_config.py 一致。
#
# UI 上操作者是「每個 channel 選一個 group_id」，所以這裡提供
# apply_trig_groups() 把「channel->group_id 對照表」轉換成「依 group_id
# 分組、每組各送一次封包」，channel_mask 用該組實際包含的 channel 算出來
# ——這樣送完之後每個 channel 最終的 group_id 保證等於操作者選的值
# （不會有「先設 group0 涵蓋 A，後面設 group1 涵蓋 B 時不小心又把 A 動到」
# 的疑慮，因為每個 channel 只會出現在自己那組的 mask 裡一次）。

_CHANNEL_MASK_BIT = {0: 3, 1: 2, 2: 1, 3: 0}  # channel index -> channel_mask bit（A/B/C/D）


def set_trig_group(fp, dest_id, group_id, channel_mask):
    """送一次 T_TRIG_MASK：把 channel_mask 選中的 channel 設成 group_id。"""
    val = ((group_id & 0x3) << 4) | (channel_mask & 0xF)
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_TRIG_MASK, beat1_lo=val)


def apply_trig_groups(fp, dest_id, channel_group_ids):
    """channel_group_ids: {channel(0-3): group_id(0-3)}，通常是全部 4 個
    channel 都指定。依 group_id 分組，每個出現過的 group_id 各送一次
    T_TRIG_MASK（channel_mask 只包含選了這個 group_id 的 channel）。"""
    by_group = {}
    for ch, gid in channel_group_ids.items():
        by_group.setdefault(gid, []).append(ch)
    for gid, chs in by_group.items():
        mask = 0
        for ch in chs:
            mask |= (1 << _CHANNEL_MASK_BIT[ch])
        set_trig_group(fp, dest_id, gid, mask)
        time.sleep(0.01)


def read_trig_group(fp, dest_id, wait=0.05):
    """T_QUERY(QT_TRIGGER_GROUP)，回傳 {"group_id_a":..,"group_id_b":..,
    "group_id_c":..,"group_id_d":..}（channel 0-3 = A-D）。query_type
    不符時回傳 None。"""
    _flush_standby(fp)
    send_query(fp, dest_id, QT_TRIGGER_GROUP)
    time.sleep(wait)
    _, got_query_type, data_words = read_status_reply(fp)
    if got_query_type != QT_TRIGGER_GROUP:
        return None
    return decode_qt_trigger_group(data_words)


# ── test2/test3 面板：N 組輪播（2026-08-20 新增）───────────────────────
# test2 = 5 組、test3 = 6 組，同一套機制只是 N 不同，共用這裡的函式。
# 上膛（arm_rotation_pattern()）沿用 arm_test1_pattern() 完全相同的打包
# 邏輯，差別只在每個 module 寫 N 個 slot（不是固定 2 個）；自動輪播
# （start_rotation_schedule()）用 group_trig_scheduler（見上方
# send_group_sched_ctrl()/set_group_sched() 章節）依設定時間自動觸發
# 切到下一個 slot，不需要新的 RTL/CDC——重用既有的 T_TRIG_START 廣播
# 機制，group_trig_scheduler 這個模組本身就是「自己當觸發源」。
#
# ⚠️ group_trig_scheduler 每片板子只有一個實例（不像 trig_timer 是每
# module 各自獨立），所以 test2 跟 test3 不能同時在同一片板子上跑——
# 後啟動的那個會直接覆寫掉前一個的排程（set_group_sched() 寫的是同一份
# 硬體排程表）。這裡不做互斥檢查，是 app.py route 層的責任（目前設計是
# 兩個面板各自獨立呼叫 Run/Stop，操作者自己要留意不要同時對同一片板子
# 開兩個）。
#
# scheduler 位移公式（2026-08-20 推導）：group_trig_scheduler 的狀態機
# 是「先倒數 mem_intv[idx] 才觸發 mem_group[idx]」，不是「先觸發才倒
# 數」，所以「第 i 組（0-indexed）實際播放多久」是由排程表裡
# **下一個** slot（(i+1) % N）的 interval_cycles 決定，不是直覺以為的
# 「slot i 存第 i 組的時長」——因為「第 i 組開始播放」這件事本身，是
# 「上一格倒數完、觸發」的結果，而觸發之後緊接著開始倒數的下一格，正是
# 用來決定「這一組（剛觸發的這組）要播多久」。slot 0 因此身兼兩職：
# run=1 剛啟動時，是「開場靜音多久」（RTL 天生行為——這格內容在第一次
# 觸發之前就已經在倒数，此時 DDR 還沒收到任何 trigger，`waveform_
# controller` 維持在既有的「未觸發前強制靜音」狀態，不是新問題）；loop
# 繞完一圈之後，則變成「最後一組（第 N-1 組）的播放時長」。已跟使用者
# 確認接受這個開場靜音空檔，不需要額外處理（例如額外送一次手動 trigger
# 消掉這段空檔）。
ROTATION_MAX_INTERVAL_SEC = 0xFFFFFFFF / DAC_FS  # interval_cycles 是 32-bit @ 100MHz，硬性上限 ≈42.9497 秒/組
ROTATION_MAX_GROUPS = 7  # 見 arm_rotation_pattern() 內 depth_val 的 3-bit 截斷說明，不用 MAX_SLOTS(8)


def arm_rotation_pattern(fp, dest_id, n_groups, dc_module_sets, rf_module_sets, amp=DEFAULT_AMP):
    """test2/test3「Run」的上膛階段（跟 arm_test1_pattern() 是同一套打包
    邏輯，唯一差異是每個 module 寫 n_groups 個 slot，不是固定 2 個）。

    dc_module_sets: {module(0-2): [(ratio_ch1, ratio_ch2), ...]}，每個
    module 的清單長度必須等於 n_groups。
    rf_module_sets: {module(3): [{"ch1": <spec>, "ch2": <spec>}, ...]}，
    spec 格式跟 arm_test1_pattern() 完全相同（single/ifft），清單長度
    必須等於 n_groups；每個 slot 各自獨立算 IFFT buffer 長度/single 週期
    數，slot 之間彼此不需要有一樣的 buffer 長度——跟 test1 一次只有 1
    組不同，這裡是 N 個獨立的 slot，各自照自己的規則走。

    「哪些 channel 要跟著輪播切換、哪些維持固定」完全是呼叫端（app.py）
    的決定：維持固定的 channel，呼叫端直接把同一組值複製 n_groups 次填
    進對應清單，這裡不知道、也不需要知道哪個 channel 有沒有勾選「跟著
    切換」。

    停在 ARMED，不會開始播放——呼叫端接著呼叫 start_rotation_schedule()
    才會真的開始自動輪播。"""
    if not (2 <= n_groups <= ROTATION_MAX_GROUPS):
        raise ValueError(f"n_groups must be between 2 and {ROTATION_MAX_GROUPS}")
    if not dc_module_sets and not rf_module_sets:
        raise ValueError("dc_module_sets and rf_module_sets must not both be empty")
    overlap = set(dc_module_sets) & set(rf_module_sets)
    if overlap:
        raise ValueError(f"module(s) {sorted(overlap)} appear in both dc_module_sets and rf_module_sets")
    for m, sets in dc_module_sets.items():
        if len(sets) != n_groups:
            raise ValueError(f"module {m}: dc sets length {len(sets)} != n_groups {n_groups}")
    for m, sets in rf_module_sets.items():
        if len(sets) != n_groups:
            raise ValueError(f"module {m}: rf sets length {len(sets)} != n_groups {n_groups}")

    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=0)  # play_en <- 0 先關掉，避免上膛過程中撞到還在播放的舊資料

    module_slots = {}

    for m, sets in dc_module_sets.items():
        slot_addr = _module_base_addr(m)
        slots = []
        for v1, v2 in sets:
            if not (-1.0 <= v1 <= 1.0) or not (-1.0 <= v2 <= 1.0):
                raise ValueError(f"module {m}: amplitude ratio out of range (-1.0~1.0): ch1={v1}, ch2={v2}")
            one_slot = pad_to_burst([_pack_ch1_ch2(int(amp * v1), int(amp * v2))] * TEST1_DC_SLOT_LEN)
            _check_slot_fits(m, slot_addr, len(one_slot))
            write_wave(fp, dest_id, slot_addr, one_slot)
            # TEST1_DC_SLOT_LEN 已經是 BURST_SAMPLES 倍數，wave_len 恆等於
            # n_samples，這裡跟 RF 分支統一欄位名稱方便下方共用 commit 迴圈。
            slots.append({"addr": slot_addr, "n_samples": len(one_slot), "wave_len": len(one_slot)})
            slot_addr += len(one_slot) * 4 + SLOT_MARGIN
        module_slots[m] = {"kind": "dc", "slots": slots, "sets": sets}

    for m, sets in rf_module_sets.items():
        slot_addr = _module_base_addr(m)
        slots = []
        info_extras = []
        max_samples = _slot_max_samples(len(sets))
        for spec in sets:
            ch1, ch2 = spec["ch1"], spec["ch2"]
            for label, ch in (("ch1", ch1), ("ch2", ch2)):
                if ch["mode"] == "single":
                    ra = ch["amp_ratio"]
                    if not (-1.0 <= ra <= 1.0):
                        raise ValueError(f"module {m} {label}: amplitude ratio out of range (-1.0~1.0): {ra}")

            if ch1["mode"] == "single" and ch2["mode"] == "single":
                # 2026-08-21：wave_len_for_two_freqs() 改用 BURST_TOLERANCE
                # 容忍窗口後 n 不一定是 BURST_SAMPLES 的倍數，true_wave_len
                # 記錄真正長度（見 arm_test1_pattern() 同款修法+下方 commit
                # 迴圈說明）——這裡是 2026-08-21 稍早才發現的既有 regression：
                # 這個分支原本沒有同步改，會把 pad_to_burst() 補的 0 樣本
                # 當波形一起播出去，造成每次迴繞相位不連續。
                f1, f2 = ch1["freq_hz"], ch2["freq_hz"]
                ra1, ra2 = ch1["amp_ratio"], ch2["amp_ratio"]
                n, k1, k2 = wave_len_for_two_freqs(f1, f2, max_samples=max_samples)
                true_wave_len = n
                actual_f1 = k1 * DAC_FS / n
                actual_f2 = k2 * DAC_FS / n
                one_slot = pad_to_burst([
                    _pack_ch1_ch2(int(amp * ra1 * math.sin(2 * math.pi * k1 * i / n)),
                                  int(amp * ra2 * math.sin(2 * math.pi * k2 * i / n)))
                    for i in range(n)])
                info_extras.append({"requested_hz": [f1, f2], "actual_hz": [actual_f1, actual_f2]})
            else:
                # 至少一個 channel 是 IFFT：共用 helper（跟 arm_test1_
                # pattern() 共用同一份邏輯，2026-08-21 抽出來，見該函式
                # 說明——這裡也是先前 regression 的源頭：舊版單次 round()
                # 沒有精確度保證，現在改用 wave_len_for_comb_plus_freq()）。
                codes1, codes2, true_wave_len, info_extra_one = _rf_ifft_mixed_codes(
                    m, ch1, ch2, amp, max_samples)
                one_slot = pad_to_burst([_pack_ch1_ch2(int(c1), int(c2))
                                          for c1, c2 in zip(codes1, codes2)])
                info_extras.append(info_extra_one)

            _check_slot_fits(m, slot_addr, len(one_slot))
            write_wave(fp, dest_id, slot_addr, one_slot)
            slots.append({"addr": slot_addr, "n_samples": len(one_slot), "wave_len": true_wave_len})
            slot_addr += len(one_slot) * 4 + SLOT_MARGIN
        module_slots[m] = {"kind": "rf", "slots": slots, "sets": sets, "info_extras": info_extras}

    for m, info in module_slots.items():
        for i, s in enumerate(info["slots"]):
            send_list_write(fp, dest_id, ch=m, slot=i, addr=s["addr"], wave_len=s["wave_len"])
            time.sleep(0.01)

    # au_play_ctrl 是四個 module 共用的單一暫存器，見 arm_waveform_
    # stereo_same() 上方同一則說明——只送這裡涵蓋的 module 一樣會把
    # 其餘 module 的 play_en 一併清成 0。
    depth_val = sum((len(info["slots"]) & 0x7) << (m * 3) for m, info in module_slots.items())
    play_val = sum(1 << m for m in module_slots.keys())
    au_play_ctrl = (depth_val << 4) | play_val
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_PLAY_CTRL, beat1_lo=au_play_ctrl)
    time.sleep(0.05)

    return {"dest_id": dest_id, "modules": {str(m): info for m, info in module_slots.items()}}


def start_rotation_schedule(fp, dest_id, group_id, modules, set_durations_sec):
    """test2/test3「Run」的第二階段——arm_rotation_pattern() 上膛完成後
    呼叫，把 modules（這次上膛涵蓋到的 module，0-3 的子集）全部指定成
    同一個 group_id（apply_trig_groups()），再依 set_durations_sec（長度
    必須等於 arm_rotation_pattern() 用的 n_groups）算出 group_trig_
    scheduler 的排程，loop_en=True、run=True 開始自動輪播。

    schedule slot (i+1) % N 存「第 i 組（0-indexed）的播放時長」，見上方
    章節開頭的公式推導；group_sel 全部排程格都填同一個 group_id（這裡
    不需要真的切換不同 group，只是借用 scheduler 的定時觸發能力）。

    group_id：0-3（set_trig_group() 的 2-bit 欄位限制，見 apply_trig_
    groups()），呼叫端自行決定要用哪一個。"""
    n = len(set_durations_sec)
    if not (2 <= n <= ROTATION_MAX_GROUPS):
        raise ValueError(f"set_durations_sec must have 2-{ROTATION_MAX_GROUPS} entries, got {n}")
    for i, sec in enumerate(set_durations_sec):
        if not (0 < sec <= ROTATION_MAX_INTERVAL_SEC):
            raise ValueError(
                f"set {i}: duration {sec}s out of range (0, {ROTATION_MAX_INTERVAL_SEC:.4f}] seconds "
                "(interval_cycles is a 32-bit @ 100MHz field — hard hardware limit)")
    if not modules:
        raise ValueError("modules must not be empty")
    if not (0 <= group_id <= 3):
        raise ValueError("group_id must be 0-3")

    apply_trig_groups(fp, dest_id, {m: group_id for m in modules})

    # 2026-08-20 修復：group_trig_scheduler 的 group_sel 欄位在下游
    # group_trig_select.v 是當成「4-bit one-hot bitmask」用（`group_
    # select[group_id]`，用 group_id 去索引 bitmask 的某一個 bit，見
    # rtl/group_trig_select.v），不是直接拿 group_id 這個數值本身。
    # 這裡原本直接把 group_id（0-3 的原始編號）當 group_sel 送進去，
    # 語意是錯的——例如 group_id=1 應該送 bitmask `0b0010`(=2)，不是
    # `1`（那是 bitmask `0b0001`，實際上會命中 group_id=0，不是
    # group_id=1）。**這個 bug 讓 test2/test3 的第一次切換（來自
    # apply_trig_groups()+trigger_playback(0xF) 那次廣播，0xF 是
    # all-bit-set，不受這個 bug 影響）看起來正常，但之後每一次真正
    # 由 group_trig_scheduler 自己觸發的切換都送錯 bitmask、完全打不中
    # 目標 module，DDR play_pos 會卡住不動——早先「test2 100µs 上機測試
    # 通過」那次很可能只是卡在 Set 1 沒有真的在輪播，肉眼一瞥沒看出來
    # （100µs 太快，卡住的靜態值跟正常輸出用眼睛看很像）。用 Architecture
    # B（arm_mode=True）上機測試 play_pos 完全卡住不動才抓到，見
    # PROJECT.md「test2/test3 多板同步」章節。修法：`1 << group_id`
    # 才是正確的 bitmask 編碼，跟 `trigger_playback(fp, 0xF)`（全部
    # bit 都設 1）、`apply_trig_groups()`/`set_trig_group()`（`group_id`
    # 本身維持原始編號不變，那是另一個獨立的暫存器，語意本來就對）
    # 兩者的既有慣例保持一致。
    group_bitmask = 1 << group_id
    schedule = [None] * n
    for i, sec in enumerate(set_durations_sec):
        slot = (i + 1) % n
        schedule[slot] = (round(sec * DAC_FS), group_bitmask)
    set_group_sched(fp, dest_id, schedule, loop_en=True, run=True)

    return {"dest_id": dest_id, "group_id": group_id, "n_groups": n, "schedule": schedule}


def stop_rotation_schedule(fp, dest_id):
    """test2/test3「Stop」的排程停止階段——立即停掉 group_trig_scheduler
    （loop_en=False, run=False，不管當下播到哪一組）。呼叫端（app.py）
    接著要另外呼叫既有的歸零機制（比照 test1_stop 用 arm_test1_
    pattern() 全 DC 0V + trigger_playback(0xF)）才會讓輸出真的靜音——
    這裡只負責停排程，不動 DDR 內容/輸出電壓。"""
    set_group_sched(fp, dest_id, [], loop_en=False, run=False)


# ── trig_delay 手動覆寫（T_TRIG_DELAY_CFG，2026-07-31 新增）─────────────
# 正常情況下 aurora_ctrl_channel.v 會自動算出跨板 trigger 延遲補償值
# （per_hop_value x hops），這個封包讓 host 對單一板子個別手動覆寫。
# 單位是 aurora_clk cycle 數（156.25MHz，6.4ns/cycle），不是 sys_clk。
#
# ⚠️ sticky，沒有「取消覆寫」的封包：送過一次之後 manual_delay_
# override_active 就會維持 1，手動值永遠優先於自動算出的補償值，直到
# 板子真的 reset（reload/重新燒錄/斷電）才會恢復自動模式。呼叫端（web
# UI）務必在送出前明確警告操作者這是不可逆動作，見 test_trig_delay.py
# 檔頭說明——這裡沿用同一份協定理解，不是猜的。

def set_trig_delay(fp, dest_id, delay):
    """T_TRIG_DELAY_CFG(0x1D)：手動覆寫 dest_id 這片板子的 trigger 延遲
    補償值（aurora_clk cycle 數，0-65535）。"""
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_TRIG_DELAY_CFG, beat1_lo=delay & 0xFFFF)


# ── DDR/Sine 模式 + sine_gen/amp_ramp_gen 參數（2026-08-03 新增，web UI
# 第一次真正用到這條路徑；2026-08-05 dac_mode_ramp 併入 sine_ctrl_regs.v，
# 改成獨立定址 idle/active 雙緩衝，取代退役的 T_DAC_MODE_RAMP 整包覆寫）
# ──────────────────────────────────────────────────────────────────────
# mode 是 per-module（4 個，DDR(0)/Sine(1)）、ramp_en 是 per-channel
# （8 個）。兩者都走 T_SINE_CTRL（PARAM_MODE=7/PARAM_RAMP_EN=6），只會
# 寫進目前 idle 那份，不會立刻生效，要另外送一次 T_TRIG_START 才會真正
# 切換——跟頻率/振幅參數是同一套雙緩衝機制，見 rtl/sine_ctrl_regs.v
# 檔頭說明。

def set_dac_mode(fp, dest_id, module, value):
    """module（0-3）的 DDR(0)/Sine(1) 模式。內部用 ch_sel=module<<1
    （RTL 端只看 ch_sel[2:1]，見 sine_ctrl_regs.v 檔頭），只動這一個
    module，不影響其他 3 個。"""
    if not (0 <= module <= 3):
        raise ValueError("module must be 0-3")
    set_sine_ctrl(fp, dest_id, module << 1, PARAM_MODE, 1 if value else 0)


def set_ramp_en(fp, dest_id, channel, value):
    """physical channel（0-7）的 amp ramp 開關，只動這一個 channel。"""
    if not (0 <= channel <= 7):
        raise ValueError("channel must be 0-7")
    set_sine_ctrl(fp, dest_id, channel, PARAM_RAMP_EN, 1 if value else 0)


def set_dac_mode_ramp(fp, dest_id, value):
    """相容層——維持原本「一次傳完整 12-bit」的呼叫方式（bits[3:0]=4 個
    module 各自 DDR(0)/Sine(1)，bits[11:4]=8 個 channel 各自 ramp_en），
    內部拆成 4 個 set_dac_mode() + 8 個 set_ramp_en() 呼叫（各自獨立
    定址寫入自己的 idle 側，不會互相干擾）。現有呼叫端（app.py、
    test_trig_timer.py 等）不用改。新寫的程式碼應該直接呼叫
    set_dac_mode()/set_ramp_en()，只在真的需要一次設定全部時才用這個。
    """
    for module in range(4):
        set_dac_mode(fp, dest_id, module, (value >> module) & 0x1)
    for channel in range(8):
        set_ramp_en(fp, dest_id, channel, (value >> (4 + channel)) & 0x1)


def set_sine_ctrl(fp, dest_id, ch_sel, param_sel, value):
    """T_SINE_CTRL(0x29)：sel={param_sel[2:0],ch_sel[2:0]}，寫入
    sine_gen/amp_ramp_gen 的其中一個參數（PARAM_TUNING_WORD/PHASE/
    START_AMP/STEP/DURATION_CYCLES/LOOP_MODE，見 awg_common.py）。
    ch_sel 用 module-major 編號（0-7，= 2*inst + (ch2?1:0)），跟既有
    ampCtrlChannelLabel()/calib_coef 同一套慣例。"""
    sel = ((param_sel & 0x7) << 3) | (ch_sel & 0x7)
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_SINE_CTRL, beat1_lo=value, beat1_hi=sel & 0x3F)
    time.sleep(0.01)


def apply_sine_channel(fp, dest_id, ch_sel, freq_hz, phase=0, ramp=None):
    """設定 sine_gen 一個 physical channel（ch_sel 0-7）：一定送
    tuning_word（頻率換算）+ phase；ramp 給定時（dict:
    start_amp/end_amp/duration_sec/loop_mode，amp 是 0-1 分數，跟
    calib_coef/amp_ctrl 同一種 Q1.16 定點慣例）才連帶送 amp_ramp_gen
    的 4 個參數。數值換算公式完全比照 set_sine_ctrl_remote.py：
    tuning_word = round(freq_hz * 2^32 / DAC_FS)，
    step 用 (end-start)*2^14/duration_cycles 算，避免長時間 ramp 每
    cycle 步進量太小被整數捨去成 0。回傳算出來的實際值方便 endpoint
    回傳給前端顯示（例如頻率換算後的實際 Hz 跟輸入值有些微誤差）。"""
    tuning_word = round(freq_hz * (2 ** 32) / DAC_FS) & 0xFFFFFFFF
    actual_freq_hz = tuning_word * DAC_FS / (2 ** 32)
    set_sine_ctrl(fp, dest_id, ch_sel, PARAM_TUNING_WORD, tuning_word)
    set_sine_ctrl(fp, dest_id, ch_sel, PARAM_PHASE, phase)
    result = {"tuning_word": tuning_word, "actual_freq_hz": actual_freq_hz}
    if ramp is not None:
        start_signed = round(ramp["start_amp"] * 65536)
        end_signed = round(ramp["end_amp"] * 65536)
        duration_cycles = round(ramp["duration_sec"] * DAC_FS)
        if duration_cycles <= 0:
            raise ValueError("duration_sec must be long enough to produce at least 1 cycle")
        step_signed = round((end_signed - start_signed) * (1 << 14) / duration_cycles)
        start_amp_reg = start_signed & 0x3FFFF
        step_reg = step_signed & 0xFFFFFFFF
        set_sine_ctrl(fp, dest_id, ch_sel, PARAM_START_AMP, start_amp_reg)
        set_sine_ctrl(fp, dest_id, ch_sel, PARAM_STEP, step_reg)
        set_sine_ctrl(fp, dest_id, ch_sel, PARAM_DURATION_CYCLES, duration_cycles)
        set_sine_ctrl(fp, dest_id, ch_sel, PARAM_LOOP_MODE, ramp["loop_mode"])
        result.update({
            "start_amp_reg": start_amp_reg,
            "step_reg": step_reg,
            "duration_cycles": duration_cycles,
        })
    return result


def apply_sine_slots(fp, dest_id, ch, freqs_hz, ramp=None):
    """比照 apply_sine_channel() 的頻率/ramp 換算公式，但寫進 N-slot
    table（set_sine_slot()）而不是單值 T_SINE_CTRL 立即生效路徑——這是
    web UI 的 Sine 頻率欄位改成 slot list 之後的正式入口（2026-08-05，
    取代原本每個 channel 一個 freq 欄位、按 Set Sine Params 立即生效
    的舊流程；理由見 rtl/sine_ctrl_regs.v 檔頭：slot table 是完全獨立
    的儲存區，不會被單值寫入路徑碰到，兩條路徑同時開放給操作者用很
    容易在不知情的狀況下互相覆蓋，所以直接讓 slot list 變成唯一入口，
    跟 DDR 的 slot list 是唯一內容機制一致）。

    freqs_hz：1-SINE_N_SLOT 個頻率（Hz），依序對應 slot 0,1,2,...，
    commit 之後這個 channel 就能搭配 Trigger Timer 自動依序輪播。

    ramp 給定時（跟 apply_sine_channel() 完全同一種 dict 格式：
    start_amp/end_amp/duration_sec/loop_mode）套用到「每一個」slot——
    共用同一組振幅包絡形狀，只有頻率逐 slot 不同（一次只能設一組
    ramp 曲線，不支援每個 slot 各自不同的振幅包絡，這是這次的刻意
    範圍縮減，需要的話之後再擴充成逐 slot 設定）。ramp=None 時每個
    slot 用固定滿幅常數（start_amp=0x10000/step=0/loop_mode=0，等同
    ramp_en=0 時原本就有的輸出——這幾個值不會影響輸出，因為
    amp_ctrl_mux.v 的 ramp_en=0 分支完全繞過這幾個暫存器）。呼叫端
    需要另外呼叫 set_ramp_en() 控制 ramp_en 開關本身（獨立的單一
    buffered 暫存器，不屬於 slot table，見 rtl 檔頭）。
    """
    if not (1 <= len(freqs_hz) <= SINE_N_SLOT):
        raise ValueError(f"freqs_hz must have 1-{SINE_N_SLOT} entries")

    if ramp is not None:
        start_signed = round(ramp["start_amp"] * 65536)
        end_signed = round(ramp["end_amp"] * 65536)
        duration_cycles = round(ramp["duration_sec"] * DAC_FS)
        if duration_cycles <= 0:
            raise ValueError("duration_sec must be long enough to produce at least 1 cycle")
        step_signed = round((end_signed - start_signed) * (1 << 14) / duration_cycles)
        start_amp_reg = start_signed & 0x3FFFF
        step_reg = step_signed & 0xFFFFFFFF
        loop_mode = ramp["loop_mode"]
    else:
        start_amp_reg = 0x10000  # 1.0 Q1.16 -- irrelevant while ramp_en=0
        step_reg = 0
        duration_cycles = DAC_FS
        loop_mode = 0

    slots = []
    actual_freqs_hz = []
    for freq_hz in freqs_hz:
        tuning_word = round(freq_hz * (2 ** 32) / DAC_FS) & 0xFFFFFFFF
        actual_freqs_hz.append(tuning_word * DAC_FS / (2 ** 32))
        slots.append({
            "tuning_word": tuning_word,
            "phase": 0,
            "start_amp": start_amp_reg,
            "step": step_reg,
            "duration_cycles": duration_cycles,
            "loop_mode": loop_mode,
        })
    set_sine_slot(fp, dest_id, ch, slots)
    return {"n_slots": len(slots), "actual_freqs_hz": actual_freqs_hz}
