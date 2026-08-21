"""
init_flow.py — web backend 用的「初始化系統」流程（2026-07-30 新增）

比照 post_flash_checklist.py 的 Phase 0/6/7/8b/5/3/4/9/10，但整套改成
master-only USB 可行版本：只用呼叫端傳進來的 master fp，其餘板子的狀態/
讀回驗證全部經由 T_QUERY(QT_BOARD_INFO/QT_CALIB_STATUS) 遠端讀，不開
slave 自己的 USB（對應真實部署情境，見 feedback_slave_usb_readonly_
testing）。

跟 post_flash_checklist.py 的另一個差異：那支腳本假設板數固定等於
BOARD_MAP 長度，這裡改成「enum 回報多少就是多少」（total_boards 由
Phase enum 動態決定），配合 web 介面依板子數量自動增加控制面板的需求。

失敗時不 sys.exit——這是常駐 web 程序，要把失敗原因包成結構化結果回傳，
讓呼叫端（Flask endpoint）轉成 JSON 顯示給使用者，不能讓整個 backend
程序跟著掛掉。呼叫端必須自己負責序列化存取（跟其他 T_QUERY 呼叫共用同一條
master USB，不能並行）。

Phase 4（ext_clk 頻率量測，逐板 measure_ext_clk_freq_remote）依賴這次
新增的 QT_BOARD_INFO ext_clk_freq_count 欄位（見 NOTES.md「QT_BOARD_INFO
擴充規格」），Phase 9 驗證依賴同批新增的 dac_mode_ramp 欄位——這兩個都要
等 Linux 端新 bitstream 上機後才有意義，新 bitstream 燒錄前呼叫這兩個
phase 會失敗或讀到全 0，這是預期中的（bitstream 還沒到位)，不是這支程式
的 bug。
"""
import sys
import os
import subprocess
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from awg_common import (
    wo, ti, send_pkt, send_beat0_pkt, send_query, read_status_reply,
    decode_rt_board_info, decode_enum_status, decode_qt_board_info,
    decode_qt_calib_status, decode_aurora_status, measure_ext_clk_freq_remote,
    WO_RT_BOARD_INFO, WO_ENUM_STATUS, WO_AURORA_STATUS,
    T_ENUM_START, T_BOARD_ID_ASSIGN, T_EXT_CLK_SEL, T_TRIG_START,
    DEST_BCAST, QT_BOARD_INFO, QT_CALIB_STATUS, TI_CMD, TI_BIT_FLUSH_STANDBY,
)
# 2026-08-05：dac_mode_ramp 併入 sine_ctrl_regs.v，退役的 T_DAC_MODE_RAMP
# 改用 _write_mode_ramp_zero()（見該函式定義處註解：雙緩衝設計下要
# 「寫→觸發→再寫」兩次才能讓兩側都歸零、之後不會被日後不相干的觸發
# 反彈回舊值），直接沿用 post_flash_checklist.py 已經寫好、驗證過的
# 同一份實作，不要各自維護一份。
from post_flash_checklist import SI5332_PROJECT, SI5332_CONFIGURE_PY, _write_mode_ramp_zero
from si5332_ctrl import apply_saved_output_state

EXT_CLK_EXPECT_HZ = 100_000_000
EXT_CLK_TOLERANCE = 0.02  # +-2%，跟 post_flash_checklist.py 一致

# 2026-08-10：Pi 5B（Linux）沒有 Windows-only 的 ClockBuilder Pro CLI，
# Phase 0 改呼叫 si5332_configure_linux.py（pyusb 直接控制，見 NOTES.md
# 「Si5332 寫入協定完整跑通」章節，已上機驗證：board B/board C 實測 ~99.996MHz）。
# Windows 端維持原本呼叫 si5332_configure.py 不變。
IS_LINUX = sys.platform.startswith("linux")
SI5332_CONFIGURE_LINUX_PY = os.path.join(os.path.dirname(__file__), "..", "si5332_configure_linux.py")
SI5332_REGISTERS_CSV = os.path.join(os.path.dirname(__file__), "..", "si5332_3out_registers.csv")


def _flush_and(fp, action):
    ti(fp, TI_BIT_FLUSH_STANDBY)
    time.sleep(0.01)
    action()


def _query_board_info(fp, dest_id, wait=0.05):
    """送 T_QUERY(QT_BOARD_INFO)，回傳 decode 過的 dict，query_type 不符時回傳 None。"""
    _flush_and(fp, lambda: send_query(fp, dest_id, QT_BOARD_INFO))
    time.sleep(wait)
    _, got_query_type, data_words = read_status_reply(fp)
    if got_query_type != QT_BOARD_INFO:
        return None
    return decode_qt_board_info(data_words)


def _query_calib(fp, dest_id, wait=0.05):
    _flush_and(fp, lambda: send_query(fp, dest_id, QT_CALIB_STATUS))
    time.sleep(wait)
    _, got_query_type, data_words = read_status_reply(fp)
    if got_query_type != QT_CALIB_STATUS:
        return None
    return decode_qt_calib_status(data_words)


def _phase(name, phases, ok, detail=None, error=None, on_phase=None):
    entry = {"name": name, "ok": ok, "detail": detail}
    if error is not None:
        entry["error"] = error
    phases.append(entry)
    if on_phase is not None:
        on_phase(entry)
    return entry


def run_init(fp, on_phase=None):
    """master-only USB 版初始化流程。回傳：
    {
      "ok": bool,
      "total_boards": int | None,
      "phases": [ {"name": ..., "ok": ..., "detail": ...}, ... ],
      "boards": { dest_id(int): {..board_info.., "ext_clk_freq_hz": float|None} }
    }
    第一個失敗的 phase 之後不再繼續，但已完成的 phases/boards 資料仍會回傳。

    on_phase：2026-08-10 新增，每個 phase 一做完就立刻呼叫一次
    on_phase(entry)（entry 跟 phases[] 裡附加的項目是同一個 dict）——給
    `app.py` 的 `api_init()` 用來即時 SSE 廣播進度，讓網頁不用整個流程
    跑完才知道目前卡在哪一步。這裡本身不碰 broadcast，只單純轉發，維持
    這支模組不依賴 Flask/SSE 的原則。
    """
    phases = []
    boards = {}

    # ── Phase 0: Si5332 ──────────────────────────────────────────────
    if IS_LINUX:
        ret = subprocess.run(
            [sys.executable, SI5332_CONFIGURE_LINUX_PY, "--registers", SI5332_REGISTERS_CSV],
            capture_output=True, text=True)
    else:
        ret = subprocess.run(
            [sys.executable, SI5332_CONFIGURE_PY, "--project", SI5332_PROJECT],
            capture_output=True, text=True)
    si5332_ok = ret.returncode == 0
    _phase("si5332", phases, si5332_ok,
           detail={"stdout": ret.stdout.strip()},
           error=None if si5332_ok else ret.stderr.strip(), on_phase=on_phase)
    if not si5332_ok:
        return {"ok": False, "total_boards": None, "phases": phases, "boards": boards}

    # si5332_configure.py just reset the chip to whatever si5332_output_
    # state.json wiped it back to the project file's baked-in OUTx_OE
    # defaults — reapply any manually-toggled channels on top (no-op if
    # that file doesn't exist yet). Deliberately non-fatal: this is a
    # "restore last preference" nicety, not core to bringing the clock up,
    # and the boards already have whatever clock the project defaults gave
    # them even if this step fails.
    outputs_result = apply_saved_output_state()
    _phase("si5332_outputs", phases, outputs_result["ok"],
           detail={"applied": outputs_result.get("applied")},
           error=outputs_result.get("error"), on_phase=on_phase)

    # ── Phase 6: 本機直送 T_BOARD_ID_ASSIGN 給 master，解開 is_master/enum 死結 ──
    info = decode_rt_board_info(wo(fp, WO_RT_BOARD_INFO))
    local_dest = info["board_id"]  # enum 前是 reset 預設值，走 loopback
    _flush_and(fp, lambda: send_pkt(fp, local_dest, T_BOARD_ID_ASSIGN, beat1_lo=0, beat1_hi=(1 << 5)))
    time.sleep(0.05)
    info = decode_rt_board_info(wo(fp, WO_RT_BOARD_INFO))
    master_unlock_ok = info["is_master"] == 1
    _phase("master_unlock", phases, master_unlock_ok, detail=info, on_phase=on_phase)
    if not master_unlock_ok:
        return {"ok": False, "total_boards": None, "phases": phases, "boards": boards}

    # ── Phase 7: enum ────────────────────────────────────────────────
    _flush_and(fp, lambda: send_beat0_pkt(fp, DEST_BCAST, T_ENUM_START))
    time.sleep(0.1)
    enum_status = decode_enum_status(wo(fp, WO_ENUM_STATUS))
    total_boards = enum_status["total_boards"]
    enum_ok = enum_status["init_ok"] == 1 and total_boards > 0
    if enum_ok:
        enum_error = None
    else:
        # 2026-08-20：訊息從籠統的「SFP 迴路未建立完成」改成明確指出是
        # master 對 slave 的哪個方向沒連上——直接讀 WO_AURORA_STATUS
        # （channel_up_0=上一片板子→master 這段、channel_up_1=master→
        # 下一片板子這段），不用再靠使用者自己猜。
        aurora_status = decode_aurora_status(wo(fp, WO_AURORA_STATUS))
        link_detail = []
        if aurora_status["channel_up_1"] == 0:
            link_detail.append("master → 下一片 slave 這段沒有連上")
        if aurora_status["channel_up_0"] == 0:
            link_detail.append("環路最後一棒（上一片 slave → master）這段沒有連上")
        link_desc = "，".join(link_detail) if link_detail else "兩個方向的 channel_up 都是 1，但 enum 仍失敗（少見，建議進一步檢查）"
        enum_error = (
            f"init_ok={enum_status['init_ok']}, total_boards={total_boards} — "
            f"這片 master 板子對 slave 的連線無法建立：{link_desc}"
            f"（channel_up_0={aurora_status['channel_up_0']}, channel_up_1={aurora_status['channel_up_1']}）。"
            "單板操作（其他板子沒開機/沒接上環狀鏈路）時這裡預期會失敗，不代表本板異常。")
    _phase("enum", phases, enum_ok, detail=enum_status, error=enum_error, on_phase=on_phase)
    if not enum_ok:
        return {"ok": False, "total_boards": None, "phases": phases, "boards": boards}

    dest_ids = list(range(total_boards))

    # ── Phase 8b: T_BOARD_ID_ASSIGN 廣播（套用 board_id/per_hop_value）─
    _flush_and(fp, lambda: send_pkt(fp, DEST_BCAST, T_BOARD_ID_ASSIGN, beat1_lo=0))
    time.sleep(0.1)

    per_hop_values = set()
    assign_ok = True
    for dest_id in dest_ids:
        bi = _query_board_info(fp, dest_id)
        if bi is None:
            assign_ok = False
            boards[dest_id] = {"error": "T_QUERY reply query_type mismatch"}
            continue
        boards[dest_id] = bi
        if bi["board_id"] != dest_id:
            assign_ok = False
        per_hop_values.add(bi["per_hop_value"])
    if len(per_hop_values) != 1 or 0 in per_hop_values:
        assign_ok = False
    _phase("board_id_assign", phases, assign_ok,
           detail={"per_hop_values": list(per_hop_values), "boards": boards}, on_phase=on_phase)
    if not assign_ok:
        return {"ok": False, "total_boards": total_boards, "phases": phases, "boards": boards}

    # unicast is_master=1 給 master（master 目前的 board_id，enum 後恆為 0）
    master_id = boards[0]["board_id"]
    _flush_and(fp, lambda: send_pkt(fp, master_id, T_BOARD_ID_ASSIGN, beat1_lo=0, beat1_hi=(1 << 5)))
    time.sleep(0.1)
    is_master_ok = True
    for dest_id in dest_ids:
        bi = _query_board_info(fp, dest_id)
        expect_master = int(dest_id == master_id)
        if bi is None or bi["is_master"] != expect_master:
            is_master_ok = False
        if bi is not None:
            boards[dest_id].update(bi)  # update 不是整個換掉，保留之前 phase 已經加進去的欄位（如 ext_clk_freq_hz）
    _phase("is_master_assign", phases, is_master_ok, detail={"master_id": master_id, "boards": boards}, on_phase=on_phase)
    if not is_master_ok:
        return {"ok": False, "total_boards": total_boards, "phases": phases, "boards": boards}

    # ── Phase 3: channel_up（已含在上面每次 T_QUERY 的 board_info 裡）──
    channel_up_ok = all(
        boards[d]["channel_up_0"] == 1 and boards[d]["channel_up_1"] == 1
        for d in dest_ids
    )
    _phase("channel_up", phases, channel_up_ok, detail={d: boards[d] for d in dest_ids}, on_phase=on_phase)
    if not channel_up_ok:
        return {"ok": False, "total_boards": total_boards, "phases": phases, "boards": boards}

    # ── Phase 5: 切外部時脈（廣播 T_EXT_CLK_SEL）─────────────────────
    _flush_and(fp, lambda: send_pkt(fp, DEST_BCAST, T_EXT_CLK_SEL, beat1_lo=1))
    time.sleep(0.05)
    ext_clk_sel_ok = True
    for dest_id in dest_ids:
        bi = _query_board_info(fp, dest_id)
        if bi is None or bi["ext_clk_sel"] != 1:
            ext_clk_sel_ok = False
        else:
            boards[dest_id].update(bi)  # update 不是整個換掉，見 is_master_assign 同樣的理由
    _phase("ext_clk_sel", phases, ext_clk_sel_ok, detail={d: boards[d] for d in dest_ids}, on_phase=on_phase)
    if not ext_clk_sel_ok:
        return {"ok": False, "total_boards": total_boards, "phases": phases, "boards": boards}

    # ── Phase 4: ext_clk 頻率量測（需要新 bitstream 的 ext_clk_freq_count 欄位）──
    ext_clk_freq_ok = True
    for dest_id in dest_ids:
        freq_hz = measure_ext_clk_freq_remote(fp, dest_id)
        err = abs(freq_hz - EXT_CLK_EXPECT_HZ) / EXT_CLK_EXPECT_HZ
        boards[dest_id]["ext_clk_freq_hz"] = freq_hz
        if err > EXT_CLK_TOLERANCE:
            ext_clk_freq_ok = False
    _phase("ext_clk_freq", phases, ext_clk_freq_ok, detail={d: boards[d] for d in dest_ids}, on_phase=on_phase)
    if not ext_clk_freq_ok:
        return {"ok": False, "total_boards": total_boards, "phases": phases, "boards": boards}

    # ── Phase 9: dac_output_mux/amp_ctrl_mux 模式強制歸零（需要新 bitstream 的 dac_mode_ramp 欄位）──
    # 2026-08-05：雙緩衝設計下寫一次不夠持久（見 post_flash_checklist.py
    # _write_mode_ramp_zero() 定義處註解 + sim/tb_dac_mode_ramp.v），要
    # 「寫→廣播觸發→再寫」兩次才能讓 active/idle 兩側都歸零。
    _flush_and(fp, lambda: _write_mode_ramp_zero(fp))
    _flush_and(fp, lambda: send_pkt(fp, DEST_BCAST, T_TRIG_START, beat1_lo=0xF))
    time.sleep(0.05)
    _flush_and(fp, lambda: _write_mode_ramp_zero(fp))
    time.sleep(0.05)
    dac_mode_ok = True
    for dest_id in dest_ids:
        bi = _query_board_info(fp, dest_id)
        if bi is None or bi["dac_mode_ramp"] != 0:
            dac_mode_ok = False
        else:
            boards[dest_id].update(bi)  # update 不是整個換掉，見 is_master_assign 同樣的理由
    _phase("dac_mode_zero", phases, dac_mode_ok, detail={d: boards[d] for d in dest_ids}, on_phase=on_phase)
    if not dac_mode_ok:
        return {"ok": False, "total_boards": total_boards, "phases": phases, "boards": boards}

    # ── Phase 10: 校準設定讀回比對（只報告差異，不覆寫）──────────────
    calib = {}
    for dest_id in dest_ids:
        c = _query_calib(fp, dest_id)
        calib[dest_id] = c if c is not None else {"error": "T_QUERY reply query_type mismatch"}
        boards[dest_id]["calib"] = calib[dest_id]

    ref = calib.get(0)
    diffs = []
    if ref is not None and "error" not in ref:
        for dest_id in dest_ids[1:]:
            c = calib[dest_id]
            if "error" in c:
                continue
            if c["scale_cfg"] != ref["scale_cfg"]:
                diffs.append(f"board {dest_id}: scale_cfg 0x{ref['scale_cfg']:02X} vs 0x{c['scale_cfg']:02X}")
            for i, (a, b) in enumerate(zip(ref["amp_ctrl"], c["amp_ctrl"])):
                if a != b:
                    diffs.append(f"board {dest_id}: amp_ctrl[{i}] 0x{a:05X} vs 0x{b:05X}")
            for i, (a, b) in enumerate(zip(ref["calib_coef"], c["calib_coef"])):
                if a != b:
                    diffs.append(f"board {dest_id}: calib_coef[{i}] 0x{a:05X} vs 0x{b:05X}")
    _phase("calib_report", phases, True, detail={"diffs": diffs}, on_phase=on_phase)

    return {"ok": True, "total_boards": total_boards, "phases": phases, "boards": boards}
