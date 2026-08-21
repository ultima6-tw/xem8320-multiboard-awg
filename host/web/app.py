"""
awg-test-step-16 Web 控制介面 backend

架構決定（2026-07-30 與使用者對齊，2026-07-31 擴充為多人同時使用）：
  - 只開 master 一條 USB 連線，不直接碰 slave 的 USB。三片板子的狀態
    全部透過 T_QUERY 經 Aurora 讀（dest_id=0/1/2...），對應真實部署
    （只有 master 接 USB）與 feedback_slave_usb_readonly_testing 的
    規則。**2026-08-10**：master 不再假設一定是 BOARD_MAP 裡的 board A——
    改成自動偵測目前接在 USB 上的第一片裝置，跟 FPGA 端 Phase 6「本機
    直送 T_BOARD_ID_ASSIGN」的既有邏輯一致（那邊本來就是看物理上接
    哪條 USB 決定 is_master，不是看序號），哪片板子接 USB 就是這次
    的 master。
  - 硬體狀態不背景輪詢：只有動作 API 被呼叫時才真的去讀/寫硬體——
    使用者明確要求「trigger 訊號時序最重要，不希望輪詢去干擾」，這條
    規則不因為多人使用而改變，背景永遠不會自己去戳硬體。
  - **多人同時使用（2026-07-31 新增）**：這台之後會放到 Pi 5B 上當一個
    真正的區網 web server，多台電腦要能同時看、同時控制。所有硬體
    存取仍然靠同一個 `_lock` 序列化（不管幾個瀏覽器連進來，同一時間
    只有一個 request 真的在跟 USB 講話），這點不需要改。但「多人同時
    看」需要新機制：`/api/events`（Server-Sent Events）——任何一個
    client 觸發動作、後端讀到新狀態後，直接推播給所有連線中的瀏覽器，
    不用等其他人自己按重新查詢。**這不是背景輪詢**：SSE 只是把「已經
    因為某個動作而讀到的資料」廣播出去，後端本身不會為了推播而主動
    多戳硬體一次。
  - 沿用 host/awg_common.py 既有的 wi/wo/ti/send_query/read_status_reply，
    不重寫底層邏輯。
  - 信任區網、不加登入/密碼（使用者決定，見 NOTES.md）——只給同一個
    實驗室網段用，不對外開放。
"""
import os
import sys
import time
import json
import queue
import threading
import subprocess

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from awg_common import (
    open_board, device_serial, send_query, read_status_reply, QT_DECODERS,
    QT_BOARD_INFO, TI_CMD, TI_BIT_FLUSH_STANDBY, BOARD_MAP, DAC_FS,
    measure_ext_clk_freq_remote,
    WI_DDR4_RW1_ADDR, WO_DDR4_RW1_STATUS, WO_DDR4_RW1_R0, WO_DDR4_RW1_R1,
    WO_DDR4_RW1_R2, WO_DDR4_RW1_R3, TI_BIT_DDR4_RD,
)
from init_flow import run_init
from board_ctrl import (
    arm_waveform_from_files, arm_waveform_stereo_same, arm_ddr_freq_slots,
    arm_stereo_staircase, arm_multitone_pattern, arm_test1_pattern,
    arm_rotation_pattern, start_rotation_schedule, stop_rotation_schedule,
    ROTATION_MAX_INTERVAL_SEC,
    trigger as trigger_playback, read_ddr_status,
    read_sine_status,
    apply_trig_groups, read_trig_group, set_trig_delay,
    set_dac_mode_ramp, apply_sine_channel, apply_sine_slots, set_ramp_en,
    set_trig_timer, set_group_sched,
)
from calib_ctrl import (
    read_calib, set_scale_cfg, set_amp_ctrl, set_calib_coef, reset_calib_coef,
    set_amp_ctrl_bulk, set_calib_coef_bulk, channel_full_scale_v,
)
from flash_ctrl import (
    read_flash_calib, save_scale_cfg_to_flash, save_calib_coef_to_flash,
)
from si5332_ctrl import (
    read_status as si5332_read_status, reconfigure as si5332_reconfigure,
    read_outputs as si5332_read_outputs, set_output_enable as si5332_set_output_enable,
    save_output_state as si5332_save_output_state,
)
from multitone_scan import (
    list_gain_correction_tables, get_active_gain_correction_name,
    set_active_gain_correction, save_gain_correction_table,
    delete_gain_correction_table,
    N_TONES as MULTITONE_N_TONES,
)
from flask import Flask, jsonify, render_template, request, Response

NOMINAL_BOARD_COUNT = len(BOARD_MAP)

app = Flask(__name__)

_lock = threading.Lock()
_dev = None  # 必須保留參照，只存 _fp 會被 GC 回收導致 USB handle 失效
_fp = None
_status_cache = {}  # dest_id(int) -> board_info dict（或 {"error": ...}）

_subscribers = []  # SSE 訂閱者的 queue.Queue() 清單
_subscribers_lock = threading.Lock()

_si5332_lock = threading.Lock()  # Si5332 是獨立 USB 裝置，跟 _lock（_fp）無關，
# 只用來避免兩個瀏覽器同時觸發 CBPro CLI 互相干擾


def broadcast(event_type, data):
    """把 data 推播給所有連線中的 /api/events client。event_type 放進
    JSON 的 "type" 欄位讓前端分派給對應的 render 函式。硬體資料本身早就
    讀好了（呼叫端已經在 _lock 保護下讀完），這裡只是單純轉發，不會
    再去碰硬體。"""
    payload = json.dumps({"type": event_type, **data})
    with _subscribers_lock:
        subs = list(_subscribers)
    for q in subs:
        try:
            q.put_nowait(payload)
        except Exception:
            pass


def connect():
    """開啟（或重新開啟）master 板的 USB 連線。
    2026-08-10（斷電重連）：board 物理斷電重插後，USB 會拿到新的
    device number，process 手上舊的 _dev/_fp 變成失效 handle；先關掉
    舊 handle 再重開，讓 Initialize System 自己就能從物理斷線中恢復，
    不需要手動重啟 service。
    2026-08-10（master 改自動偵測）：不再假設 master 是 BOARD_MAP 裡
    寫死的序號（原本硬寫 board A）——FPGA 端 Phase 6「本機直送
    T_BOARD_ID_ASSIGN」本來就是看「誰接在這條 USB 上」決定
    is_master，不是看序號，這裡改成跟它一致：open_board() 不指定
    serial，自動偵測目前接在 USB 上的第一片裝置直接當 master。多片
    板子同時接 USB 只會發生在調機情境，這時任意挑一片即可。
    """
    global _dev, _fp
    if _dev is not None:
        try:
            _dev.Close()
        except Exception as e:
            print(f"[web] warning: closing previous device handle failed: {e}")
    print("[web] opening master board (auto-detect USB) ...")
    _dev, _fp = open_board()
    print(f"[web] master board connected: {device_serial(_dev)}")


def query_board_info(dest_id, wait=0.05):
    """送 T_QUERY(board_info) 給 dest_id，回傳 decode 過的 dict。跟
    query_board_status.py 同一套流程（flush standby 保險 + 送 query + 等
    + 讀回）。呼叫端必須持有 _lock。"""
    _fp.ActivateTriggerIn(TI_CMD, TI_BIT_FLUSH_STANDBY)
    time.sleep(0.01)
    send_query(_fp, dest_id, QT_BOARD_INFO)
    time.sleep(wait)
    src, got_query_type, data_words = read_status_reply(_fp)
    if got_query_type != QT_BOARD_INFO:
        return {"error": f"unexpected reply query_type=0x{got_query_type:02X} (stale reply not flushed yet? retry)"}
    info = QT_DECODERS[QT_BOARD_INFO](data_words)
    info["_src"] = src
    return info


STATE_FILE = os.path.join(os.path.dirname(__file__), "state_record.json")

# 這些欄位不是 query_board_info() 回傳內容的一部分，任何用一次全新
# query_board_info() 結果整包覆蓋 _status_cache[dest_id] 的地方（Refresh
# Status、Initialize System、trig_delay、dac_mode_ramp 這幾支 API）都要
# 呼叫 _carry_sticky_fields()，否則會把這些「只能靠自己記錄、沒有對應
# 硬體讀回」的欄位意外清空——trig_timer/sine_ctrl/group_sched 這類純
# 寫入無讀回的設定就是這種情況（2026-08-06 使用者明確要求：使用者在
# 這裡輸入過的內容，本身就是唯一的記錄來源，不能因為別的動作觸發了
# 一次全板重讀就把它洗掉）。
_STICKY_KEYS = ("ext_clk_freq_hz", "trig_timer", "armed_ddr", "sine_slots")


def _carry_sticky_fields(dest_id, new_info):
    """把 _STICKY_KEYS 裡列的欄位，從舊的 _status_cache[dest_id] 搬到
    new_info（呼叫端接下來會用 new_info 整包覆蓋 _status_cache[dest_id]）。
    ext_clk_freq_hz 是原本就有的案例（2026-08-03）：query_board_info()
    的回傳不含這個欄位（只有裸的 ext_clk_freq_count），只在 Initialize
    System 或 api_board_ext_clk_freq_refresh() 主動量測時才會寫入，不搬
    的話下一次 Refresh Status 就會消失、變回「not measured yet」。"""
    old_info = _status_cache.get(dest_id)
    if isinstance(old_info, dict):
        for key in _STICKY_KEYS:
            if key in old_info:
                new_info[key] = old_info[key]


def _persist_status():
    """把 _status_cache 整包同步寫進硬碟——每次操作處理完、回應使用者
    之前呼叫（呼叫端必須持有 _lock）。這是這些欄位（尤其 trig_timer/
    sine_ctrl/group_sched 這類完全沒有硬體讀回機制的欄位）的唯一真相
    來源：Flask process 重啟、或另一個瀏覽器連上，都是靠這個檔案回復
    狀態，不是靠記憶體或即時推播（2026-08-06 使用者明確要求：不要背景
    輪詢，只要求「操作時寫檔、讀取時讀檔」，避免干擾 Trigger 時序）。
    dest_id 這個 int key 存成 JSON 前轉成字串（JSON object key 本來就
    只能是字串），讀回時 _load_status() 再轉回 int。"""
    try:
        with open(STATE_FILE, "w") as f:
            json.dump({str(k): v for k, v in _status_cache.items()}, f)
    except Exception as e:
        print(f"[web] WARNING: failed to persist state to {STATE_FILE}: {e}")


def _load_status():
    """開機時把上次的記錄檔讀回 _status_cache，讓 Flask process 重啟也
    不會遺失 trig_timer 這類沒有硬體讀回機制的欄位。讀不到（第一次
    執行）或格式壞掉都不視為致命錯誤，就從空狀態開始，等第一次
    Refresh Status／操作重新填。呼叫端接下來會呼叫 refresh_all_status()
    重新查詢硬體，會用 _carry_sticky_fields() 保留這裡讀回的 sticky
    欄位、但其餘欄位（channel_up_0 等）會被新的查詢結果覆蓋——這是刻意
    的，硬體讀得到的欄位就該以剛查到的真實硬體狀態為準,只有讀不到的
    欄位才靠這個檔案。"""
    global _status_cache
    if not os.path.exists(STATE_FILE):
        return
    try:
        with open(STATE_FILE) as f:
            raw = json.load(f)
        _status_cache = {int(k): v for k, v in raw.items()}
        print(f"[web] loaded persisted state from {STATE_FILE} ({len(_status_cache)} board(s))")
    except Exception as e:
        print(f"[web] WARNING: failed to load persisted state from {STATE_FILE}: {e}")


def refresh_all_status():
    """查 dest_id=0（master/自己）拿到真正的 total_boards，再依序查其餘板子。
    不假設 BOARD_MAP 的板數就是目前環路實際板數。

    2026-08-03（使用者最終確認）：連帶用 measure_ext_clk_freq_remote()
    重新量測每片板子的 ext_clk_freq，並用 read_trig_group() 讀回每片
    板子的 trigger group 歸屬——這兩項本來因為效能考量（各自多花
    0.3-0.4 秒/板的 Aurora 來回）拆成獨立按鈕（ext_clk_freq 卡片旁的
    refresh 按鈕、modules 卡片旁的 Read Details 按鈕），但使用者發現
    「Refresh Status 沒有真的更新所有狀態」後，確認寧可接受變慢也要
    一次查全，所以這裡改回主動觸發。單片量測/讀取失敗不影響其他板子
    或整個 request——量測失敗時 `_carry_ext_clk_freq_hz()` 保留的舊值
    還在，trigger group 讀取失敗就維持該欄位不存在（前端顯示
    "not read yet"）。

    ⚠️ ddr_status（current_idx/next_idx/mux_sel/play_pos）刻意
    **不**放進這裡（2026-08-03，使用者確認）：這是動態的即時播放狀態，
    一旦有任何 trigger 發生（尤其是自動連續觸發）就會立刻過期，跟
    ext_clk_freq/trigger group 這種相對靜態、快照即可代表現況的資料
    性質不同——放進 Refresh Status 會給人「這是持續正確的即時狀態」
    的錯覺，實際上只是「查詢當下那一刻」的快照，查完之後只要再觸發
    一次就不準了。改成必須手動觸發的獨立讀取（見下方
    api_board_ddr_status()，module 卡片旁的「Read Playback Status」
    按鈕），且额外透過既有的 T_TRIG_START 觸發後自動重讀+推播機制
    （見 api_trigger()）保持相對新鮮，而不是靠 Refresh Status 定期
    覆蓋一個「看起來即時、其實是舊資料」的數字。"""
    with _lock:
        try:
            master_info = query_board_info(0)
            _carry_sticky_fields(0, master_info)
        except Exception as e:
            master_info = {"error": str(e)}
        _status_cache[0] = master_info

        total_boards = master_info.get("total_boards") if isinstance(master_info, dict) else None
        if not total_boards:
            total_boards = NOMINAL_BOARD_COUNT

        for dest_id in range(1, total_boards):
            try:
                new_info = query_board_info(dest_id)
                _carry_sticky_fields(dest_id, new_info)
                _status_cache[dest_id] = new_info
            except Exception as e:
                _status_cache[dest_id] = {"error": str(e)}

        for dest_id in range(total_boards):
            info = _status_cache.get(dest_id)
            if not isinstance(info, dict) or "error" in info:
                continue
            try:
                info["ext_clk_freq_hz"] = measure_ext_clk_freq_remote(_fp, dest_id)
            except Exception:
                pass  # 保留 _carry_sticky_fields() 搬過來的舊值（如果有的話）
            try:
                trig_group = read_trig_group(_fp, dest_id)
                if trig_group is not None:
                    info.update(trig_group)
            except Exception:
                pass  # 前端會顯示 "not read yet"，不視為整個 request 失敗

        broadcast("status", {"boards": _status_cache})
        _persist_status()
    return _status_cache


@app.route("/api/board/<int:dest_id>/ext_clk_freq/refresh", methods=["POST"])
def api_board_ext_clk_freq_refresh(dest_id):
    """單獨重新量測這一片板子的 ext_clk_freq（2026-08-03 新增）——
    Initialize System 已經在開機時量過一次，這個 endpoint 只給使用者
    需要重新確認數值時手動觸發，不是背景輪詢也不是 Refresh Status 的
    一部分。"""
    with _lock:
        try:
            freq_hz = measure_ext_clk_freq_remote(_fp, dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        if isinstance(_status_cache.get(dest_id), dict):
            _status_cache[dest_id]["ext_clk_freq_hz"] = freq_hz
        broadcast("status", {"boards": _status_cache})
        _persist_status()
    return jsonify({"ok": True, "dest_id": dest_id, "ext_clk_freq_hz": freq_hz})


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/events")
def api_events():
    """SSE 推播端點。前端用 EventSource('/api/events') 連上後，先收到一次
    目前 cache 的完整狀態（讓剛連上的瀏覽器不用等下一個動作才同步），之後
    任何一台電腦觸發的動作都會即時推播過來。這是單純的 pub/sub 轉發，
    不會為了推播去主動讀硬體。"""
    def gen():
        q = queue.Queue()
        with _subscribers_lock:
            _subscribers.append(q)
        try:
            yield f"data: {json.dumps({'type': 'status', 'boards': _status_cache})}\n\n"
            while True:
                msg = q.get()
                yield f"data: {msg}\n\n"
        finally:
            with _subscribers_lock:
                if q in _subscribers:
                    _subscribers.remove(q)
    resp = Response(gen(), mimetype="text/event-stream")
    resp.headers["Cache-Control"] = "no-cache"
    resp.headers["X-Accel-Buffering"] = "no"
    return resp


@app.route("/api/server_info")
def api_server_info():
    """回傳這台 server 目前的區網 IP，給遠端連線的人知道要連哪個位址。
    2026-08-10 新增：這台 Pi 有線+無線兩張網卡，IP 不只一個，全部列出來
    交給使用者自己判斷要用哪個，不猜測哪個才是"對的"那個。不碰硬體，
    不需要 _lock。"""
    try:
        result = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=5)
        ips = [ip for ip in result.stdout.split() if "." in ip]  # 只留 IPv4，IPv6 帶冒號會被濾掉
    except Exception:
        ips = []
    return jsonify({"ips": ips, "port": 5000})


@app.route("/api/status")
def api_status():
    """回傳目前 cache 的狀態，不觸發新的查詢。"""
    return jsonify(_status_cache)


@app.route("/api/status/refresh", methods=["POST"])
def api_status_refresh():
    """手動重新查詢全部板子狀態——由使用者按按鈕觸發，不是背景輪詢。"""
    return jsonify(refresh_all_status())


@app.route("/api/init", methods=["POST"])
def api_init():
    """初始化系統：si5332 確保 ACTIVE -> 解鎖 is_master/enum 死結 -> enum
    取得 total_boards -> 廣播 board_id/per_hop_value/is_master -> 切外部
    時脈 -> dac_mode 歸零 -> 校準讀回比對，全部走 master 這條 USB + T_QUERY，
    見 init_flow.py。跟狀態查詢一樣，是使用者按按鈕才觸發的動作，不是背景
    自動執行的流程。

    2026-08-04（使用者確認的設計原則）：初始化完成後，畫面上的設定值
    （這裡是 trigger group）就應該已經知道，不該要求操作者另外手動按
    Read 才看得到目前狀態。init_flow.py 本身的階段順序是已經
    上機驗證過的硬體流程，故意不去動它——這裡在 run_init() 跑完、拿到
    每片板子的結果之後，比照 refresh_all_status() 的既有寫法，多補一次
    read_trig_group()，合併進同一份 board info dict（同時也是
    result['boards'][dest_id] 跟 _status_cache[dest_id]，是同一個物件，
    這裡 .update() 兩邊都會生效）。單片讀取失敗不影響其他板子或整個
    initialization 結果，只是該欄位缺席（前端顯示 "not read yet"）。

    2026-08-10：每次呼叫都先 connect() 重開 master 板 USB 連線，讓這個
    按鈕在板子物理斷電重插後也能自己恢復，不需要另外重啟 web service。

    2026-08-10（即時進度）：run_init() 每個 phase 一做完就透過 on_phase
    callback 呼叫這裡的 _broadcast_init_phase()，立刻 SSE 廣播成
    "init_progress" 事件，讓網頁不用等全部 11 個 phase 跑完（可能要
    十幾秒）才知道目前卡在哪一步——之前只有最後一次性的 "init" 事件，
    使用者反映斷線/卡住時完全看不出跑到哪。"""
    def _broadcast_init_phase(entry):
        broadcast("init_progress", entry)

    with _lock:
        connect()
        result = run_init(_fp, on_phase=_broadcast_init_phase)
        for dest_id, info in result.get("boards", {}).items():
            if isinstance(info, dict) and "error" not in info:
                try:
                    trig_group = read_trig_group(_fp, dest_id)
                    if trig_group is not None:
                        info.update(trig_group)
                except Exception:
                    pass
                _carry_sticky_fields(dest_id, info)
                _status_cache[dest_id] = info
        broadcast("init", result)
        _persist_status()
    return jsonify(result)


@app.route("/api/board/<int:dest_id>/arm", methods=["POST"])
def api_board_arm(dest_id):
    """上膛波形，每個 channel 各自獨立、每個 slot 各自一個上傳檔案解析
    出的振幅樣本（2026-08-03 改版，per 使用者回饋：DDR 播放的是使用者
    上傳的任意波形資料，不是 host 端算出的頻率正弦波——那是 Sine 模式
    才有的概念，見 /api/board/<id>/sine_ctrl）：body =
    {"channel_slot_values": {"0": [[0.0, 0.309, ...], [...]], "2": [...]},
    "amp": 8000(可省略), "labels": {"0": ["file1.csv", "file2.csv"], ...}
    (可省略)}（channel index 字串 -> 該 channel 的 slot 清單，每個 slot
    是一組 -1.0~1.0 振幅值，2-8 個 slot，前端已經在瀏覽器端把上傳的
    CSV/文字檔解析成這個格式）。停在 ARMED，還沒 trigger（trigger 是
    全域廣播動作，見 /api/trigger）。

    2026-08-07 新增 `labels`：DDR4 裡實際存的只有樣本值，硬體沒有存
    slot 內容的名字/來源、也沒有對應的讀回欄位（跟 Sine 頻率不同，
    Sine 有暫存器可以讀回確認）——所以「這個 module 上次送了什麼」
    只能由 web app 自己記住，記進 _status_cache[dest_id]["armed_ddr"]
    （跟 trig_timer 同一種「純寫入無讀回，靠本介面自己記錄」處理方式，
    見 _STICKY_KEYS），module 卡片顯示用。labels 純粹是給人看的標籤，
    不影響上膛內容，缺席或跟 channel_slot_values 數量對不上也不擋
    上膛，只是該 module 卡片不顯示檔名。"""
    body = request.get_json(force=True) or {}
    raw = body.get("channel_slot_values")
    amp = body.get("amp", 8000)
    labels_raw = body.get("labels") or {}
    if not raw:
        return jsonify({"ok": False, "error": "channel_slot_values must not be empty"}), 400
    try:
        channel_slot_values = {int(ch): [list(map(float, slot)) for slot in slots] for ch, slots in raw.items()}
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "channel_slot_values keys must be integers, values must be lists of number lists"}), 400
    with _lock:
        try:
            result = arm_waveform_from_files(_fp, dest_id, channel_slot_values, amp=amp)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        armed_ddr = _status_cache.setdefault(dest_id, {}).setdefault("armed_ddr", {})
        for ch, slots in channel_slot_values.items():
            labels = labels_raw.get(str(ch))
            armed_ddr[str(ch)] = {
                "n_slots": len(slots),
                "labels": labels if isinstance(labels, list) else [],
                "ts": time.time(),
            }
        _persist_status()
        # armed_ddr 整包（不只剛剛動到的 channel）一起回傳/廣播，讓前端可以
        # 直接整包塞回 lastStatus[dest_id].armed_ddr 重畫 module 卡片，不用
        # 另外發一次 status 查詢才看得到剛送出的內容（見 renderArmResult()）。
        response = {"ok": True, "armed_ddr": dict(armed_ddr), **result}
        broadcast("arm", response)
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/arm_stereo_same", methods=["POST"])
def api_board_arm_stereo_same(dest_id):
    """跟 /api/board/<id>/arm 幾乎一樣，差別只有兩個：body 的
    module_slot_values 用 module index（0-3，即 z0-z3）當 key（不是
    physical channel），且該 module 的 ch1/ch2 兩個實體聲道會輸出同一份
    內容（arm_waveform_stereo_same()，用 _pack_ch1_ch2(v,v) 取代
    /api/board/<id>/arm 用的 _pack_ch1_only()——後者只會讓 ch1 有訊號、
    ch2 永遠靜音，是 2026-08-14 就記錄過的已知限制）。
    body = {"module_slot_values": {"0": [[0.0, 0.309, ...], [...]], ...},
    "amp": 8000(可省略)}。停在 ARMED，還沒 trigger。"""
    body = request.get_json(force=True) or {}
    raw = body.get("module_slot_values")
    amp = body.get("amp", 8000)
    if not raw:
        return jsonify({"ok": False, "error": "module_slot_values must not be empty"}), 400
    try:
        module_slot_values = {int(m): [list(map(float, slot)) for slot in slots] for m, slots in raw.items()}
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "module_slot_values keys must be integers, values must be lists of number lists"}), 400
    with _lock:
        try:
            result = arm_waveform_stereo_same(_fp, dest_id, module_slot_values, amp=amp)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        response = {"ok": True, **result}
        broadcast("arm", response)
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/arm_ddr_freq_slots", methods=["POST"])
def api_board_arm_ddr_freq_slots(dest_id):
    """DDR Slot Setting 卡片「輸入頻率」入口（2026-08-18 新增，跟既有
    「上傳檔案」入口並列，不取代它——任意波形仍然只能靠上傳）。跟
    /api/board/<id>/arm_stereo_same 的差異：body 送的是頻率（Hz），
    不是使用者自己算好的振幅樣本，伺服器端用 wave_len_for_freq() 精確
    算出每個頻率對應的 buffer 長度+週期數，ch1/ch2 兩個實體聲道輸出
    同一份正弦波。
    body = {"module_freqs_hz": {"0": [900000, 600000], ...},
    "amp": 8000(可省略)}。停在 ARMED，還沒 trigger。"""
    body = request.get_json(force=True) or {}
    raw = body.get("module_freqs_hz")
    amp = body.get("amp", 8000)
    if not raw:
        return jsonify({"ok": False, "error": "module_freqs_hz must not be empty"}), 400
    try:
        module_freqs_hz = {int(m): [float(f) for f in freqs] for m, freqs in raw.items()}
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "module_freqs_hz keys must be integers, values must be lists of numbers"}), 400
    with _lock:
        try:
            result = arm_ddr_freq_slots(_fp, dest_id, module_freqs_hz, amp=amp)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        armed_ddr = _status_cache.setdefault(dest_id, {}).setdefault("armed_ddr", {})
        for m, slots in result["module_slots"].items():
            labels = [f"{s['actual_hz']/1e3:.4f} kHz" for s in slots]
            armed_ddr[str(m)] = {"n_slots": len(slots), "labels": labels, "ts": time.time()}
        _persist_status()
        response = {"ok": True, "armed_ddr": dict(armed_ddr), **result}
        broadcast("arm", response)
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/arm_stereo_staircase", methods=["POST"])
def api_board_arm_stereo_staircase(dest_id):
    """N 階 DC 階梯測試圖案（2026-08-07 新增，見 board_ctrl.arm_stereo_
    staircase() 檔頭）：同一個 module 的 ch1/ch2 各自獨立內容，示波器辨識
    channel 用。body = {"modules": [0,1,2,3]（可省略，預設全部 4 個）,
    "steps": 8（可省略，2-8）, "amp_ratio": 1.0（可省略，0.0-1.0，比例不是
    絕對電壓——這個專案沒有 DAC code 對應實際伏特的驗證過公式，見
    arm_stereo_staircase() 檔頭）, "step_duration_sec"（可省略，每一階
    維持多久，預設 256 samples ≈2.56µs）, "start_level"（可省略，
    "neg_max" 預設雙極性或 "zero" 單極性，見 arm_stereo_staircase()
    檔頭 2026-08-10 段落）}。停在 ARMED，還沒 trigger（見 /api/trigger）。"""
    body = request.get_json(force=True, silent=True) or {}
    modules = body.get("modules", [0, 1, 2, 3])
    steps = body.get("steps", 8)
    amp_ratio = body.get("amp_ratio", 1.0)
    step_duration_sec = body.get("step_duration_sec")
    start_level = body.get("start_level", "neg_max")
    with _lock:
        try:
            result = arm_stereo_staircase(_fp, dest_id, modules, steps=steps,
                                           amp_ratio=amp_ratio, step_duration_sec=step_duration_sec,
                                           start_level=start_level)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        armed_ddr = _status_cache.setdefault(dest_id, {}).setdefault("armed_ddr", {})
        for m in modules:
            armed_ddr[str(m)] = {
                "n_slots": 1,
                "labels": [f"{steps}-step staircase (ch1/ch2 independent, "
                           f"amp={amp_ratio:.2f}, start={start_level}, "
                           f"{result['step_duration_sec']*1e6:.2f}µs/step)"],
                "ts": time.time(),
            }
        _persist_status()
        response = {"ok": True, "armed_ddr": dict(armed_ddr), **result}
        broadcast("arm", response)
    return jsonify(response)


@app.route("/api/trigger", methods=["POST"])
def api_trigger():
    """廣播 T_TRIG_START，不是 per-board 動作。body = {"group_select": 15(可省略，預設全部 group)}。
    觸發會影響全部已知板子的播放狀態，所以順便逐板重讀 ddr_status **和
    sine_status** 並一次性推播給所有連線的瀏覽器，讓大家都看到觸發後的
    最新播放位置/頻率，不用各自再手動按「讀取播放狀態」。

    2026-08-07 新增 sine 這半段（per 使用者回饋：「按下 trigger 時，應該
    就去更新 control 介面的內容」）——DDR 這裡本來就有做，Sine 之前沒有，
    這是這次補上的部分，兩者現在對稱。sine_status 讀取失敗不影響 ddr
    那半段或整個 request，該板該次就是沒有新的 sine 資料（前端維持顯示
    上一次讀到的值）。"""
    body = request.get_json(force=True, silent=True) or {}
    group_select = body.get("group_select", 0xF)
    with _lock:
        try:
            trigger_playback(_fp, group_select)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        dest_ids = sorted(_status_cache.keys()) or list(range(NOMINAL_BOARD_COUNT))
        boards_status = {}
        boards_sine = {}
        for dest_id in dest_ids:
            try:
                status = read_ddr_status(_fp, dest_id)
                if status is not None:
                    boards_status[dest_id] = status
            except Exception:
                pass
            try:
                sine_status = read_sine_status(_fp, dest_id)
                if sine_status is not None:
                    boards_sine[dest_id] = {
                        "channels": [
                            {"freq_hz": ch["tuning_word"] * DAC_FS / (2 ** 32)}
                            for ch in sine_status["channels"]
                        ]
                    }
            except Exception:
                pass
        broadcast("ddr_status_all", {"boards": boards_status, "group_select": group_select})
        broadcast("sine_status_all", {"boards": boards_sine, "group_select": group_select})
    return jsonify({"ok": True, "group_select": group_select, "boards": boards_status, "sine_boards": boards_sine})


def _ddr4_read4(addr):
    """讀 4 個 32-bit word（16 bytes）從 master 板本機 DDR4，透過
    fp_ddr4_rw_1（見 vivado/create_bd.tcl 的註解：DDR4 write 已改走
    ddr_writer_0，fp_ddr4_rw_1 專門保留做 read-back 用）。這是本機直讀，
    沒有 dest_id 轉送機制，只能讀 master 板（dest_id=0）自己的 DDR4。
    addr 必須是 16-byte 對齊——fp_ddr4_rw.v 的 ARSIZE 是寫死的 128-bit，
    位址沒對齊會靜默回傳整個對齊區塊的 word0，不會報錯（見
    feedback_fp_ddr4_rw_narrow_read_trap 這個已知坑），呼叫端必須自己
    保證對齊。呼叫端必須持有 _lock。"""
    _fp.SetWireInValue(WI_DDR4_RW1_ADDR, addr & 0xFFFFFFFF)
    _fp.UpdateWireIns()
    _fp.ActivateTriggerIn(TI_CMD, TI_BIT_DDR4_RD)
    t0 = time.time()
    st = 0
    while time.time() - t0 < 1.0:
        _fp.UpdateWireOuts()
        st = _fp.GetWireOutValue(WO_DDR4_RW1_STATUS)
        if st & 0x02:
            return [_fp.GetWireOutValue(a) for a in
                    (WO_DDR4_RW1_R0, WO_DDR4_RW1_R1, WO_DDR4_RW1_R2, WO_DDR4_RW1_R3)]
        time.sleep(0.001)
    raise RuntimeError(f"DDR4 read timeout at addr=0x{addr:08X}, wo_status=0x{st:08X}")


# ── Multitone 測試圖案（2026-08-14）──────────────────────────────────
# 500Hz~500kHz、500Hz 間距、1000 音梳狀波形，一次性上膛播放。原本是動態
# notch 掃描（背景執行緒逐步排除單一頻率、2-slot 乒乓上傳），批次大小
# 算式在實測上傳速度下無法收斂，Pi 5B 上被 OOM killer 砍掉 process，
# 已拿掉整套 ScanController，改成跟 arm_stereo_staircase() 同一種「單一
# 連續波形一次性上膛」寫法，詳見 board_ctrl.arm_multitone_pattern()。
@app.route("/api/board/<int:dest_id>/arm_multitone_pattern", methods=["POST"])
def api_board_arm_multitone_pattern(dest_id):
    """500Hz~500kHz、500Hz 間距、1000 音梳狀測試圖案（見 board_ctrl.
    arm_multitone_pattern() 檔頭）。body = {"modules": [0,1,2,3]（可省略，
    預設全部 4 個）, "amp_ratio": 1.0（可省略，0.0-1.0）, "exclude_hz":
    [999, 9990]（可省略，2026-08-19 新增，host 端算好再上膛版的 IFFT
    notch 排除頻率功能，見 board_ctrl.arm_multitone_pattern() 檔頭）,
    "module_exclude_hz": {"0": [999], "1": [9990]}（可省略，2026-08-19
    新增，讓同一次呼叫裡不同 module 各自排除不同頻率，覆蓋該 module 的
    exclude_hz——JSON body 的 key 一定是字串，這裡轉成 int 給
    board_ctrl.arm_multitone_pattern() 用）}。停在 ARMED，還沒 trigger
    （見 /api/trigger）。"""
    body = request.get_json(force=True, silent=True) or {}
    modules = body.get("modules", [0, 1, 2, 3])
    amp_ratio = body.get("amp_ratio", 1.0)
    exclude_hz = body.get("exclude_hz")
    module_exclude_hz_raw = body.get("module_exclude_hz")
    module_exclude_hz = {int(k): v for k, v in module_exclude_hz_raw.items()} if module_exclude_hz_raw else None
    with _lock:
        try:
            result = arm_multitone_pattern(_fp, dest_id, modules, amp_ratio=amp_ratio,
                                            exclude_hz=exclude_hz, module_exclude_hz=module_exclude_hz)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        armed_ddr = _status_cache.setdefault(dest_id, {}).setdefault("armed_ddr", {})
        excluded_by_module = result.get("excluded_hz_by_module", {})
        for m in modules:
            # 2026-08-19 新增：multitone_amp_ratio/multitone_exclude_hz 是
            # 給網頁 Edit Settings 卡片「重新展開時回填目前設定」用的原始
            # 數值（見 index.html 的 loadMultitoneFormFromArmed()）——
            # 上面的 labels 只是給人看的文字，沒辦法反解析回 amp_ratio/
            # exclude_hz，這裡另外存一份結構化的，不然使用者反饋「已經
            # 有設定了，按 Edit 卻是空白」完全沒有資料可以回填。
            armed_ddr[str(m)] = {
                "n_slots": 1,
                "labels": [f"multitone 500Hz-500kHz comb (amp_ratio={amp_ratio:.2f})"],
                "ts": time.time(),
                "multitone_amp_ratio": amp_ratio,
                "multitone_exclude_hz": excluded_by_module.get(str(m), []),
            }
        _persist_status()
        response = {"ok": True, "armed_ddr": dict(armed_ddr), **result}
        broadcast("arm", response)
    return jsonify(response)


def _parse_dc_channel(m, label, raw, fs):
    """回傳 (spec, error_or_None)——跟 _parse_rf_channel() 同一種模式，
    2026-08-21 新增 ramp 模式。spec 是 board_ctrl.arm_test1_pattern()
    認得的格式：float（fixed，振幅比例）或 {"mode": "ramp",
    "start_ratio": ..., "slope_ratio_per_s": ...}（斜率單位是使用者輸入
    的實際電壓/秒，這裡換算成 ratio/秒）。"""
    mode = raw.get("mode")
    if mode == "ramp":
        start_v = float(raw["start_v"])
        slope_v_per_s = float(raw["slope_v_per_s"])
        if slope_v_per_s == 0:
            raise ValueError(f"z{m} {label}: ramp slope must not be 0")
        start_ratio = start_v / fs
        if not (-1.0 <= start_ratio <= 1.0):
            return None, f"z{m} {label}: ramp start {start_v}V exceeds current ±{fs}V range"
        return {"mode": "ramp", "start_ratio": start_ratio, "slope_ratio_per_s": slope_v_per_s / fs}, None
    if mode == "fixed":
        v = float(raw["v"])
        ratio = v / fs
        if not (-1.0 <= ratio <= 1.0):
            return None, f"z{m} {label}: {v}V exceeds current ±{fs}V range"
        return ratio, None
    raise ValueError(f"z{m} {label}: mode must be 'fixed' or 'ramp', got {mode!r}")


def _dc_channel_label(spec, fs):
    """armed_ddr 顯示用的人類可讀字串，跟 _rf_channel_label() 同一種
    用途（2026-08-21 新增，DC 改成 fixed/ramp 兩種 spec 後不能再直接
    `v * fs` 格式化）。"""
    if isinstance(spec, dict):
        return f"DC ramp {spec['start_ratio'] * fs:+.3f}V @ {spec['slope_ratio_per_s'] * fs:+.3f}V/s"
    return f"DC {spec * fs:+.3f}V"


def _parse_rf_channel(m, label, raw, fs):
    """回傳 (spec, error_or_None)。error 非 None 時 spec 是 None——跟 DC
    一樣，range 超出的錯誤要收集起來一起回報，不是踩到第一個就停，所以
    用回傳值而不是直接 raise。2026-08-20 從 api_board_arm_test1() 內部
    搬到 module 層級，讓 test2/test3 的 _api_arm_rotation() 也能重用
    （逐 set 呼叫同一個 per-channel 解析邏輯，不重寫一份）。"""
    mode = raw.get("mode")
    if mode == "single":
        freq_hz = float(raw["freq_hz"])
        if freq_hz <= 0:
            raise ValueError(f"z{m} {label}: freq_hz must be positive")
        amp_v = float(raw["amp_v"])
        ratio = amp_v / fs
        if not (-1.0 <= ratio <= 1.0):
            return None, f"z{m} {label}: {amp_v}V exceeds current ±{fs}V range"
        return {"mode": "single", "freq_hz": freq_hz, "amp_ratio": ratio}, None
    elif mode == "ifft":
        start_hz = float(raw["start_hz"])
        end_hz = float(raw["end_hz"])
        step_hz = float(raw["step_hz"])
        if start_hz <= 0 or end_hz <= 0 or step_hz <= 0:
            raise ValueError(f"z{m} {label}: IFFT start_hz/end_hz/step_hz must all be positive")
        if start_hz > end_hz:
            raise ValueError(f"z{m} {label}: IFFT start_hz must be <= end_hz")
        spec = {"mode": "ifft", "start_hz": start_hz, "end_hz": end_hz, "step_hz": step_hz}
        ex_start, ex_end = raw.get("exclude_start_hz"), raw.get("exclude_end_hz")
        if ex_start is not None and ex_end is not None:
            spec["exclude_start_hz"] = float(ex_start)
            spec["exclude_end_hz"] = float(ex_end)
        return spec, None
    raise ValueError(f"z{m} {label}: mode must be 'single' or 'ifft', got {mode!r}")


def _rf_channel_label(spec):
    """armed_ddr 顯示用的人類可讀字串，同樣 2026-08-20 搬到 module 層級
    供 test1/test2/test3 共用。"""
    if spec["mode"] == "single":
        return f"{spec['freq_hz']:.0f}Hz @ {spec['amp_ratio']:+.3f} ratio"
    label = f"IFFT {spec['start_hz']:.0f}-{spec['end_hz']:.0f}Hz step {spec['step_hz']:.0f}Hz"
    if "exclude_start_hz" in spec:
        label += f" (exclude {spec['exclude_start_hz']:.0f}-{spec['exclude_end_hz']:.0f}Hz)"
    return label


@app.route("/api/board/<int:dest_id>/arm_test1", methods=["POST"])
def api_board_arm_test1(dest_id):
    """test1 面板「Run」——6 個 DC 通道（z0-z2，各自固定電壓）+ 2 個 RF
    通道（z3，各自獨立模式：單頻或 IFFT 梳狀波）填完一次寫入+開始播放
    （2026-08-20 新增，見 PROJECT.md「test1 面板」章節；RF 的單頻/IFFT
    per-channel 模式選擇是同一天稍晚加的，見同一章節後段）。

    body = {"dc": {"0": {"ch1": <spec>, "ch2": <spec>}, "1": {...}, "2": {...}},
            "rf": {"3": {"ch1": <spec>, "ch2": <spec>}}}
    DC 每個 channel 的 <spec>（2026-08-21 改版，見 _parse_dc_channel()）：
      固定電壓：{"mode": "fixed", "v": ...}
      斜率模式：{"mode": "ramp", "start_v": ..., "slope_v_per_s": ...}
                （新增，碰到目前檔位的 ±滿幅時自動跳回 start_v，形成
                sawtooth；見 board_ctrl.arm_test1_pattern()/
                _dc_ramp_cycle_length() 說明）。同一個 module 兩個
                channel 都選 ramp 時，算出來的週期長度必須完全相同
                （硬體限制，同 RF 的 IFFT 雙 channel）。
    RF 每個 channel 的 <spec> 是下面兩種之一：
      單頻：{"mode": "single", "freq_hz": ..., "amp_v": ...}（amp_v 是
             實際電壓，peak，用 channel_full_scale_v() 換算成 -1.0~1.0）
      IFFT 梳狀波：{"mode": "ifft", "start_hz": ..., "end_hz": ...,
                    "step_hz": ..., "exclude_start_hz": ...（可選）,
                    "exclude_end_hz": ...（可選）}——不用填振幅，固定
                    0.95 滿幅自動正規化（見 board_ctrl.arm_test1_
                    pattern() 說明）。**兩個 channel 都選 IFFT 時
                    start_hz/end_hz/step_hz 必須完全相同**（buffer
                    長度只能有一個，硬體限制），這裡先擋一次，
                    arm_test1_pattern() 內部還會再防禦性檢查一次。

    DC 的振幅是實際電壓（V）——這裡先讀一次該板目前的 scale_cfg，用
    channel_full_scale_v() 換算成 -1.0~1.0 振幅比例，任何一個 channel
    超出目前檔位的範圍就整批擋下、不寫入任何東西（見下方 out_of_range
    收集邏輯，不是踩到第一個錯誤就停）。強制先把 4 個 module 全部設成
    DDR、無 ramp（跟 Reset to Zero 的 Phase 2 同一個常數 0x0），再呼叫
    arm_test1_pattern() 寫入+送一次全 group 廣播 trigger（0xF，跟 Reset
    to Zero 同一種「不用另外選 group」用法）——一次 Run 涵蓋寫入到真正
    開始播放。"""
    body = request.get_json(force=True, silent=True) or {}
    dc_raw = body.get("dc") or {}
    rf_raw = body.get("rf") or {}
    if not dc_raw and not rf_raw:
        return jsonify({"ok": False, "error": "dc and rf must not both be empty"}), 400

    with _lock:
        try:
            calib = read_calib(_fp, dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        if calib is None:
            return jsonify({"ok": False, "error": "T_QUERY reply query_type mismatch (stale reply not flushed yet? retry)"}), 409
        scale_cfg = calib["scale_cfg"]

        out_of_range = []

        dc_module_volts = {}
        try:
            for m_str, chans in dc_raw.items():
                m = int(m_str)
                fs1 = channel_full_scale_v(scale_cfg, m * 2)
                fs2 = channel_full_scale_v(scale_cfg, m * 2 + 1)
                spec1, err1 = _parse_dc_channel(m, "ch1", chans["ch1"], fs1)
                spec2, err2 = _parse_dc_channel(m, "ch2", chans["ch2"], fs2)
                errs = [e for e in (err1, err2) if e]
                if errs:
                    out_of_range.extend(errs)
                    continue
                dc_module_volts[m] = (spec1, spec2)
        except (TypeError, ValueError, KeyError) as e:
            return jsonify({"ok": False, "error": f"dc format invalid: {e}"}), 400

        rf_module_values = {}
        try:
            for m_str, chans in rf_raw.items():
                m = int(m_str)
                fs1 = channel_full_scale_v(scale_cfg, m * 2)
                fs2 = channel_full_scale_v(scale_cfg, m * 2 + 1)
                ch1_spec, err1 = _parse_rf_channel(m, "ch1", chans["ch1"], fs1)
                ch2_spec, err2 = _parse_rf_channel(m, "ch2", chans["ch2"], fs2)
                errs = [e for e in (err1, err2) if e]
                if errs:
                    out_of_range.extend(errs)
                    continue
                if ch1_spec["mode"] == "ifft" and ch2_spec["mode"] == "ifft":
                    key1 = (ch1_spec["start_hz"], ch1_spec["end_hz"], ch1_spec["step_hz"])
                    key2 = (ch2_spec["start_hz"], ch2_spec["end_hz"], ch2_spec["step_hz"])
                    if key1 != key2:
                        return jsonify({"ok": False, "error":
                            f"z{m}: 兩個 channel 都選 IFFT 時 start_hz/end_hz/step_hz 必須完全相同"}), 400
                rf_module_values[m] = {"ch1": ch1_spec, "ch2": ch2_spec}
        except (TypeError, ValueError, KeyError) as e:
            return jsonify({"ok": False, "error": f"rf format invalid: {e}"}), 400

        if out_of_range:
            return jsonify({"ok": False, "error": "voltage out of range: " + "; ".join(out_of_range)}), 400

        try:
            # 2026-08-20 新增：test2/test3 的 group_trig_scheduler 每片板子
            # 只有一個實例，如果使用者先跑了 test2/3 沒按 Stop 就切來操作
            # test1，背景排程還是會持續自主觸發，打斷/覆蓋這裡剛寫入的靜態
            # 波形。防禦性停用一次——沒有排程在跑時是無害的 no-op（送
            # depth=0 的 control-only 封包），見 board_ctrl.stop_rotation_
            # schedule() 說明。
            stop_rotation_schedule(_fp, dest_id)
            set_dac_mode_ramp(_fp, dest_id, 0x0)  # 4 個 module 全部 DDR、無 ramp
            result = arm_test1_pattern(_fp, dest_id, dc_module_volts, rf_module_values)
            trigger_playback(_fp, 0xF)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400

        # 2026-08-20 新增：原本這裡沒有更新 _status_cache[dest_id]
        # ["armed_ddr"]，跟其他 arm_*() route（見 arm_multitone_pattern
        # 那個 route 同一個做法）不一致——使用者反饋「不同電腦的瀏覽器
        # 看不到一樣的內容」，根因是 test1 的上膛結果只活在觸發那次
        # request 的回應跟 result-box 裡，沒有存進伺服器端共用狀態，
        # 別的瀏覽器（或同一個瀏覽器重新整理後）完全查不到「channel 6
        # 現在到底是什麼設定」。比照既有慣例把結果寫進 armed_ddr + 呼叫
        # _persist_status()，這樣 /api/status、SSE broadcast 都能反映
        # 出來，任何瀏覽器連上來都看得到同一份現況。_rf_channel_label()
        # 2026-08-20 稍晚搬到 module 層級（見上方），這裡不再自己定義。

        armed_ddr = _status_cache.setdefault(dest_id, {}).setdefault("armed_ddr", {})
        for m, (spec1, spec2) in dc_module_volts.items():
            fs1 = channel_full_scale_v(scale_cfg, m * 2)
            fs2 = channel_full_scale_v(scale_cfg, m * 2 + 1)
            armed_ddr[str(m)] = {"n_slots": 2,
                                  "labels": [_dc_channel_label(spec1, fs1), _dc_channel_label(spec2, fs2)],
                                  "ts": time.time()}
        for m, spec in rf_module_values.items():
            armed_ddr[str(m)] = {"n_slots": 2,
                                  "labels": [_rf_channel_label(spec["ch1"]), _rf_channel_label(spec["ch2"])],
                                  "ts": time.time()}
        _persist_status()

        response = {"ok": True, "armed_ddr": dict(armed_ddr), **result}
        broadcast("arm", response)
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/test1_stop", methods=["POST"])
def api_board_test1_stop(dest_id):
    """test1 面板「Stop」——把 4 個 module（z0-z3，即 test1 涵蓋的全部 8
    個實體聲道）全部歸零成 DC 0V 再送一次全 group trigger（2026-08-20
    新增）。不碰 RF 的頻率邏輯——歸零一律走 DC 路徑最直接。"""
    with _lock:
        try:
            stop_rotation_schedule(_fp, dest_id)  # 見 api_board_arm_test1() 同一則說明
            set_dac_mode_ramp(_fp, dest_id, 0x0)
            result = arm_test1_pattern(_fp, dest_id, {0: (0.0, 0.0), 1: (0.0, 0.0),
                                                        2: (0.0, 0.0), 3: (0.0, 0.0)}, {})
            trigger_playback(_fp, 0xF)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400

        # 2026-08-20 新增：跟 api_board_arm_test1() 同一個理由——沒有
        # 更新 armed_ddr 的話，別的瀏覽器看不到「已經按過 Stop、現在是
        # 0V」這個現況。
        armed_ddr = _status_cache.setdefault(dest_id, {}).setdefault("armed_ddr", {})
        for m in range(4):
            armed_ddr[str(m)] = {"n_slots": 2, "labels": ["DC +0.000V", "DC +0.000V"], "ts": time.time()}
        _persist_status()

        response = {"ok": True, "armed_ddr": dict(armed_ddr), **result}
        broadcast("arm", response)
    return jsonify(response)


# ── test2/test3 面板：N 組輪播（2026-08-20 新增）───────────────────────
# test2 = 5 組、test3 = 6 組，同一套 body 格式/驗證/上膛/排程邏輯，只有
# n_groups 不同，共用 _api_arm_rotation()/_api_stop_rotation()，兩個
# 面板各自的 route 只是傳不同的 n_groups/group_id。
#
# body 格式（跟 test1 的 dc/rf 格式同構，只是每個 channel 從單一值變成
# n_groups 個值的清單）：
#   {"dc": {"0": [[v1,v2], [v1,v2], ...n_groups筆...], "1": [...], "2": [...]},
#    "rf": {"3": [{"ch1": <spec>, "ch2": <spec>}, ...n_groups筆...]},
#    "durations_sec": [d0, d1, ..., d(n_groups-1)]}
# <spec> 格式跟 test1 完全相同（single/ifft，見 _parse_rf_channel()）。
#
# 「哪些 channel 要跟著輪播切換、哪些維持固定」是前端的責任：不跟著
# 切換的 channel，前端送出前直接把 Set 1 的值複製 n_groups 次填進清單
# ——後端完全不知道、也不需要知道有沒有勾選「跟著切換」這件事，跟
# board_ctrl.arm_rotation_pattern() 的設計一致。
#
# durations_sec 每一筆的上限見 board_ctrl.ROTATION_MAX_INTERVAL_SEC
# （interval_cycles 32-bit @ 100MHz，硬性上限 ≈42.9497 秒/組）；前端也要
# 各自擋一次，這裡是最後一道防線。
def _api_arm_rotation(dest_id, n_groups, group_id):
    body = request.get_json(force=True, silent=True) or {}
    dc_raw = body.get("dc") or {}
    rf_raw = body.get("rf") or {}
    durations_raw = body.get("durations_sec") or []
    if not dc_raw and not rf_raw:
        return jsonify({"ok": False, "error": "dc and rf must not both be empty"}), 400
    if len(durations_raw) != n_groups:
        return jsonify({"ok": False, "error": f"durations_sec must have exactly {n_groups} entries"}), 400
    try:
        durations = [float(d) for d in durations_raw]
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "durations_sec must all be numbers"}), 400
    for i, d in enumerate(durations):
        if not (0 < d <= ROTATION_MAX_INTERVAL_SEC):
            return jsonify({"ok": False, "error":
                f"set{i} duration {d}s must be > 0 and <= {ROTATION_MAX_INTERVAL_SEC:.4f}s"
                " (group_trig_scheduler 32-bit interval_cycles @ 100MHz 硬性上限)"}), 400

    with _lock:
        try:
            calib = read_calib(_fp, dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        if calib is None:
            return jsonify({"ok": False, "error": "T_QUERY reply query_type mismatch (stale reply not flushed yet? retry)"}), 409
        scale_cfg = calib["scale_cfg"]

        out_of_range = []

        dc_module_sets = {}
        try:
            for m_str, sets in dc_raw.items():
                m = int(m_str)
                if len(sets) != n_groups:
                    return jsonify({"ok": False, "error": f"dc z{m}: must have exactly {n_groups} sets"}), 400
                fs1 = channel_full_scale_v(scale_cfg, m * 2)
                fs2 = channel_full_scale_v(scale_cfg, m * 2 + 1)
                ratios = []
                for i, pair in enumerate(sets):
                    v1_volts, v2_volts = float(pair[0]), float(pair[1])
                    r1, r2 = v1_volts / fs1, v2_volts / fs2
                    if not (-1.0 <= r1 <= 1.0):
                        out_of_range.append(f"z{m} set{i} ch1: {v1_volts}V exceeds current ±{fs1}V range")
                    if not (-1.0 <= r2 <= 1.0):
                        out_of_range.append(f"z{m} set{i} ch2: {v2_volts}V exceeds current ±{fs2}V range")
                    ratios.append((r1, r2))
                dc_module_sets[m] = ratios
        except (TypeError, ValueError, IndexError):
            return jsonify({"ok": False, "error": "dc keys must be module indices, values must be lists of [v_ch1, v_ch2]"}), 400

        rf_module_sets = {}
        try:
            for m_str, sets in rf_raw.items():
                m = int(m_str)
                if len(sets) != n_groups:
                    return jsonify({"ok": False, "error": f"rf z{m}: must have exactly {n_groups} sets"}), 400
                fs1 = channel_full_scale_v(scale_cfg, m * 2)
                fs2 = channel_full_scale_v(scale_cfg, m * 2 + 1)
                specs = []
                for i, chans in enumerate(sets):
                    ch1_spec, err1 = _parse_rf_channel(m, f"set{i} ch1", chans["ch1"], fs1)
                    ch2_spec, err2 = _parse_rf_channel(m, f"set{i} ch2", chans["ch2"], fs2)
                    errs = [e for e in (err1, err2) if e]
                    if errs:
                        out_of_range.extend(errs)
                        continue
                    if ch1_spec["mode"] == "ifft" and ch2_spec["mode"] == "ifft":
                        key1 = (ch1_spec["start_hz"], ch1_spec["end_hz"], ch1_spec["step_hz"])
                        key2 = (ch2_spec["start_hz"], ch2_spec["end_hz"], ch2_spec["step_hz"])
                        if key1 != key2:
                            return jsonify({"ok": False, "error":
                                f"z{m} set{i}: 兩個 channel 都選 IFFT 時 start_hz/end_hz/step_hz 必須完全相同"}), 400
                    specs.append({"ch1": ch1_spec, "ch2": ch2_spec})
                rf_module_sets[m] = specs
        except (TypeError, ValueError, KeyError) as e:
            return jsonify({"ok": False, "error": f"rf format invalid: {e}"}), 400

        if out_of_range:
            return jsonify({"ok": False, "error": "voltage out of range: " + "; ".join(out_of_range)}), 400

        try:
            set_dac_mode_ramp(_fp, dest_id, 0x0)  # 4 個 module 全部 DDR、無 ramp
            result = arm_rotation_pattern(_fp, dest_id, n_groups, dc_module_sets, rf_module_sets)
            modules = sorted(set(dc_module_sets) | set(rf_module_sets))
            sched_result = start_rotation_schedule(_fp, dest_id, group_id, modules, durations)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400

        armed_ddr = _status_cache.setdefault(dest_id, {}).setdefault("armed_ddr", {})
        for m, ratios in dc_module_sets.items():
            fs1 = channel_full_scale_v(scale_cfg, m * 2)
            fs2 = channel_full_scale_v(scale_cfg, m * 2 + 1)
            labels = [f"Set{i}: {r1 * fs1:+.3f}V / {r2 * fs2:+.3f}V ({durations[i]:.1f}s)"
                      for i, (r1, r2) in enumerate(ratios)]
            armed_ddr[str(m)] = {"n_slots": n_groups, "labels": labels, "ts": time.time()}
        for m, specs in rf_module_sets.items():
            labels = [f"Set{i}: {_rf_channel_label(s['ch1'])} / {_rf_channel_label(s['ch2'])} ({durations[i]:.1f}s)"
                      for i, s in enumerate(specs)]
            armed_ddr[str(m)] = {"n_slots": n_groups, "labels": labels, "ts": time.time()}
        _persist_status()

        response = {"ok": True, "armed_ddr": dict(armed_ddr), "durations_sec": durations,
                    "group_id": group_id, **result, "schedule": sched_result["schedule"]}
        broadcast("arm", response)
    return jsonify(response)


def _api_stop_rotation(dest_id):
    """test2/test3 面板「Stop」——先停掉 group_trig_scheduler
    （stop_rotation_schedule()，不管當下播到哪一組），再比照 test1_stop
    把 4 個 module 全部歸零成 DC 0V 並送一次全 group 廣播 trigger，確保
    輸出真的靜音，不是停在最後一組的殘留波形。"""
    with _lock:
        try:
            stop_rotation_schedule(_fp, dest_id)
            set_dac_mode_ramp(_fp, dest_id, 0x0)
            result = arm_test1_pattern(_fp, dest_id, {0: (0.0, 0.0), 1: (0.0, 0.0),
                                                        2: (0.0, 0.0), 3: (0.0, 0.0)}, {})
            trigger_playback(_fp, 0xF)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400

        armed_ddr = _status_cache.setdefault(dest_id, {}).setdefault("armed_ddr", {})
        for m in range(4):
            armed_ddr[str(m)] = {"n_slots": 2, "labels": ["DC +0.000V", "DC +0.000V"], "ts": time.time()}
        _persist_status()

        response = {"ok": True, "armed_ddr": dict(armed_ddr), **result}
        broadcast("arm", response)
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/arm_test2", methods=["POST"])
def api_board_arm_test2(dest_id):
    """test2 面板「Run」——5 組輪播版的 test1，見 _api_arm_rotation()
    章節說明。group_id 固定用 1（跟 test3 的 2 錯開，純粹方便辨識；
    group_trig_scheduler 硬體本身每片板子只有一個實例，test2/test3
    仍然不能同時在同一片板子上跑，見 board_ctrl.py 對應章節）。"""
    return _api_arm_rotation(dest_id, n_groups=5, group_id=1)


@app.route("/api/board/<int:dest_id>/test2_stop", methods=["POST"])
def api_board_test2_stop(dest_id):
    return _api_stop_rotation(dest_id)


@app.route("/api/board/<int:dest_id>/arm_test3", methods=["POST"])
def api_board_arm_test3(dest_id):
    """test3 面板「Run」——6 組輪播版的 test1，見 _api_arm_rotation()
    章節說明。group_id 固定用 2。"""
    return _api_arm_rotation(dest_id, n_groups=6, group_id=2)


@app.route("/api/board/<int:dest_id>/test3_stop", methods=["POST"])
def api_board_test3_stop(dest_id):
    return _api_stop_rotation(dest_id)


@app.route("/api/board/0/ddr4_peek")
def api_ddr4_peek():
    """Debug-only（2026-08-06 新增）：直接讀 master 板 DDR4 在某個位址的
    實際內容——不是 play_pos/current_idx 這種播放位置指標，是真正存在
    DDR4 裡的樣本值。用來確認 Reset to Zero／Arm 之後，DDR4 裡的「靜音」
    slot 內容是不是真的全部是 0，而不是只信任間接指標。只能讀 master
    （dest_id=0）自己的 DDR4，沒有 dest_id 轉送機制。
    query string: ?addr=0x2000（16-byte 對齊，10進位或 0x 開頭皆可，
    預設 board_ctrl.SLOT0_ADDR）&words=64（4 的倍數，預設 64=256
    words=1024 bytes=1 個 burst，見 pad_to_burst() 的 256-sample 邊界）。
    """
    try:
        addr = int(request.args.get("addr", "0x2000"), 0)
        words = int(request.args.get("words", 64))
    except ValueError:
        return jsonify({"ok": False, "error": "addr/words must be valid integers"}), 400
    if addr % 16 != 0:
        return jsonify({"ok": False, "error": "addr must be 16-byte aligned (fp_ddr4_rw_1 silently returns the wrong block otherwise)"}), 400
    if words <= 0 or words % 4 != 0:
        return jsonify({"ok": False, "error": "words must be a positive multiple of 4"}), 400
    with _lock:
        try:
            data = []
            for i in range(0, words, 4):
                data.extend(_ddr4_read4(addr + i * 4))
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify({"ok": True, "addr": addr, "words": [f"0x{w:08X}" for w in data],
                     "all_zero": all(w == 0 for w in data)})


@app.route("/api/board/<int:dest_id>/ddr_status")
def api_board_ddr_status(dest_id):
    """讀該板 4 個 channel 的 current_idx/next_idx/mux_sel/play_pos（T_QUERY(QT_DDR_STATUS)）。"""
    with _lock:
        try:
            status = read_ddr_status(_fp, dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        if status is None:
            return jsonify({"ok": False, "error": "T_QUERY reply query_type mismatch (stale reply not flushed yet? retry)"}), 409
        response = {"ok": True, "dest_id": dest_id, **status}
        broadcast("ddr_status", response)
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/sine_status")
def api_board_sine_status(dest_id):
    """讀該板 8 個 physical channel 目前 active 側的頻率（T_QUERY(QT_
    SINE_STATUS)，2026-08-05 新增——見 read_sine_status() 檔頭：Sine
    模式原本完全沒有讀回，比 DDR 少了可觀測性，這裡補上，跟
    /ddr_status 同一種形狀（單獨一個 endpoint，module 卡片的「Read
    Playback Status」按鈕統一觸發兩者）。回傳 `channels`（8 筆，每筆
    含換算好的 `freq_hz`）而不是原始 `tuning_word`，前端不用自己重算。"""
    with _lock:
        try:
            status = read_sine_status(_fp, dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        if status is None:
            return jsonify({"ok": False, "error": "T_QUERY reply query_type mismatch (stale reply not flushed yet? retry)"}), 409
        channels = [
            {"freq_hz": ch["tuning_word"] * DAC_FS / (2 ** 32)}
            for ch in status["channels"]
        ]
        # 2026-08-10：補上 mux_sel_sync（sys_clk 側同步後的雙緩衝 active 側
        # 指標，8-bit，bit i = physical channel i 目前是 a/b 哪一側）供除錯
        # 用——之前只回傳算好的 freq_hz，這個底層欄位一直有讀到卻被濾掉，
        # 這次「Bug D」追查需要直接比對 trigger 前後 mux_sel_sync 有沒有
        # 翻轉，才知道是 trigger 沒到還是寫入沒進去。
        response = {"ok": True, "dest_id": dest_id, "channels": channels,
                    "mux_sel_sync": status.get("mux_sel_sync")}
        broadcast("sine_status", response)
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/trig_group", methods=["POST"])
def api_board_trig_group_set(dest_id):
    """設定該板 4 個 channel 的 trigger group 歸屬：body = {"groups": {"0": 0, "1": 0, "2": 1, "3": 1}}
    （channel index "0"-"3" -> group_id 0-3，key 用字串是因為 JSON object key 本來就是字串）。
    設完立刻讀回目前實際值一起回傳/推播，讓操作者確認真的生效。"""
    body = request.get_json(force=True) or {}
    groups = body.get("groups")
    if not groups or len(groups) != 4:
        return jsonify({"ok": False, "error": "groups must specify a group_id for all 4 channels (0-3)"}), 400
    try:
        channel_group_ids = {int(ch): int(gid) for ch, gid in groups.items()}
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "groups keys/values must be integers"}), 400
    with _lock:
        try:
            apply_trig_groups(_fp, dest_id, channel_group_ids)
            readback = read_trig_group(_fp, dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        if readback is None:
            return jsonify({"ok": False, "error": "T_QUERY reply query_type mismatch (stale reply not flushed yet? retry)"}), 409
        # 2026-08-04：寫進 _status_cache（不只是 broadcast 給當下連線中的
        # 分頁），這樣晚一步才連上來的新分頁 GET /api/status 也看得到最新
        # trigger group，不用等它自己再讀一次。
        if isinstance(_status_cache.get(dest_id), dict):
            _status_cache[dest_id].update(readback)
        response = {"ok": True, "dest_id": dest_id, **readback}
        broadcast("trig_group", response)
        _persist_status()
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/trig_group")
def api_board_trig_group_get(dest_id):
    """讀該板 4 個 channel 目前的 trigger group 歸屬（T_QUERY(QT_TRIGGER_GROUP)）。"""
    with _lock:
        try:
            status = read_trig_group(_fp, dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        if status is None:
            return jsonify({"ok": False, "error": "T_QUERY reply query_type mismatch (stale reply not flushed yet? retry)"}), 409
        if isinstance(_status_cache.get(dest_id), dict):
            _status_cache[dest_id].update(status)
        response = {"ok": True, "dest_id": dest_id, **status}
        broadcast("trig_group", response)
        _persist_status()
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/trig_delay", methods=["POST"])
def api_board_trig_delay_set(dest_id):
    """手動覆寫該板的 trigger 延遲補償值（aurora_clk cycle 數，0-65535）。
    body = {"delay": 120}。⚠️ sticky，沒有「取消覆寫」的封包，前端必須在
    送出前警告操作者這是不可逆動作（見 board_ctrl.set_trig_delay()）。
    設完直接複用 query_board_info() 重讀該板的 QT_BOARD_INFO（trig_delay/
    manual_delay_active 本來就是這個 query_type 的欄位）、更新
    _status_cache、用既有的 "status" 事件推播，讓畫面上原本就有的
    trig_delay/manual_delay_active 顯示自動更新，不用另外做一套 render。"""
    body = request.get_json(force=True) or {}
    delay = body.get("delay")
    if delay is None or not isinstance(delay, int) or not (0 <= delay <= 0xFFFF):
        return jsonify({"ok": False, "error": "delay must be an integer in [0, 65535]"}), 400
    with _lock:
        try:
            set_trig_delay(_fp, dest_id, delay)
            info = query_board_info(dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        if isinstance(info, dict) and "error" not in info:
            _carry_sticky_fields(dest_id, info)
            _status_cache[dest_id] = info
        broadcast("status", {"boards": _status_cache})
        _persist_status()
    return jsonify({"ok": True, "dest_id": dest_id, **info})


# ── DDR/Sine mode + sine_gen 參數（2026-08-03 新增，web UI 第一次真正
# 用到這條路徑，見 board_ctrl.py 對應區塊的完整說明）─────────────────────

@app.route("/api/board/<int:dest_id>/dac_mode_ramp", methods=["POST"])
def api_board_dac_mode_ramp_set(dest_id):
    """設定 dest_id 這片板子的 dac_mode_ramp（12-bit：bits[3:0]=4個
    module 各自 DDR(0)/Sine(1)，bits[11:4]=8個 channel 各自 ramp_en）。
    body = {"value": 0-4095}。這個 endpoint 本身的介面仍是「一次傳完整
    12-bit」（前端要送出目前畫面上全部 4 個 module+8 個 channel 的完整
    狀態），但 2026-08-05 起 board_ctrl.set_dac_mode_ramp() 內部已改成
    逐一獨立定址寫入每個 module/channel 自己的 idle 緩衝（不再是整包
    覆寫、無緩衝的舊設計）——即使這次呼叫漏帶或帶錯某個 module/channel
    的值，頂多是那個 module/channel 自己下次觸發時生效錯的值，不會像
    舊設計一樣透過任何模組的觸發就打斷其他正在播放中的 module（細節見
    board_ctrl.set_dac_mode_ramp()/set_dac_mode()/set_ramp_en() 的
    說明）。設完直接複用 query_board_info() 重讀該板的 QT_BOARD_INFO
    （dac_mode_ramp 本來就是這個 query_type 的欄位之一），更新
    _status_cache、用既有的 "status" 事件推播。"""
    body = request.get_json(force=True) or {}
    value = body.get("value")
    if value is None or not isinstance(value, int) or not (0 <= value <= 0xFFF):
        return jsonify({"ok": False, "error": "value must be an integer in [0, 4095]"}), 400
    with _lock:
        try:
            set_dac_mode_ramp(_fp, dest_id, value)
            info = query_board_info(dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        if isinstance(info, dict) and "error" not in info:
            _carry_sticky_fields(dest_id, info)
            _status_cache[dest_id] = info
        broadcast("status", {"boards": _status_cache})
        _persist_status()
    return jsonify({"ok": True, "dest_id": dest_id, **info})


@app.route("/api/board/<int:dest_id>/sine_ctrl", methods=["POST"])
def api_board_sine_ctrl_set(dest_id):
    """設定 dest_id 這片板子一個 physical channel（ch_sel 0-7）的
    sine_gen 頻率（+ 選擇性的 amp_ramp_gen 振幅斜坡）。
    body = {"ch_sel": 0-7, "freq_hz": float, "phase": int(可省略，預設0),
    "ramp": {"start_amp": 0-1, "end_amp": 0-1, "duration_sec": float,
    "loop_mode": 0/1/2} 或省略/null（不啟用振幅斜坡）}。
    純寫入，沒有對應的 QT_BOARD_INFO 欄位可以讀回確認（頻率/振幅斜坡
    參數不像 dac_mode_ramp 那樣有暴露在既有 query 裡），回傳
    board_ctrl.apply_sine_channel() 算出來的實際值（tuning_word 換算
    後的實際 Hz 等）方便前端顯示核對。"""
    body = request.get_json(force=True) or {}
    ch_sel = body.get("ch_sel")
    freq_hz = body.get("freq_hz")
    phase = body.get("phase", 0)
    ramp = body.get("ramp")
    if ch_sel is None or not isinstance(ch_sel, int) or not (0 <= ch_sel <= 7):
        return jsonify({"ok": False, "error": "ch_sel must be an integer in [0, 7]"}), 400
    # 0 是合法值（2026-08-04，使用者要求「0時是輸出0V」）：tuning_word=0
    # 讓 phase_acc 停在 phase_stage（apply_sine_channel() 預設 0），對應
    # sine_lut[0]=0（見 rtl/sine_gen.v 的 trig_start/phase_acc 邏輯 +
    # gen_sine_lut.py 自己印出的 "sample[0]=0000 (expect 0000, sin(0)=0)"），
    # 是真正的 0V DC 輸出，只有負數才是真的無效輸入。
    if freq_hz is None or not isinstance(freq_hz, (int, float)) or freq_hz < 0:
        return jsonify({"ok": False, "error": "freq_hz must be a non-negative number"}), 400
    if not isinstance(phase, int):
        return jsonify({"ok": False, "error": "phase must be an integer"}), 400
    if ramp is not None:
        required = ("start_amp", "end_amp", "duration_sec", "loop_mode")
        if not all(k in ramp for k in required):
            return jsonify({"ok": False, "error": f"ramp must include all of {required}"}), 400
    with _lock:
        try:
            result = apply_sine_channel(_fp, dest_id, ch_sel, freq_hz, phase=phase, ramp=ramp)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify({"ok": True, "dest_id": dest_id, "ch_sel": ch_sel, **result})


@app.route("/api/board/<int:dest_id>/sine_slot", methods=["POST"])
def api_board_sine_slot_set(dest_id):
    """設定 dest_id 這片板子一個 physical channel（ch_sel 0-7）的 Sine
    N-slot 排程表（2026-08-05 新增，取代 /sine_ctrl 原本承擔的「Sine
    頻率」角色——見 board_ctrl.apply_sine_slots() 檔頭：slot table 是
    獨立儲存區，不會被 /sine_ctrl 的單值寫入路徑碰到，兩條路徑同時
    開放給操作者用容易互相覆蓋，所以 web UI 只保留這一條當 Sine 頻率
    的唯一入口，`/sine_ctrl` 的 freq_hz 用法保留給其他既有腳本/流程
    使用、不從這個 endpoint 呼叫）。

    body = {"ch_sel": 0-7, "freqs_hz": [float, ...]（1-SINE_N_SLOT 筆，
    依序對應 slot 0,1,2,...）, "ramp_en": bool,
    "ramp": {"start_amp": 0-1, "end_amp": 0-1, "duration_sec": float,
    "loop_mode": 0/1/2} 或省略/null（ramp_en=true 時必填，套用到全部
    slot 共用同一組振幅包絡形狀）}。

    commit 之後這個 channel 的下一次 Trigger 就會立刻顯示 slot 0（不
    需要額外的暖機 trigger，見 rtl/sine_ctrl_regs.v 檔頭），之後每次
    Trigger（不管是手動、群組觸發、還是 trig_timer 自動排程）依序
    揭露下一個 slot，在 depth 邊界正確循環。純寫入，沒有對應的
    QT_BOARD_INFO 欄位可以讀回確認（跟 /sine_ctrl 同一種限制）。"""
    body = request.get_json(force=True) or {}
    ch_sel = body.get("ch_sel")
    freqs_hz = body.get("freqs_hz")
    ramp_en = bool(body.get("ramp_en"))
    ramp = body.get("ramp") if ramp_en else None
    if ch_sel is None or not isinstance(ch_sel, int) or not (0 <= ch_sel <= 7):
        return jsonify({"ok": False, "error": "ch_sel must be an integer in [0, 7]"}), 400
    if not isinstance(freqs_hz, list) or not freqs_hz:
        return jsonify({"ok": False, "error": "freqs_hz must be a non-empty list"}), 400
    for f in freqs_hz:
        if not isinstance(f, (int, float)) or f < 0:
            return jsonify({"ok": False, "error": "every freq_hz must be a non-negative number"}), 400
    if ramp_en:
        required = ("start_amp", "end_amp", "duration_sec", "loop_mode")
        if not ramp or not all(k in ramp for k in required):
            return jsonify({"ok": False, "error": f"ramp_en=true requires ramp with all of {required}"}), 400
    with _lock:
        try:
            set_ramp_en(_fp, dest_id, ch_sel, ramp_en)
            result = apply_sine_slots(_fp, dest_id, ch_sel, freqs_hz, ramp=ramp)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        # 2026-08-07 新增：跟 armed_ddr 同一種「純寫入無讀回,靠本介面自己
        # 記錄」處理方式（見 _STICKY_KEYS）——QT_SINE_STATUS 只能讀到目前
        # active 那一組的頻率,讀不到整張 slot table,module 卡片要顯示
        # 「上次送出的完整排程」只能靠這裡自己記。
        sine_slots = _status_cache.setdefault(dest_id, {}).setdefault("sine_slots", {})
        sine_slots[str(ch_sel)] = {"freqs_hz": freqs_hz, "ramp_en": ramp_en, "ts": time.time()}
        _persist_status()
        # 這個 endpoint 沒有對應的 SSE 事件（跟 arm 不同），sine_slots 整包
        # 直接放進回應，前端呼叫端自己塞回 lastStatus[dest_id].sine_slots
        # 重畫 module 卡片（見 commitSineSlotsForModule()/applyBulkSine()）。
        sine_slots_out = dict(sine_slots)
    return jsonify({"ok": True, "dest_id": dest_id, "ch_sel": ch_sel, "ramp_en": ramp_en,
                     "sine_slots": sine_slots_out, **result})


# ── trig_timer / group_trig_scheduler（2026-08-05 新增）────────────────────
# 這兩個功能的 RTL/host wrapper（board_ctrl.set_trig_timer()/
# set_group_sched()）2026-08-04 就做完並上機驗證通過了（見 test_trig_
# timer.py/test_trig_timer_16slot.py/test_group_trig_scheduler.py），但
# web app 一直沒有對應的 endpoint/UI，只能用獨立腳本操作——這裡補上。

@app.route("/api/board/<int:dest_id>/trig_timer", methods=["POST"])
def api_board_trig_timer_set(dest_id):
    """設定 dest_id 這片板子一個 module（port 0-3，對應 z0-z3）的
    trig_timer 排程。body = {"port": 0-3, "intervals_sec": [float, ...]
    (1-16 筆，run=true 時必填), "loop_en": bool, "run": bool}。
    intervals_sec 為空清單只送 stop 用的 control-only 封包（比照
    board_ctrl.set_trig_timer() 的既有語意）。run=True 時會讓 trig_timer
    進入 armed 狀態，等下一次這個 module 所屬 trigger group 的 Trigger
    當 first_trigger 才真正開始跑；run=False 立即停止/解除 armed。
    純寫入，沒有對應的讀回機制（跟 sine_ctrl 同一種限制）——這裡送出去
    的 body 本身就是唯一的記錄來源，成功後直接寫進 _status_cache[dest_id]
    ["trig_timer"][port]（2026-08-06 新增，見 _persist_status()），不是
    等硬體讀回來才知道現在設的是什麼。"""
    body = request.get_json(force=True) or {}
    port = body.get("port")
    intervals_sec = body.get("intervals_sec", [])
    loop_en = bool(body.get("loop_en", False))
    run = bool(body.get("run", False))
    if port is None or not isinstance(port, int) or not (0 <= port <= 3):
        return jsonify({"ok": False, "error": "port must be an integer in [0, 3]"}), 400
    if not isinstance(intervals_sec, list):
        return jsonify({"ok": False, "error": "intervals_sec must be a list"}), 400
    if run and not (1 <= len(intervals_sec) <= 16):
        return jsonify({"ok": False, "error": "intervals_sec must have 1-16 entries when run=true"}), 400
    for v in intervals_sec:
        if not isinstance(v, (int, float)) or v <= 0:
            return jsonify({"ok": False, "error": "each interval must be a positive number of seconds"}), 400
    intervals_cycles = [round(v * DAC_FS) for v in intervals_sec]
    with _lock:
        try:
            set_trig_timer(_fp, dest_id, port, intervals_cycles, loop_en=loop_en, run=run)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        response = {"ok": True, "dest_id": dest_id, "port": port, "intervals_sec": intervals_sec,
                     "loop_en": loop_en, "run": run, "n_slots": len(intervals_cycles)}
        board = _status_cache.setdefault(dest_id, {})
        if not isinstance(board, dict):
            board = {}
            _status_cache[dest_id] = board
        board.setdefault("trig_timer", {})[str(port)] = {
            "intervals_sec": intervals_sec, "loop_en": loop_en, "run": run,
        }
        broadcast("trig_timer_state", response)
        _persist_status()
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/group_sched", methods=["POST"])
def api_board_group_sched_set(dest_id):
    """設定 dest_id 這片板子的 group_trig_scheduler 排程——只有 is_master
    的板子送出的排程才會真的產生跨板廣播效果（跟 /api/trigger 的既有
    限制一致），前端只在 master 板的 Edit Settings 顯示這張卡片，這裡
    後端不額外擋非 master 呼叫（跟其他 endpoint 一致，不做本來就有
    UI 層限制的重複檢查）。
    body = {"schedule": [[interval_sec, group_select], ...] (1-16 筆，
    group_select 是 4-bit mask，bit N = fire group N，run=true 時必填），
    "loop_en": bool, "run": bool}。run=True 時立刻開始跑（不像
    trig_timer 要等外部 first_trigger——這個模組本身就是要「自己當
    觸發源」）；run=False 立即停止。"""
    body = request.get_json(force=True) or {}
    schedule = body.get("schedule", [])
    loop_en = bool(body.get("loop_en", False))
    run = bool(body.get("run", False))
    if not isinstance(schedule, list):
        return jsonify({"ok": False, "error": "schedule must be a list"}), 400
    if run and not (1 <= len(schedule) <= 16):
        return jsonify({"ok": False, "error": "schedule must have 1-16 entries when run=true"}), 400
    entries_cycles = []
    for entry in schedule:
        if not isinstance(entry, list) or len(entry) != 2:
            return jsonify({"ok": False, "error": "each schedule entry must be [interval_sec, group_select]"}), 400
        interval_sec, group_select = entry
        if not isinstance(interval_sec, (int, float)) or interval_sec <= 0:
            return jsonify({"ok": False, "error": "interval_sec must be a positive number of seconds"}), 400
        if not isinstance(group_select, int) or not (0 <= group_select <= 0xF):
            return jsonify({"ok": False, "error": "group_select must be an integer in [0, 15]"}), 400
        entries_cycles.append((round(interval_sec * DAC_FS), group_select))
    with _lock:
        try:
            set_group_sched(_fp, dest_id, entries_cycles, loop_en=loop_en, run=run)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify({"ok": True, "dest_id": dest_id, "loop_en": loop_en, "run": run,
                     "n_slots": len(entries_cycles)})


# ── Calibration tab endpoints（2026-07-31 新增，見 calib_ctrl.py）───────────

def _read_flash_best_effort(dest_id):
    """QT_FLASH_STATUS 擴充版還沒 build 回來之前，硬體不一定認得這個
    query_type（見 flash_ctrl.py 檔頭說明）——這裡刻意不讓 Flash 欄位
    查詢失敗擋住 Live 欄位的正常顯示，抓例外/None 都當成「Flash 欄位
    暫時無法讀取」，不是整個 request 失敗。"""
    try:
        return read_flash_calib(_fp, dest_id)
    except Exception as e:
        return {"error": str(e)}


@app.route("/api/board/<int:dest_id>/calib")
def api_board_calib_get(dest_id):
    """讀該板目前的 scale_cfg/amp_ctrl/calib_coef（T_QUERY(QT_CALIB_STATUS)，
    Live 欄位）+ flash 目前存的內容（T_QUERY(QT_FLASH_STATUS)，Flash 欄位，
    見 flash_ctrl.py 檔頭「Live 跟 Flash 不強求即時同步」的設計說明）。"""
    with _lock:
        try:
            status = read_calib(_fp, dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        if status is None:
            return jsonify({"ok": False, "error": "T_QUERY reply query_type mismatch (stale reply not flushed yet? retry)"}), 409
        response = {"ok": True, "dest_id": dest_id, **status, "flash": _read_flash_best_effort(dest_id)}
        broadcast("calib", response)
    return jsonify(response)


def _reread_calib_and_broadcast(dest_id):
    """set 類 endpoint 共用：寫完立刻重讀整包校準狀態一起回傳/推播，
    讓操作者看到真正落地的值，不是只看到「送出成功」。"""
    status = read_calib(_fp, dest_id)
    response = {"ok": True, "dest_id": dest_id, **(status or {})}
    broadcast("calib", response)
    return response


@app.route("/api/board/<int:dest_id>/scale_cfg", methods=["POST"])
def api_board_scale_cfg_set(dest_id):
    """body = {"value": 0-255}（8-bit，每個 bit 一個通道的量程檔位）。"""
    body = request.get_json(force=True) or {}
    value = body.get("value")
    if value is None or not isinstance(value, int) or not (0 <= value <= 0xFF):
        return jsonify({"ok": False, "error": "value must be an integer in [0, 255]"}), 400
    with _lock:
        try:
            set_scale_cfg(_fp, dest_id, value)
            response = _reread_calib_and_broadcast(dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/amp_ctrl", methods=["POST"])
def api_board_amp_ctrl_set(dest_id):
    """body = {"channel": 0-7 或 null(廣播全部8個), "value": 0-262143}
    （18-bit 定點，1.0=0x10000，換算交給前端做，這裡收原始整數）。"""
    body = request.get_json(force=True) or {}
    channel = body.get("channel")
    value = body.get("value")
    if channel is not None and not (isinstance(channel, int) and 0 <= channel <= 7):
        return jsonify({"ok": False, "error": "channel must be null or an integer in [0, 7]"}), 400
    if value is None or not isinstance(value, int) or not (0 <= value <= 0x3FFFF):
        return jsonify({"ok": False, "error": "value must be an integer in [0, 262143]"}), 400
    with _lock:
        try:
            set_amp_ctrl(_fp, dest_id, channel, value)
            response = _reread_calib_and_broadcast(dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/amp_ctrl_bulk", methods=["POST"])
def api_board_amp_ctrl_bulk_set(dest_id):
    """一次設定多個 channel（UI 只有一個 Set 按鈕，不用每個 channel 各按
    一次）。body = {"values": {"0": 65536, "1": 65536, ...}}（channel
    index 字串 -> 18-bit 值，JSON object key 本來就是字串）。"""
    body = request.get_json(force=True) or {}
    raw_values = body.get("values")
    if not raw_values:
        return jsonify({"ok": False, "error": "values must not be empty"}), 400
    try:
        values = {int(ch): int(v) for ch, v in raw_values.items()}
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "values keys/values must be integers"}), 400
    for ch, v in values.items():
        if not (0 <= ch <= 7):
            return jsonify({"ok": False, "error": f"channel {ch} out of range [0, 7]"}), 400
        if not (0 <= v <= 0x3FFFF):
            return jsonify({"ok": False, "error": f"value for channel {ch} out of range [0, 262143]"}), 400
    with _lock:
        try:
            set_amp_ctrl_bulk(_fp, dest_id, values)
            response = _reread_calib_and_broadcast(dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/calib_coef_bulk", methods=["POST"])
def api_board_calib_coef_bulk_set(dest_id):
    """一次設定多個 calib_coef index，同上但 index 範圍是 0-31。
    body = {"values": {"0": 65536, "16": 0, ...}}。"""
    body = request.get_json(force=True) or {}
    raw_values = body.get("values")
    if not raw_values:
        return jsonify({"ok": False, "error": "values must not be empty"}), 400
    try:
        values = {int(idx): int(v) for idx, v in raw_values.items()}
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "values keys/values must be integers"}), 400
    for idx, v in values.items():
        if not (0 <= idx <= 31):
            return jsonify({"ok": False, "error": f"index {idx} out of range [0, 31]"}), 400
        if not (0 <= v <= 0x3FFFF):
            return jsonify({"ok": False, "error": f"value for index {idx} out of range [0, 262143]"}), 400
    with _lock:
        try:
            set_calib_coef_bulk(_fp, dest_id, values)
            response = _reread_calib_and_broadcast(dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/calib_coef", methods=["POST"])
def api_board_calib_coef_set(dest_id):
    """body = {"index": 0-31, "value": 0-262143}（18-bit，見 calib_ctrl.py
    檔頭的 index bit 排列說明）。"""
    body = request.get_json(force=True) or {}
    index = body.get("index")
    value = body.get("value")
    if index is None or not isinstance(index, int) or not (0 <= index <= 31):
        return jsonify({"ok": False, "error": "index must be an integer in [0, 31]"}), 400
    if value is None or not isinstance(value, int) or not (0 <= value <= 0x3FFFF):
        return jsonify({"ok": False, "error": "value must be an integer in [0, 262143]"}), 400
    with _lock:
        try:
            set_calib_coef(_fp, dest_id, index, value)
            response = _reread_calib_and_broadcast(dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/calib_reset", methods=["POST"])
def api_board_calib_reset(dest_id):
    """T_CALIB_RST：只重置 calib_coef 陣列回硬體預設值，不影響 scale_cfg/amp_ctrl。"""
    with _lock:
        try:
            reset_calib_coef(_fp, dest_id)
            response = _reread_calib_and_broadcast(dest_id)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify(response)


# ── 「Save to Flash」endpoint（2026-07-31 新增，見 flash_ctrl.py）───────────
# 跟上面的 Set/Set All 是完全分開的第二層功能：這裡不接受 body 傳新值，
# 一律先讀目前 Live 值（read_calib()）再存進 flash——Save to Flash 存的
# 是「目前已生效、畫面上看到的值」，不是輸入框裡還沒送出的草稿。
#
# ⚠️ 依賴 QT_FLASH_STATUS 擴充版（含 scale_cfg/calib_coef 內容，不只
# busy/done/err），Linux 端還在做，這兩個 endpoint 目前只能語法檢查，
# 硬體實測要等 build 傳回來，見 NOTES.md「擴充：QT_FLASH_STATUS 加上
# flash 實際內容」章節。

@app.route("/api/board/<int:dest_id>/scale_cfg/save_flash", methods=["POST"])
def api_board_scale_cfg_save_flash(dest_id):
    """把目前 live 的 scale_cfg 存進 flash sector 1（T_FLASH_TARGET_SEL+
    T_FLASH_WRITE_DATA），輪詢存檔完成後回傳最新 Flash 欄位內容。"""
    with _lock:
        try:
            live = read_calib(_fp, dest_id)
            if live is None:
                return jsonify({"ok": False, "error": "T_QUERY reply query_type mismatch (stale reply not flushed yet? retry)"}), 409
            flash_status = save_scale_cfg_to_flash(_fp, dest_id, live["scale_cfg"])
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        response = {"ok": True, "dest_id": dest_id, "scale_cfg": live["scale_cfg"], "flash": flash_status}
        broadcast("calib_flash", response)
    return jsonify(response)


@app.route("/api/board/<int:dest_id>/calib_coef/save_flash", methods=["POST"])
def api_board_calib_coef_save_flash(dest_id):
    """把目前 32 筆 live calib_coef 全部存進 flash sector 3，輪詢存檔
    完成後回傳最新 Flash 欄位內容。"""
    with _lock:
        try:
            live = read_calib(_fp, dest_id)
            if live is None:
                return jsonify({"ok": False, "error": "T_QUERY reply query_type mismatch (stale reply not flushed yet? retry)"}), 409
            flash_status = save_calib_coef_to_flash(_fp, dest_id, live["calib_coef"])
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        response = {"ok": True, "dest_id": dest_id, "calib_coef": live["calib_coef"], "flash": flash_status}
        broadcast("calib_flash", response)
    return jsonify(response)


@app.route("/api/si5332/status")
def api_si5332_status():
    """讀 Si5332 USYS_STAT。跟三片 AWG 板子的 _lock/_fp 完全無關（獨立
    USB 裝置），只用自己的 _si5332_lock 避免兩個瀏覽器同時觸發 CBPro
    CLI。手動查詢，不是背景輪詢。"""
    with _si5332_lock:
        result = si5332_read_status()
    broadcast("si5332_status", result)
    return jsonify(result)


@app.route("/api/si5332/reconfigure", methods=["POST"])
def api_si5332_reconfigure():
    """重新寫入 Si5332 時脈設定——會影響目前三片板子共用的外部 DAC 時脈
    來源，前端必須先 confirm() 過才能送到這裡。完成後順便重讀一次狀態，
    回應跟廣播都直接附上最新 status，不用使用者再多按一次查詢。"""
    with _si5332_lock:
        reconf = si5332_reconfigure()
        status = si5332_read_status()
    response = {**reconf, "status": status}
    broadcast("si5332_reconfigure", response)
    return jsonify(response)


@app.route("/api/si5332/outputs")
def api_si5332_outputs():
    """讀 6 路輸出（OUT0_OE..OUT5_OE）目前開關狀態 + 固定頻率值。手動
    查詢，不是背景輪詢；跟 _lock/_fp 無關（獨立裝置），只用 _si5332_lock。"""
    with _si5332_lock:
        result = si5332_read_outputs()
    broadcast("si5332_outputs", result)
    return jsonify(result)


@app.route("/api/si5332/outputs/<int:channel>", methods=["POST"])
def api_si5332_output_set(channel):
    """開關單一輸出通道（OUTx_OE）。body = {"enabled": true/false}。
    這是 volatile 的即時切換，不影響 RAM 內其他設定，跟 reconfigure
    是完全不同的操作（見 si5332_ctrl.py 模組說明）——但關掉一個目前正
    餵給某片板子的通道一樣會讓那片板子瞬間失去外部時脈，所以前端一樣
    要在關閉動作前 confirm()。完成後順便重讀一次全部 6 路，回應/廣播
    都附上最新狀態。"""
    body = request.get_json(force=True, silent=True) or {}
    if "enabled" not in body:
        return jsonify({"ok": False, "error": "missing 'enabled' in request body"}), 400
    if not (0 <= channel < 6):
        return jsonify({"ok": False, "error": f"channel must be 0-5, got {channel}"}), 400
    with _si5332_lock:
        try:
            set_result = si5332_set_output_enable(channel, bool(body["enabled"]))
        except ValueError as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        outputs = si5332_read_outputs()
        if outputs["ok"]:
            si5332_save_output_state(outputs["channels"])
    response = {**set_result, "outputs": outputs}
    broadcast("si5332_outputs", outputs)
    return jsonify(response)


# ── 梳狀波形增益校正表（2026-08-19 新增，同日改版為「資料夾存多份+
# 標記其中一份 active」設計，見 multitone_scan.py 檔頭說明）─────────────
# 放大器對不同頻率的增益不同，多音梳狀波形（multitone_scan.
# precompute_full_sum()）每個諧波需要各自乘上一個校正係數。跟任何板子
# 無關（純 host 端算波形樣本用的表格），不用 _lock/_fp。

@app.route("/api/multitone_gain_correction/list")
def api_multitone_gain_correction_list():
    """列出資料夾裡所有已儲存的校正表，附上哪一個目前是 active。"""
    active = get_active_gain_correction_name()
    tables = [{"name": name, "active": name == active} for name in list_gain_correction_tables()]
    return jsonify({"ok": True, "tables": tables, "active": active,
                     "expected_count": MULTITONE_N_TONES})


@app.route("/api/multitone_gain_correction", methods=["POST"])
def api_multitone_gain_correction_save():
    """上傳一份新的校正表存進資料夾（不會自動變成 active，上傳跟啟用
    是兩個獨立步驟，見上方模組註解）。body = {"name": "amp_v1.csv",
    "values": [1.0, 1.02, ...]}，長度必須精確等於 MULTITONE_N_TONES
    （依 bin 1..N_TONES 順序），前端已經在 client 端逐行解析+驗證過
    數值合法性（照抄既有 slot 檔案上傳的 handleSlotFileChange() 慣例），
    這裡只做長度檢查。"""
    body = request.get_json(force=True) or {}
    name = body.get("name")
    values = body.get("values")
    if not name or not isinstance(name, str):
        return jsonify({"ok": False, "error": "missing 'name'"}), 400
    if not isinstance(values, list) or len(values) != MULTITONE_N_TONES:
        return jsonify({"ok": False, "error": f"values must be a list of exactly {MULTITONE_N_TONES} numbers"}), 400
    try:
        values = [float(v) for v in values]
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "values must all be numbers"}), 400
    try:
        save_gain_correction_table(name, values)
    except ValueError as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify({"ok": True, "name": name, "count": len(values)})


@app.route("/api/multitone_gain_correction/activate", methods=["POST"])
def api_multitone_gain_correction_activate():
    """設定哪一份已儲存的校正表是 active。body = {"name": "amp_v1.csv"}。
    這個選擇存成檔案（multitone_scan.GAIN_CORRECTION_ACTIVE_FILE），
    重開機/重啟 server 後 load_gain_correction() 一樣找得到。"""
    body = request.get_json(force=True) or {}
    name = body.get("name")
    if not name or not isinstance(name, str):
        return jsonify({"ok": False, "error": "missing 'name'"}), 400
    try:
        set_active_gain_correction(name)
    except FileNotFoundError:
        return jsonify({"ok": False, "error": f"table '{name}' not found"}), 404
    return jsonify({"ok": True, "active": name})


@app.route("/api/multitone_gain_correction/<name>", methods=["DELETE"])
def api_multitone_gain_correction_delete(name):
    """刪除一份已儲存的校正表（2026-08-20 新增，per 使用者要求新分頁要
    能刪除）。刪掉的剛好是目前 active 的那份時，`delete_gain_
    correction_table()` 會一併清掉 active 記錄（見該函式說明）。"""
    try:
        delete_gain_correction_table(name)
    except FileNotFoundError:
        return jsonify({"ok": False, "error": f"table '{name}' not found"}), 404
    except ValueError as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify({"ok": True, "name": name})


if __name__ == "__main__":
    _load_status()
    try:
        connect()
    except (Exception, SystemExit) as e:
        print(f"[web] warning: no master board found at startup ({e}); "
              f"server will still start -- use Initialize System once a board is attached")
    refresh_all_status()
    # 2026-08-14：獨立印出實際網卡 IP（跟 /api/server_info 同一套
    # hostname -I 邏輯），不依賴 Werkzeug 自己的啟動 banner——啟動當下
    # 網路介面還沒就緒時，banner 只會列出 127.0.0.1，看不到真正的
    # 區網 IP，這樣 log 裡至少有一份可靠記錄。
    try:
        _ips = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=5).stdout.split()
        _ips = [ip for ip in _ips if "." in ip]
        print(f"[web] server IP(s): {', '.join(_ips) if _ips else 'unknown'} : port 5000")
    except Exception as e:
        print(f"[web] warning: could not determine server IP: {e}")
    # host="0.0.0.0"：開放區網存取（2026-07-31 起不再只限 localhost，
    # 之後會放到 Pi 5B 上當共用 control station）。threaded=True：
    # 讓 /api/events 的長連線不會擋住其他 API request（Flask 開發伺服器
    # 預設單執行緒，SSE 這種常駐連線一定要開 threaded 才不會卡死其他人）。
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
