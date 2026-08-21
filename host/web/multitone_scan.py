"""
multitone_scan.py -- 500Hz~500kHz、500Hz 間距、1000 音 Schroeder phase
梳狀測試波形產生（board_ctrl.arm_multitone_pattern() 用）。

2026-08-14：原本這裡還有一個動態 notch 掃描的 ScanController（背景執行緒
逐步排除單一頻率、2-slot 乒乓切換上機），但 compute_batch_steps() 的批次
大小安全係數算式在實測上傳速度（~13MB/s）下對任何 K 都無法收斂，導致
K 一路衝到 N_TONES（1000），單一 batch 炸出 1000*repeat_count*N ≈ 100 億
樣本，Pi 5B 上直接被 OOM killer 砍掉 process（見 NOTES.md 對應章節）。
已確認 precompute_full_sum() 這個「完整 1000 音、不排除任何頻率」的
波形本身沒有問題（FFT 驗證 + 上機示波器確認），改成透過 Bulk DDR Setup
面板的「Generate multitone test pattern」按鈕一次性上膛播放，不需要動態
掃描/批次上傳，因此拿掉 ScanController 那整套。
"""
import os
import numpy as np

FS = 100_000_000
BURST_SAMPLES = 256  # 跟 awg_common.py 同一個常數，DDR buffer 長度必須是這個的倍數
# 2026-08-18：N 從 200_000 改成 200_192（= 256*782，burst 邊界的整數倍）
# ——原本的 200_000 不是 256 的倍數，board_ctrl.arm_multitone_pattern()
# 上膛時會用 pad_to_burst() 補 192 個 0 樣本湊成 256 倍數，導致實際寫進
# DDR4 的 buffer 長度（200192）跟波形本身的週期（200000）對不上，波形
# 尾端多出一小段不連續的靜音，也讓「用 buffer 內第幾個 sample 換算諧波
# 相位」這件事完全算不準（IFFT 掃頻相減功能需要 buffer 長度精確等於
# 週期，見 rtl/notch_sine_gen.v/notch_sweep_ctrl.v 檔頭說明、
# PROJECT.md 2026-08-18 對應章節的 Opus 覆核分析）。改成 200192 後不用
# 補零，buffer 長度精確等於週期，順便也修掉這個一直存在、沒人特別
# 注意到的尾端不連續小 glitch。頻率間距從精確 500Hz 微幅變成
# FS/N=499.5205Hz（bin 1000 從 500kHz 變成 499.5205kHz），這個誤差量級
# 遠低於先前上機驗證過的可辨識度門檻（2026-08-14 記錄：N=200000 版本
# 尾端補零造成的誤差是頻率間距從 500.00Hz 偏到 499.52Hz，跟這次改的
# 誤差量級幾乎一樣，當時已確認頻譜儀上分辨不出差異）。
N = 200_192
N_TONES = 1000               # bin 1..1000 = 500Hz..~500kHz

# 2026-08-19 新增：後端放大器對不同頻率的增益不同，梳狀波形產生時每個
# 諧波需要各自乘上一個校正係數，輸出到示波器上才會（趨近）平坦，不被
# 放大器的頻率響應扭曲。校正表是使用者實測放大器增益曲線後才會有的
# 資料，格式是一行一個係數、共 N_TONES 行、依 bin 1..N_TONES 順序（跟
# host/web/index.html 既有的「一行一個樣本值」上傳慣例一致）。
#
# 存成「資料夾 + 多份具名檔案 + 一份記錄目前 active 是哪個檔名」的設計
# （不是單一固定檔案）——使用者可能量測過不只一組放大器/設定，上傳
# 之後不會馬上生效，要用哪一份需要另外「點一下」設成 active，這個選擇
# 本身也存成檔案，重開機/重啟 server 後 load_gain_correction() 一樣
# 找得到。folder 是空的、或沒有 active 記錄、或 active 指向的檔案不見
# 了，都 fallback 成全部 1.0（完全不影響現有行為）。
GAIN_CORRECTION_DIR = os.path.join(os.path.dirname(__file__), "multitone_gain_correction")
GAIN_CORRECTION_ACTIVE_FILE = os.path.join(GAIN_CORRECTION_DIR, "_active.txt")


def list_gain_correction_tables():
    """回傳資料夾裡所有已儲存的校正表檔名（不含路徑），按檔名排序。"""
    if not os.path.isdir(GAIN_CORRECTION_DIR):
        return []
    return sorted(f for f in os.listdir(GAIN_CORRECTION_DIR)
                  if f.endswith(".csv") and os.path.isfile(os.path.join(GAIN_CORRECTION_DIR, f)))


def get_active_gain_correction_name():
    """回傳目前設為 active 的檔名，沒設定過（或指向的檔案不存在）回傳 None。"""
    if not os.path.exists(GAIN_CORRECTION_ACTIVE_FILE):
        return None
    with open(GAIN_CORRECTION_ACTIVE_FILE, "r", encoding="utf-8") as f:
        name = f.read().strip()
    if not name or not os.path.exists(os.path.join(GAIN_CORRECTION_DIR, name)):
        return None
    return name


def set_active_gain_correction(name):
    """設定哪個已儲存的檔案是 active（下次 arm_multitone_pattern() 呼叫
    load_gain_correction() 就會用這個）。name 必須是已經存在的檔案，
    不存在就丟 FileNotFoundError（呼叫端負責先 save_gain_correction_
    table() 或確認清單裡有這個檔名）。"""
    path = _gain_correction_path(name)
    if not os.path.exists(path):
        raise FileNotFoundError(name)
    with open(GAIN_CORRECTION_ACTIVE_FILE, "w", encoding="utf-8") as f:
        f.write(name)


def _gain_correction_path(name):
    """把使用者給的檔名轉成資料夾內的完整路徑，順便擋掉路徑穿越
    （2026-08-20 新增，這個 name 之後會透過網頁 API 直接來自使用者輸入，
    比照專案其餘 web API 對使用者輸入檔名的處理慣例，不能直接信任
    os.path.join 的結果，只允許單純檔名，不能包含路徑分隔符/`..`）。"""
    base = os.path.basename(name)
    if not base or base != name or base in (".", ".."):
        raise ValueError(f"invalid name: {name!r}")
    return os.path.join(GAIN_CORRECTION_DIR, base)


def save_gain_correction_table(name, values):
    """把一份校正表存進資料夾（不自動設成 active，上傳跟啟用是兩個
    獨立步驟）。values 長度必須精確等於 N_TONES，否則丟 ValueError。
    name 已存在就直接覆蓋（符合「重新上傳同名檔案＝更新」的直覺）。

    2026-08-19：每個係數必須是 0.0~1.0（不是任意正數）——這裡的語意是
    「相對放大器增益最弱那個頻率的衰減比例」：增益最弱的頻率係數=1.0
    （已經是驅動極限，沒有再放大的空間），其他頻率的係數<1.0（增益比
    最弱點好，所以要往下衰減，讓修正後的整體輸出趨近平坦）。這樣設計
    保證修正後的訊號永遠不會比修正前需要更多輸出裕度（只衰減、不放
    大），避免疊加後意外超過原本的飽和保護上限。"""
    if len(values) != N_TONES:
        raise ValueError(f"expected exactly {N_TONES} values, got {len(values)}")
    if any(v < 0.0 or v > 1.0 for v in values):
        raise ValueError("every coefficient must be between 0.0 and 1.0 (relative to the weakest-gain frequency)")
    path = _gain_correction_path(name)
    os.makedirs(GAIN_CORRECTION_DIR, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(str(v) for v in values) + "\n")


def delete_gain_correction_table(name):
    """刪除一份已儲存的校正表（2026-08-20 新增，per 使用者要求新分頁要
    能刪除）。刪除目前 active 的那份時，一併清掉 active 記錄（不留著
    指向不存在檔案的殘留狀態，load_gain_correction() 才不會需要額外
    處理這個邊界情況）。name 不存在就丟 FileNotFoundError。"""
    path = _gain_correction_path(name)
    if not os.path.exists(path):
        raise FileNotFoundError(name)
    os.remove(path)
    if get_active_gain_correction_name() == name:
        os.remove(GAIN_CORRECTION_ACTIVE_FILE)


def load_gain_correction():
    """讀目前 active 的校正表，回傳長度 N_TONES 的 np.array。沒有設定
    active（or 對應檔案不存在/格式不對）都 fallback 成全部 1.0（不整檔
    拒絕，因為這裡是伺服器端讀檔案不是使用者當下互動，比照
    apply_saved_output_state() 對「檔案不存在」的處理慣例——安靜跳過，
    不是錯誤）。"""
    name = get_active_gain_correction_name()
    if name is None:
        return np.ones(N_TONES)
    path = os.path.join(GAIN_CORRECTION_DIR, name)
    try:
        with open(path, "r", encoding="utf-8") as f:
            values = [float(line.strip()) for line in f if line.strip()]
    except (OSError, ValueError):
        return np.ones(N_TONES)
    if len(values) != N_TONES:
        return np.ones(N_TONES)
    return np.array(values)


def precompute_full_sum(gain_correction=None, exclude_bins=None):
    """算一次完整 1000 音總和（Schroeder phase，PAPR 最小化），回傳
    (x_full, bins, w, phis, n)。x_full 值域約 -N_TONES..N_TONES，尚未
    正規化。

    gain_correction：長度 N_TONES 的陣列，依 bin 1..N_TONES 順序，每個
    諧波的振幅校正係數（見上方 GAIN_CORRECTION_CSV 說明）。None（預設）
    等同全部 1.0，完全不改變原本的波形——呼叫端要套用校正表需要自己呼叫
    load_gain_correction() 再傳進來（這個函式本身保持純函式，不做檔案
    I/O，方便既有的 FFT 驗證腳本直接呼叫）。

    exclude_bins：2026-08-19 新增，要完全排除（振幅設為 0）的諧波編號
    集合（1..N_TONES）。跟 gain_correction 疊加使用、互不影響——這是
    「host 端算好再上膛」版的 IFFT notch 排除頻率功能（取代原本
    FPGA 即時扣除的 notch_bank.v/notch_sine_gen.v 設計，見 PROJECT.md
    2026-08-19 對應章節：因為實際使用情境是「選好要排除的頻率、算好、
    上膛播放，之後偶爾才改」，不需要真正即時無縫切換，改成 host 端算
    好直接省掉大量 FPGA 硬體/BRAM 資源，且不受聲道數限制）。"""
    n = np.arange(N)
    bins = np.arange(1, N_TONES + 1)
    phis = -np.pi * bins * (bins - 1) / N_TONES
    w = 2 * np.pi * bins / N
    gain = np.ones(N_TONES) if gain_correction is None else np.asarray(gain_correction).copy()
    if exclude_bins:
        for k in exclude_bins:
            gain[k - 1] = 0.0

    x_full = np.zeros(N)
    for i in range(N_TONES):
        x_full += gain[i] * np.cos(w[i] * n + phis[i])

    return x_full, bins, w, phis, n


# 2026-08-20 新增：test1 面板 RF channel 的參數化 IFFT 梳狀波形（使用者
# 自訂 start/end/step，取代上面 precompute_full_sum() 寫死的 500Hz~
# 500kHz/500Hz 間距/1000 音）。拆成兩步（跟 precompute_full_sum() 是
# 一次函式不同）：compute_comb_bins() 只算「頻率結構」（bins/N，由
# start/end/step 決定），render_comb_waveform() 套用「這個 channel 自己
# 的 exclude 範圍」算出實際波形——這樣兩個 channel 都選 IFFT、start/
# end/step 相同但 exclude 範圍不同時，只需要呼叫一次 compute_comb_bins()
# 共用頻率結構，render_comb_waveform() 各自呼叫一次，不用重算兩次頻率
# 結構（見 PROJECT.md「test1 IFFT RF channel」章節，使用者確認 exclude
# 範圍不用跟另一個 channel 一致，只有 start/end/step 需要）。

def compute_comb_bins(start_hz, end_hz, step_hz, n=None, bin_stride=None):
    """算出 IFFT 梳狀波形的頻率結構：buffer 長度 n、實際頻率間距
    actual_step_hz、bins（實際會出現的諧波編號，w/phis 兩端點都是相對
    bins 陣列本身的第一個/最後一個位置算 Schroeder phase，不是絕對編號
    ——這樣不管 start_hz 選多少，PAPR 最小化的效果都一致，等同
    precompute_full_sum() 在 start_hz 剛好等於基頻時的特例）。

    n/bin_stride（2026-08-21 新增，兩者必須一起給或都不給）：給定時直接
    採用（不重新用 step_hz 反推 n），諧波間隔改成每隔 bin_stride 個頻率
    格點取一個——用於 board_ctrl.arm_test1_pattern() 的 IFFT+Single
    頻率混用（awg_common.wave_len_for_comb_plus_freq() 算出的 n 通常不
    是單純用 step_hz 反推的值，見該函式說明）。不給時（`n=None`，預設）
    維持原本行為：`n` 純粹由 step_hz 反推（四捨五入到 256 倍數，`bin_
    stride` 恆為 1），向下相容，雙 IFFT 混用（不牽涉 Single 頻率）跟
    `arm_multitone_pattern()` 的 `precompute_full_sum()` 路徑都不受影響。

    回傳 (n, actual_step_hz, bins, w, phis, bin_hz)——**`bin_hz`
    （=FS/n，2026-08-21 新增）是這次改動新增的回傳值**，`render_comb_
    waveform()` 的 exclude 頻率轉 bin 編號要用這個而不是 actual_step_hz
    （`bin_stride>1` 時兩者不相等）。"""
    if step_hz <= 0:
        raise ValueError("step_hz must be positive")
    if not (0 < start_hz <= end_hz):
        raise ValueError("start_hz must be positive and <= end_hz")
    if end_hz >= FS / 2:
        raise ValueError(f"end_hz must be below Nyquist ({FS / 2:.0f} Hz)")
    if (n is None) != (bin_stride is None):
        raise ValueError("n and bin_stride must be given together (or both omitted)")

    if n is None:
        n_raw = FS / step_hz
        n = max(BURST_SAMPLES, round(n_raw / BURST_SAMPLES) * BURST_SAMPLES)
        bin_stride = 1
    bin_hz = FS / n
    actual_step_hz = bin_stride * bin_hz

    start_bin = max(1, round(start_hz / bin_hz))
    end_bin = min(n // 2 - 1, round(end_hz / bin_hz))
    if end_bin < start_bin:
        raise ValueError("start_hz/end_hz/step_hz 算出來的頻率點範圍是空的，"
                          "請確認 step_hz 沒有大到讓 start_hz~end_hz 之間一個點都沒有")

    bins = np.arange(start_bin, end_bin + 1, bin_stride)
    m = len(bins)
    rel_idx = np.arange(1, m + 1)   # Schroeder phase 用相對位置，不是絕對 bin 編號
    phis = -np.pi * rel_idx * (rel_idx - 1) / m
    w = 2 * np.pi * bins / n

    return n, actual_step_hz, bins, w, phis, bin_hz


def render_comb_waveform(n, bins, w, phis, bin_hz,
                          exclude_start_hz=None, exclude_end_hz=None):
    """套用這個 channel 自己的 exclude 範圍（可以跟另一個 channel 不同，
    只有 compute_comb_bins() 算出的頻率結構要共用），算出實際波形
    x_full（值域約 -len(bins)..len(bins)，尚未正規化，用法跟
    precompute_full_sum() 的 x_full 一致）。exclude_start_hz/
    exclude_end_hz 任一個是 None 就不排除任何頻率。

    第 5 個參數 2026-08-21 從 `actual_step_hz` 改成 `bin_hz`（=FS/n）
    ——`compute_comb_bins()` 支援 `bin_stride>1` 後，`actual_step_hz`
    (=`bin_stride*bin_hz`) 不再等於單一頻率格點間距，exclude 頻率轉
    bin 編號一定要除以 `bin_hz` 才對得上 `bins` 陣列本身的編號系統，
    不然 notch 會挖到錯誤位置且不會有任何錯誤訊息（這是這次改動裡
    最容易漏掉、最難在示波器上察覺的一點，上機前先用頻譜回歸測試
    抓過一次，見 NOTES.md 對應章節）。

    內部改用 `np.fft.irfft()` 取代原本逐諧波疊加 cos() 的 O(m·n) 迴圈
    （2026-08-21，數學上完全等價，已用單元測試對照兩種算法逐點比對
    ——buffer 長度可能到百萬級、諧波數也可能到數千個時，O(n log n) 的
    irfft 比 O(m·n) 快非常多）。"""
    m = len(bins)
    gain = np.ones(m)
    if exclude_start_hz is not None and exclude_end_hz is not None:
        ex_start_bin = round(exclude_start_hz / bin_hz)
        ex_end_bin = round(exclude_end_hz / bin_hz)
        gain[(bins >= ex_start_bin) & (bins <= ex_end_bin)] = 0.0

    spectrum = np.zeros(n // 2 + 1, dtype=complex)
    spectrum[bins] = 0.5 * n * gain * np.exp(1j * phis)
    return np.fft.irfft(spectrum, n=n)
