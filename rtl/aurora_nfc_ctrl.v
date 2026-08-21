`timescale 1ns/1ps
`default_nettype none

// aurora_nfc_ctrl.v -- 2026-07-30 新增
//
// 背景：`T_WAVEFORM_STREAM` 大封包資料錯位 bug 的真正根因（見 NOTES.md
// 2026-07-30「Opus agent 查出真正根因」章節）：`async_fifo_aurora_rx`
// （本板 Aurora RX 本地送達的 256 深緩衝）跟 `relay_fifo_0`（本板轉送
// 用的 256 深緩衝）的寫入側都完全沒有 flow control，一旦本板下游處理
// 跟不上（`ddr_writer.v` 的 `pacc_fifo` 滿）造成這兩顆 FIFO 也跟著滿，
// 之後進來的 Aurora beat 會被無聲丟棄。Aurora RX（Framing mode）本身
// 沒有 tready，唯一能讓對面真正「暫停送資料」的機制是 Aurora 64B/66B
// 核心內建的 Native Flow Control（NFC，`aurora_64b66b_0` 的
// `flow_mode` 改成 `Immediate_NFC` 後才會出現 `s_axi_nfc_*` 這組
// port，見 create_bd.tcl 對應章節）。
//
// 方向確認（不用兩邊都送，只送一邊）：本板的 Layer2 資料轉送
// （`aurora_data_channel_0`，`aurora_data_channel_relayfifo.v`）固定
// 只吃 `rx0`、只送 `tx1`（見該檔案開頭註解「Incoming relay source
// (fixed direction: rx0 = aurora_64b66b_0/SFP1)」），`rx1` 只接
// Layer3 `aurora_ctrl_channel.v` 的低流量控制封包，不會塞爆這兩顆
// FIFO。也就是說 `async_fifo_aurora_rx`/`relay_fifo_0` 會滿，一定是
// `rx0` 這個方向進來的資料造成的，只需要對 `aurora_64b66b_0` 送 NFC
// 暫停要求，不用碰 `aurora_64b66b_1`。
//
// NFC 訊息格式跟時序（直接讀 Xilinx IP 產生出來的實際 RTL 原始碼
// `..._tx_ll_control_sm.v`/`..._rx_ll_nfc.v` 確認，不是查文件用猜的）：
//   - `s_axi_nfc_tdata[8:15]`：pause count（對面收到後設進
//     `nfc_counter_r`，每次對面「原本要送資料卻被擋下來」就減 1，
//     減完歸零才恢復送資料——count 單位是「對面被擋下的資料拍數」，
//     不是固定時間）
//   - `s_axi_nfc_tdata[7]`（XOFF flag）：這次刻意**不使用**——實際
//     追查 `RX_LL_NFC` 的邏輯，XOFF 那條路徑在 `nfc_counter_r` 已經
//     是 0 的情況下，幾乎在同一拍就會被 `DECREMENT_NFC` 條件連動清掉
//     `TX_WAIT`，不是穩定的「無限期暫停，等明確 XON」語意（至少這個
//     IP 版本看起來是這樣，信心不到 100%，為了不賭這個不確定的行為，
//     這裡改用下面的作法）
//   - 改用「重送 count=8'hFF（最大值）的一般 NFC 訊息」：只要
//     `congested` 還是 1，這個模組就會送一次 count=0xFF 的請求，把
//     對面的暫停額度頂到滿；`congested` 一旦變 0 就停止重送，對面的
//     暫停額度會在最多 255 拍內自然歸零、恢復送資料——不依賴任何
//     「明確 XON」訊息，行為完全由已驗證過的 count 遞減機制決定，比賭
//     XOFF 語意更穩妥。
//
// 2026-07-30 review 修正（Opus agent 找到的問題，見 NOTES.md 同日
// 章節）：原本設計是「每次被 ack 就立刻送下一次」，等於每個 cycle 都
// 在送新請求——一次 count=0xFF 就能頂住對面 255 拍，每拍重送是完全
// 不必要的 255 倍頻率，而且 Aurora 的 `tx_ll_control_sm` 會優先插入
// NFC/CC 這類控制符號、犧牲使用者資料傳輸權（見該模組註解："data...
// can use the channel when there is no CC, NFC message..."）——連續
// 佔用控制符號插入權，會排擠掉本板 `aurora_64b66b_0` 同一個 TX 上
// 的 Layer3 控制封包（`aurora_ctrl_channel_0/ctrl_tx0_*`，enum/
// trigger/reserve 協定都走這裡），有卡住協定的風險。改成請求 ack 後
// 進入一段冷卻期（`COOLDOWN_CYCLES`，遠小於 255 拍額度，留足安全
// 餘裕給 NFC 生效的來回延遲），冷卻期間完全不佔用 TX，冷卻結束後
// `congested` 還在才送下一次。

module aurora_nfc_ctrl (
    input  wire        aurora_clk,
    input  wire        rst,

    // 本板本地送達/轉送緩衝任一個接近滿，就要求對面（rx0 方向的鄰居）
    // 暫停送資料
    input  wire        congested,

    // 接 aurora_64b66b_0 的 s_axi_nfc_*（flow_mode=Immediate_NFC 才有
    // 這組 port，見 create_bd.tcl）
    output reg         nfc_tvalid,
    output wire [15:0] nfc_tdata,
    input  wire        nfc_tready
);

    localparam [7:0] PAUSE_COUNT     = 8'hFF;   // 對面每次收到請求可以撐的最大拍數
    localparam [7:0] COOLDOWN_CYCLES = 8'd160;  // 遠小於 255，留安全餘裕才重送

    // tdata[7]=0（不用 XOFF，見上方說明）、tdata[8:15]=8'hFF（最大
    // pause count）、其餘 bit 這個 IP 版本沒用到，固定 0
    assign nfc_tdata = {8'h00, PAUSE_COUNT};

    reg [7:0] cooldown;

    always @(posedge aurora_clk) begin
        if (rst) begin
            nfc_tvalid <= 1'b0;
            cooldown   <= 8'd0;
        end else if (nfc_tvalid && nfc_tready) begin
            // 這次請求被對面 ack 了：先進冷卻期，不要每拍都重送
            nfc_tvalid <= 1'b0;
            cooldown   <= COOLDOWN_CYCLES;
        end else if (!nfc_tvalid) begin
            if (cooldown != 8'd0) begin
                cooldown <= cooldown - 8'd1;
            end else if (congested) begin
                nfc_tvalid <= 1'b1;
            end
        end
        // nfc_tvalid==1 且 nfc_tready==0（對面還沒 ack）：不做任何事，
        // nfc_tvalid 維持 1，不受 congested 影響——AXI4-Stream「valid
        // 不能在 handshake 完成前撤回」的規則在這裡成立
    end

endmodule
`default_nettype wire
