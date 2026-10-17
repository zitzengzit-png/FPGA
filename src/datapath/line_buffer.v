// ============================================================
// line_buffer —— FIFO(17位) 行缓冲 + 行尾标记对齐
//   bit15:0 = 像素；bit16 = 本行最后一个字（行尾标记）
//   读侧启动时先丢弃到标记之后，再从标记后的第一个字开始一行
//   ⇒ “本行第一个像素”由标记确定，不再依赖计数相位（根治整行错位）
// ============================================================
`timescale 1ns / 1ps

module line_buffer (
    input  wire        clk_50m,
    input  wire        clk_30m,
    input  wire        rst_n,
    output reg  [20:0] sdr_addr,
    output wire        sdr_rd,
    input  wire [15:0] sdr_rdata,
    input  wire        sdr_rvalid,
    input  wire        sdr_busy,
    input  wire        en,
    input  wire        pop,
    input  wire        row_start,
    input  wire        line_start,
    output wire [15:0] pixel,
    output wire        valid
);

localparam LINE_WORDS = 720;
localparam SYNC_WM    = 11'd1536;
localparam LINE_WM    = 11'd720;

wire        wrfull;
wire [10:0] wrusedw;
wire [16:0] fifo_q;
wire [15:0] q    = fifo_q[15:0];
wire        mark = fifo_q[16];
wire        rdempty;
wire [10:0] rdusedw;

reg         clearing;
reg  [3:0]  clr_wait;
reg  [11:0] clr_att;
reg         clr_pop;
localparam CLR_WAIT = 4'd8;
localparam CLR_MAX  = 12'd2600;

reg         align_wait;
reg         align_ok;
// 对齐丢弃：一直读到"帧首标记出现在 FIFO 头部"；此时标记本身尚未弹掉，
// 它就是图像 (行0,列0) 那个字 ⇒ 直接以它为行首（不再多弹一拍）
wire        align_pop = align_wait && (rdusedw > 11'd4) && !mark;

reg [9:0]  write_cnt;
reg [20:0] line_base;
reg        rd_active;
reg [2:0]  rvalid_cnt;

wire fifo_room = (wrusedw <= 11'd2032);
assign sdr_rd = rd_active && !sdr_busy;
// 帧首标记：只在「图像第 0 行、第 0 列」那个字上置 1（line_base==0 且 write_cnt==0）
// 读侧对齐时丢弃到该标记为止，头部正好是图像 (行0,列0) ⇒ 不会整体下移一行
wire        row_end_w = (line_base == 21'd0) && (write_cnt == 10'd0);

// 官方 IP（MegaWizard 生成，17 位 = {bit16 行尾标记, 16 位像素}，已启用 aclr）
//   文件：ipcores/ffifowith17/ffifo.v，模块名 ffifo
//   参数：17bit x 2048、showahead=OFF、同步级数 4、read/write_aclr_synch=ON、dc_aclr=1
//   端口：aclr/data/rdclk/rdreq/wrclk/wrreq/q/rdempty/rdfull/rdusedw/wrempty/wrfull/wrusedw
ffifo u_fifo (
    .aclr    (~rst_n),
    .data    ({row_end_w, sdr_rdata}),
    .wrclk   (clk_50m),
    .wrreq   (sdr_rvalid),
    .wrfull  (wrfull),
    .wrusedw (wrusedw),
    .wrempty (),
    .rdclk   (clk_30m),
    .rdreq   (pop || clr_pop || align_pop),
    .q       (fifo_q),
    .rdempty (rdempty),
    .rdfull  (),
    .rdusedw (rdusedw)
);


// ============ 50MHz 写侧：SDRAM 突发8读 + 行计数 ============
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        write_cnt  <= 10'd0;
        line_base  <= 21'd0;
        sdr_addr   <= 21'd0;
        rd_active  <= 1'b0;
        rvalid_cnt <= 3'd0;
    end
    else if (en) begin
        if (!sdr_busy && fifo_room && write_cnt < LINE_WORDS && !rd_active) begin
            sdr_addr   <= line_base + write_cnt;
            rd_active  <= 1'b1;
            rvalid_cnt <= 3'd0;
        end
        if (sdr_rvalid) begin
            write_cnt <= write_cnt + 10'd1;
            if (rvalid_cnt == 3'd7) begin
                rd_active  <= 1'b0;
                rvalid_cnt <= 3'd0;
            end
            else
                rvalid_cnt <= rvalid_cnt + 3'd1;

            if (write_cnt == LINE_WORDS - 1) begin
                write_cnt <= 10'd0;
                if (line_base == 21'd479 * LINE_WORDS)
                    line_base <= 21'd0;
                else
                    line_base <= line_base + LINE_WORDS;
            end
        end
    end
    else
        rd_active <= 1'b0;
end

// ============ 30MHz 读侧(1)：复位后精确读空（aclr 已清指针，这里只是保险）============
always @(posedge clk_30m or negedge rst_n) begin
    if (!rst_n) begin
        clearing <= 1'b1;
        clr_wait <= 4'd0;
        clr_att  <= 12'd0;
        clr_pop  <= 1'b0;
    end
    else if (clearing) begin
        clr_pop <= 1'b0;
        if (clr_wait == CLR_WAIT - 4'd1) begin
            clr_wait <= 4'd0;
            if ((rdusedw == 11'd0) || (clr_att >= CLR_MAX)) begin
                clearing <= 1'b0;
                clr_att  <= 12'd0;
            end
            else begin
                clr_pop <= 1'b1;
                clr_att <= clr_att + 12'd1;
            end
        end
        else
            clr_wait <= clr_wait + 4'd1;
    end
    else
        clr_pop <= 1'b0;
end

// ============ 30MHz 读侧(2)：标记对齐 + 行控制 ============
reg [9:0] pop_cnt;
reg       line_active;
reg       frame_synced;

always @(posedge clk_30m or negedge rst_n) begin
    if (!rst_n) begin
        line_active  <= 1'b0;
        pop_cnt      <= 10'd0;
        frame_synced <= 1'b0;
        align_wait   <= 1'b0;
        align_ok     <= 1'b0;
    end
    else if (align_wait) begin
        // ① 丢弃到「帧首标记」：mark=1 时头部就是图像 (行0,列0)，直接对齐完成
        if (mark) begin
            align_wait <= 1'b0;
            align_ok   <= 1'b1;
        end
    end
    else if (line_active) begin
        if (pop) begin
            if (pop_cnt == LINE_WORDS - 1) begin
                pop_cnt     <= 10'd0;
                line_active <= 1'b0;
            end
            else
                pop_cnt <= pop_cnt + 10'd1;
        end
    end
    else if (!frame_synced) begin
        if (align_ok) begin
            // 对齐已完成：等帧首，且 FIFO 里又攒够一整行，才开显
            if (line_start && !clearing && (rdusedw >= LINE_WM)) begin
                frame_synced <= 1'b1;
                line_active  <= 1'b1;
            end
        end
        else if (!clearing && (rdusedw >= SYNC_WM)) begin
            align_wait <= 1'b1;          // 攒够 → 开始丢弃找行尾标记
        end
    end
    else begin
        if (!clearing && (rdusedw >= LINE_WM))
            line_active <= 1'b1;
    end
end

assign pixel = q;
assign valid = line_active;

endmodule
