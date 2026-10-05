// ============================================================
// 模块：line_buffer —— 行缓冲（FIFO 跨时钟域）
// 功能：50MHz 域从 SDRAM 突发8 读一行(720字)，写入双时钟 FIFO；
//       30MHz 域从 FIFO 流式读出给显示。
// 使用 IP：ipcores/FIFO30M50M/FIFO（dcfifo，深度 2048，16bit）
// ============================================================
`timescale 1ns / 1ps

module line_buffer (
    input  wire        clk_50m,   // SDRAM 读时钟 50MHz
    input  wire        clk_30m,   // 显示时钟 30MHz
    input  wire        rst_n,     // 复位（低有效）

    // ---- SDRAM 读接口（50MHz）----
    output reg  [20:0] sdr_addr,  // 读地址
    output reg         sdr_rd,    // 读请求（突发8）
    input  wire [15:0] sdr_rdata, // 读数据
    input  wire        sdr_rvalid,// 读有效
    input  wire        sdr_busy,  // SDRAM 忙
    input  wire        en,        // 读使能（写阶段拉低，防止读控制器卡死）

    // ---- 显示接口（30MHz）----
    input  wire        pop,       // 弹出（图片区且缓冲有数据）
    input  wire        line_start,// 行起始脉冲（图片区前 1 拍）
    output wire [15:0] pixel,     // 像素数据
    output wire        valid      // 数据有效（当前行已就绪）
);

localparam LINE_WORDS = 720;   // 每行像素数
localparam BURSTS     = 90;    // 每行突发次数 = 720/8

// ============ FIFO 端口线 ============
wire        wrfull;
wire [10:0] wrusedw;
wire [15:0] q;
wire        rdempty;
wire [10:0] rdusedw;

// ============ 双时钟 FIFO（IP 核，深度 2048 × 16bit）============
FIFO u_fifo (
    .data    (sdr_rdata),   // 写数据：SDRAM 读出的字
    .wrclk   (clk_50m),
    .wrreq   (sdr_rvalid),  // SDRAM 读有效即写入
    .wrfull  (wrfull),
    .wrusedw (wrusedw),
    .wrempty (),
    .rdclk   (clk_30m),
    .rdreq   (pop),         // 读请求：显示弹出
    .q       (q),
    .rdempty (rdempty),
    .rdfull  (),
    .rdusedw (rdusedw)
);

// ============ 50MHz 写侧（SDRAM 读控制器）============
reg [6:0]  burst_cnt;    // 当前行突发计数 0~89
reg [9:0]  write_cnt;    // 当前行已收字数 0~719
reg [20:0] line_base;    // 当前行在 SDRAM 的起始地址

// FIFO 留有 16 字余量时才发下一组突发（防溢出）
wire fifo_room = (wrusedw <= 11'd2032);

always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        burst_cnt <= 7'd0;
        write_cnt <= 10'd0;
        line_base <= 21'd0;
        sdr_addr  <= 21'd0;
        sdr_rd    <= 1'b0;
    end
    else begin
        sdr_rd <= 1'b0;

        if (en) begin
            // 发突发读：SDRAM 空闲、FIFO 有空间、本行未读完
            if (!sdr_busy && fifo_room && burst_cnt < BURSTS) begin
                sdr_addr  <= line_base + (burst_cnt << 3);   // 8 字对齐
                sdr_rd    <= 1'b1;
                burst_cnt <= burst_cnt + 7'd1;
            end

            // 收数据计数（数据由 FIFO 的 wrreq=sdr_rvalid 自动写入）
            if (sdr_rvalid) begin
                write_cnt <= write_cnt + 10'd1;
                if (write_cnt == LINE_WORDS - 1) begin
                    // 一行 720 字收满，换下一行
                    write_cnt <= 10'd0;
                    burst_cnt <= 7'd0;
                    // 480 行后回绕到 0（一帧 480 行）
                    if (line_base == 21'd479 * LINE_WORDS)
                        line_base <= 21'd0;
                    else
                        line_base <= line_base + LINE_WORDS;
                end
            end
        end
    end
end

// ============ 30MHz 读侧（行就绪控制）============
reg        line_active;   // 当前行数据就绪，正在弹出
reg [9:0]  pop_cnt;       // 当前行已弹出字数 0~719

always @(posedge clk_30m or negedge rst_n) begin
    if (!rst_n) begin
        line_active <= 1'b0;
        pop_cnt     <= 10'd0;
    end
    else begin
        if (line_active) begin
            // 正在弹出一行
            if (pop) begin
                if (pop_cnt == LINE_WORDS - 1) begin
                    pop_cnt     <= 10'd0;
                    line_active <= 1'b0;   // 一行读完
                end
                else
                    pop_cnt <= pop_cnt + 10'd1;
            end
        end
        else begin
            // 等待：行起始且 FIFO 已有一整行（720 字）
            if (line_start && rdusedw >= 11'd720)
                line_active <= 1'b1;
        end
    end
end

// ============ 输出 ============
assign pixel = q;
assign valid = line_active;

endmodule
