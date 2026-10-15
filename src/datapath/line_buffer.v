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
    output wire        sdr_rd,    // 读请求（突发8，保持高直到 SDRAM 接受）
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

// ============ FIFO 端口线 ============
// ---- 写侧（50MHz SDRAM 域）----
wire        wrfull;    // 写满标志（未用；防溢出改用 wrusedw 判断）
wire [10:0] wrusedw;   // 写侧已用字数：判断 fifo_room，防止 FIFO 写溢出
// ---- 读侧（30MHz 显示域）----
wire [15:0] q;         // 读出的像素数据（16bit RGB565）
wire        rdempty;   // 读空标志（未用）
wire [10:0] rdusedw;   // 读侧已用字数：判断整行 720 字是否就绪

// ============ 双时钟 FIFO（IP 核，深度 2048 × 16bit）============
// 写侧 50MHz、读侧 30MHz，实现跨时钟域隔离（SDRAM 域 → 显示域）
FIFO u_fifo (
    // ---- 写侧（50MHz SDRAM 域）----
    .data    (sdr_rdata),   // 写数据：SDRAM 读出的字（16bit）
    .wrclk   (clk_50m),     // 写时钟：50MHz
    .wrreq   (sdr_rvalid),  // 写请求：SDRAM 读有效即写入
    .wrfull  (wrfull),      // 写满标志（深度 2048 满）
    .wrusedw (wrusedw),     // 写侧已用字数（11bit，最大 2048）
    .wrempty (),            // 写空标志：未连接
    // ---- 读侧（30MHz 显示域）----
    .rdclk   (clk_30m),     // 读时钟：30MHz
    .rdreq   (pop),         // 读请求：显示侧弹出像素
    .q       (q),           // 读出的数据（rdreq 后 1 拍生效，注意对齐）
    .rdempty (rdempty),     // 读空标志（未用）
    .rdfull  (),            // 读满标志：未连接
    .rdusedw (rdusedw)      // 读侧已用字数（11bit，判断 rdusedw>=720 行就绪）
);

// ============ 50MHz 写侧（SDRAM 读控制器）============
reg [9:0]  write_cnt;    // 当前行已收字数 0~719
reg [20:0] line_base;    // 当前行在 SDRAM 的起始地址
reg        rd_active;    // 读请求进行中（已发，等收满 8 字）
reg [2:0]  rvalid_cnt;   // 当前突发已收字数 0~7

// FIFO 留有 16 字余量时才发下一组突发（防溢出）
wire fifo_room = (wrusedw <= 11'd2032);

// sdr_rd 组合输出：读请求进行中且 SDRAM 空闲时保持高
// 【修复】原设计 sdr_rd 为 1 拍脉冲，若 SDRAM 控制器恰有刷新请求(ref_req)，
// 会优先刷新而漏掉读请求 → burst_cnt 多计、write_cnt 少收 → 写侧卡死、
// FIFO 攒不满一行 → valid 恒 0 → 黑屏。改为保持高直到控制器接受。
assign sdr_rd = rd_active && !sdr_busy;

always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        write_cnt  <= 10'd0;
        line_base  <= 21'd0;
        sdr_addr   <= 21'd0;
        rd_active  <= 1'b0;
        rvalid_cnt <= 3'd0;
    end
    else begin
        if (en) begin
            // 发突发读：SDRAM 空闲、FIFO 有空间、本行未读完、无进行中的读
            if (!sdr_busy && fifo_room && write_cnt < LINE_WORDS && !rd_active) begin
                sdr_addr   <= line_base + write_cnt;  // write_cnt 是 8 的倍数
                rd_active  <= 1'b1;
                rvalid_cnt <= 3'd0;
            end

            // 收数据计数（数据由 FIFO 的 wrreq=sdr_rvalid 自动写入）
            if (sdr_rvalid) begin
                write_cnt <= write_cnt + 10'd1;
                if (rvalid_cnt == 3'd7) begin
                    // 收满 8 字，本突发完成
                    rd_active  <= 1'b0;
                    rvalid_cnt <= 3'd0;
                end
                else
                    rvalid_cnt <= rvalid_cnt + 3'd1;

                if (write_cnt == LINE_WORDS - 1) begin
                    // 一行 720 字收满，换下一行
                    write_cnt <= 10'd0;
                    // 480 行后回绕到 0（一帧 480 行）
                    if (line_base == 21'd479 * LINE_WORDS)
                        line_base <= 21'd0;
                    else
                        line_base <= line_base + LINE_WORDS;
                end
            end
        end
        else begin
            rd_active <= 1'b0;
        end
    end
end

// ============ 30MHz 读侧（行就绪控制 + 帧同步）============
reg        line_active;   // 当前行数据就绪，正在弹出
reg [9:0]  pop_cnt;       // 当前行已弹出字数 0~719
reg        frame_synced;  // 帧同步标志：已对齐到显示侧的 pixel_y=0

always @(posedge clk_30m or negedge rst_n) begin
    if (!rst_n) begin
        line_active  <= 1'b0;
        pop_cnt      <= 10'd0;
        frame_synced <= 1'b0;
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
            if (frame_synced) begin
                // 帧同步后：FIFO 攒满一整行就弹（不依赖行起始脉冲，避免跳行滚动）
                if (rdusedw >= 11'd720)
                    line_active <= 1'b1;
            end
            else begin
                // 帧同步前：等帧起始（pixel_y=0 的行开始）且 FIFO 已攒满一整行，
                // 保证图片第 0 行正好显示在屏幕第一行，解决「第一行不在第一行」。
                if (line_start && rdusedw >= 11'd720) begin
                    frame_synced <= 1'b1;
                    line_active  <= 1'b1;
                end
            end
        end
    end
end

// ============ 输出 ============
assign pixel = q;
assign valid = line_active;

endmodule
