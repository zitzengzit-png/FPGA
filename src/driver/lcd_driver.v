// ============================================================
// 模块：lcd_driver —— LCD 屏幕驱动模块
// 目标屏：ATK-MD0430R-800480（800x480，DE 模式，30MHz 像素时钟）
// 时序：H_total=928 H_active=800；V_total=525 V_active=480
// ============================================================
`timescale 1ns / 1ps

module lcd_driver (
    // ---- 输入 ----
    input  wire        clk,          // 30MHz 像素时钟（PLL 提供）
    input  wire        rst_n,        // 异步复位，低有效
    input  wire [15:0] pixel_data,   // displayer 送来的像素 RGB565

    // ---- 输出给屏幕 ----
    output wire        lcd_clk,      // 像素时钟（直通 clk）
    output reg         lcd_de,       // 数据使能 DE
    output reg         lcd_hs,       // 水平同步 HSYNC
    output reg         lcd_vs,       // 垂直同步 VSYNC
    output reg         lcd_bl,       // 背光控制
    output reg         lcd_rst,      // LCD 复位
    output reg  [15:0] lcd_rgb,      // RGB565 16 位（R[4:0] G[5:0] B[4:0]）

    // ---- 输出给显示模块（组合输出，避免引入额外一拍延迟）----
    output wire [9:0]  pixel_x,      // 当前像素横坐标 0~799
    output wire [8:0]  pixel_y,      // 当前像素纵坐标 0~479
    output wire        pixel_valid   // 有效区指示（控制 displayer 是否发数据）
);

// ============ 时序参数（按手册典型值）============
localparam H_SYNC   = 40;    // 水平同步脉宽
localparam H_BP     = 40;    // 水平后肩
localparam H_ACTIVE = 800;   // 水平有效区
localparam H_FP     = 48;    // 水平前肩
localparam H_TOTAL  = H_SYNC + H_BP + H_ACTIVE + H_FP;  // 928

localparam V_SYNC   = 2;     // 垂直同步脉宽（原 0，面板需非零 VSYNC 才能同步）
localparam V_BP     = 30;    // 垂直后肩
localparam V_ACTIVE = 480;   // 垂直有效区
localparam V_FP     = 13;    // 垂直前肩
localparam V_TOTAL  = V_SYNC + V_BP + V_ACTIVE + V_FP;  // 525

localparam H_START  = H_SYNC + H_BP;   // 有效区起始 80
localparam V_START  = V_SYNC + V_BP;   // 有效区起始 32

localparam DELAY_1MS = 30000;          // 1ms 上电延时 = 30MHz × 1ms = 30000 个时钟

// 像素时钟直通输出（不经过寄存器，避免时钟延迟/歪斜）
assign lcd_clk = clk;

// ============ 计数器：当前行(h_cnt)/当前列(v_cnt) ============
reg [9:0] h_cnt;   // 当前行 = 水平计数器 0~927
reg [9:0] v_cnt;   // 当前列 = 垂直计数器 0~524
reg        flag;       // 1ms 上电延时完成标志：0=延时中，1=正常计数
reg [14:0] delay_cnt;  // 1ms 延时计数器

// ---- 时序块 1：计数（含 1ms 上电延时）----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        flag      <= 1'b0;       // 复位后先进入延时阶段
        delay_cnt <= 15'd0;
        h_cnt     <= 10'd0;
        v_cnt     <= 10'd0;
    end
    else begin
        if (!flag) begin         // 延时阶段：只计 1ms，冻结 h_cnt/v_cnt
            if (delay_cnt == DELAY_1MS - 1) begin
                flag      <= 1'b1;      // 满 1ms，进入正常计数
                delay_cnt <= 15'd0;
            end
            else
                delay_cnt <= delay_cnt + 15'd1;
        end
        else begin               // 正常阶段：原来的计数逻辑
            if (h_cnt == H_TOTAL - 1) begin        // 行到行周期归 0，列+1
                h_cnt <= 10'd0;
                if (v_cnt == V_TOTAL - 1)         // 列到列周期归 0
                    v_cnt <= 10'd0;
                else
                    v_cnt <= v_cnt + 10'd1;
            end
            else
                h_cnt <= h_cnt + 10'd1;
        end
    end
end

// ============ 组合逻辑：有效区标志 + 有效坐标 ============
wire       active = (h_cnt >= H_START) && (h_cnt < H_START + H_ACTIVE) &&
                    (v_cnt >= V_START) && (v_cnt < V_START + V_ACTIVE);
wire [9:0] x_comb = active ? (h_cnt - H_START) : 10'd0;  // 有效区内 0~799
wire [8:0] y_comb = active ? (v_cnt - V_START) : 9'd0;   // 有效区内 0~479

assign pixel_x     = x_comb;
assign pixel_y     = y_comb;
assign pixel_valid = active;

// active 延迟 1 拍（对齐 line_buffer 的注册读延迟）
reg active_d1;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        active_d1 <= 1'b0;
    else
        active_d1 <= active;
end

// ============ 时序块 2：输出（统一寄存，保证 DE 与 RGB 对齐）============
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        lcd_de  <= 1'b0;
        lcd_hs  <= 1'b0;
        lcd_vs  <= 1'b0;
        lcd_bl  <= 1'b0;
        lcd_rst <= 1'b0;
        lcd_rgb <= 16'd0;
    end
    else begin
        lcd_hs  <= 1'b1;               // DE 模式：HSYNC 拉高
        lcd_vs  <= 1'b1;               // DE 模式：VSYNC 拉高
        lcd_bl  <= 1'b1;               // 背光常开
        lcd_rst <= flag;               // 1ms 延时期间拉低复位，之后释放

        if (active_d1) begin           // 有效区（延迟1拍对齐line_buffer）：输出，拉高 DE
            lcd_de  <= 1'b1;
            lcd_rgb <= pixel_data;     // 已是 RGB565，直接输出
        end
        else begin                     // 消隐区：DE 拉低，输出 0
            lcd_de  <= 1'b0;
            lcd_rgb <= 16'd0;
        end
    end
end

endmodule
