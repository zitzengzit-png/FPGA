// ============================================================
// 模块：displayer —— 图像显示模块
// 功能：从 line_buffer（FIFO）读 720x480 图像并输出
// ============================================================
`timescale 1ns / 1ps

module displayer (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        pixel_valid,
    input  wire [9:0]  pixel_x,
    input  wire [8:0]  pixel_y,
    output wire [15:0] pixel_data,   // RGB565

    // ---- line_buffer 接口 ----
    input  wire [15:0] pixel,        // 像素数据
    input  wire        valid,        // 数据有效
    output wire        pop,          // 弹出
    output wire        line_start,   // 行起始（图片区前 1 拍，仅帧首行）
    output wire        row_start     // 图像区每一行的行首前 1 拍（行锁定用）
);

localparam IMG_W  = 720;
localparam IMG_H  = 480;
localparam IMG_X0 = (800 - IMG_W) / 2;   // 40，水平居中

// 图片区（720x480）
wire in_image = pixel_valid &&
                (pixel_x >= IMG_X0) && (pixel_x < IMG_X0 + IMG_W) &&
                (pixel_y < IMG_H);

// 帧起始脉冲：第一行（pixel_y==0）图片区前 1 拍，用于 line_buffer 帧同步
// 保证图片第 0 行正好显示在屏幕第一行，避免读阶段开始时 pixel_y 已跑偏导致错位
assign line_start = pixel_valid && (pixel_x == (IMG_X0 - 1)) && (pixel_y == 9'd0);

// 图像区每行的行首前 1 拍脉冲（x = IMG_X0-1）：line_buffer 只用它在行首开行，
// 保证「一行 = 恰好 720 个字」且一定从本行第一个像素开始，
// 绝不会因为 FIFO 中途才攒够而"半路开行"造成少弹/串到下一行。
assign row_start  = pixel_valid && (pixel_x == (IMG_X0 - 1)) && (pixel_y < IMG_H);

// 图片区内且数据有效才弹出（FIFO 的 rdreq）
assign pop = in_image & valid;

// 图片区内且数据有效才输出像素，否则黑边
// 【修复】FIFO 的 q 比 pop 晚 1 拍（rdreq 后 1 拍数据才出现在 q 上），
//        输出的选通信号必须用「同一拍」的 in_image/valid 才有对齐：
//        某一拍显示的像素 = 1 拍前 q 上的字 = 2 拍前的 pop 读出的字，
//        所以这里把 in_image/valid 也延迟 1 拍（= 对齐到 2 拍前，即 pop 那一拍）。
//        原代码用当前拍的 in_image，一行最后一个像素（屏幕 x=759）会被判成
//        「非图片区」而输出黑点 —— 表现为图像右边多一条 1 像素宽的竖直黑线。
reg in_image_d1;
reg valid_d1;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        in_image_d1 <= 1'b0;
        valid_d1    <= 1'b0;
    end
    else begin
        in_image_d1 <= in_image;
        valid_d1    <= valid;
    end
end

assign pixel_data = (in_image_d1 && valid_d1) ? pixel : 16'h0000;

endmodule
