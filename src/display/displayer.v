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
    output wire        line_start    // 行起始（图片区前 1 拍）
);

localparam IMG_W  = 720;
localparam IMG_H  = 480;
localparam IMG_X0 = (800 - IMG_W) / 2;   // 40，水平居中

// 图片区（720x480）
wire in_image = pixel_valid &&
                (pixel_x >= IMG_X0) && (pixel_x < IMG_X0 + IMG_W) &&
                (pixel_y < IMG_H);

// 行起始脉冲：图片区第 1 个像素的前 1 拍（pixel_x == IMG_X0-1）
// 提前 1 拍发出，让 line_buffer 的行就绪标志在图片区首拍就生效
assign line_start = pixel_valid && (pixel_x == (IMG_X0 - 1)) && (pixel_y < IMG_H);

// 图片区内且数据有效才弹出（FIFO 的 rdreq）
assign pop = in_image & valid;

// 图片区内且数据有效才输出像素，否则黑边
assign pixel_data = (in_image && valid) ? pixel : 16'h0000;

endmodule
