// ============================================================
// 模块：displayer —— 图像显示模块
// 功能：产生要显示的图像内容（像素数据）
// ============================================================
`timescale 1ns / 1ps

module displayer (
    // ---- 输入 ----
    input  wire        pixel_valid,  // 有效区指示
    input  wire [9:0]  pixel_x,      // 当前像素横坐标 0~799
    input  wire [8:0]  pixel_y,      // 当前像素纵坐标 0~479

    // ---- 输出 ----
    output reg  [23:0] pixel_data    // 传给 lcd_driver 的像素 RGB888
);

// x+y 的和（扩成 11 位，防止 10 位相加溢出）
wire [10:0] xy_sum = {1'b0, pixel_x} + {2'b0, pixel_y};

// ---- 组合逻辑：根据坐标产生颜色 ----
always @(*) begin
    if (!pixel_valid)
        pixel_data = 24'h000000;   // 无效区 → 纯黑
    else if (xy_sum > 200)
        pixel_data = 24'h00FF00;   // 绿
    else if (xy_sum < 200)
        pixel_data = 24'h0000FF;   // 蓝
    else
        pixel_data = 24'hFF0000;   // ==200 → 红
end

endmodule
