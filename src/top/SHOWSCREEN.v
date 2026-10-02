// ============================================================
// 顶层模块：SHOWSCREEN
// 器件：EP4CE6F17C8 (Cyclone IV E)
// 功能：PLL(50MHz→30MHz) + lcd_driver + displayer
//       驱动 ATK-MD0430R-800480 屏幕显示测试图案
// ============================================================
`timescale 1ns / 1ps

module SHOWSCREEN (
    // ---- 输入 ----
    input  wire        sys_clk,     // 板载 50MHz 时钟
    input  wire        sys_rst_n,   // 复位（低有效）

    // ---- 输出给屏幕 ----
    output wire        lcd_clk,     // 像素时钟 30MHz
    output wire        lcd_de,      // 数据使能
    output wire        lcd_hs,      // 水平同步
    output wire        lcd_vs,      // 垂直同步
    output wire        lcd_bl,      // 背光控制
    output wire        lcd_rst,     // LCD 复位
    output wire [15:0] lcd_rgb      // RGB565 16 位
);

// ---- 内部信号 ----
wire        clk_30m;      // PLL 输出的 30MHz 像素时钟
wire        pll_locked;   // PLL 锁定指示
wire        rst_n_int;    // 内部复位：外部复位释放 且 PLL 锁定后才释放
wire [9:0]  pixel_x;      // 当前像素横坐标
wire [8:0]  pixel_y;      // 当前像素纵坐标
wire        pixel_valid;  // 有效区指示
wire [23:0] pixel_data;   // 像素数据 RGB888

// PLL 锁定 + 外部复位：两者都满足才释放内部复位
assign rst_n_int = sys_rst_n & pll_locked;

// ---- PLL：50MHz → 30MHz ----
pll30Mhz u_pll (
    .areset (~sys_rst_n), // areset 高有效，接反相后的复位
    .inclk0 (sys_clk),    // 50MHz 输入
    .c0     (clk_30m),    // 30MHz 输出
    .locked (pll_locked)  // 锁定指示
);

// ---- LCD 驱动模块 ----
lcd_driver u_lcd_driver (
    .clk         (clk_30m),
    .rst_n       (rst_n_int),
    .pixel_data  (pixel_data),
    .lcd_clk     (lcd_clk),
    .lcd_de      (lcd_de),
    .lcd_hs      (lcd_hs),
    .lcd_vs      (lcd_vs),
    .lcd_bl      (lcd_bl),
    .lcd_rst     (lcd_rst),
    .lcd_rgb     (lcd_rgb),
    .pixel_x     (pixel_x),
    .pixel_y     (pixel_y),
    .pixel_valid (pixel_valid)
);

// ---- 图像显示模块 ----
displayer u_displayer (
    .pixel_valid (pixel_valid),
    .pixel_x     (pixel_x),
    .pixel_y     (pixel_y),
    .pixel_data  (pixel_data)
);

endmodule
