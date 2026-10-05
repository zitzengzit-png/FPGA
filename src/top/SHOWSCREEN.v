// ============================================================
// 顶层模块：SHOWSCREEN
// 器件：EP4CE10F17C8 (Cyclone IV E)
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
    output wire [15:0] lcd_rgb,     // RGB565 16 位

    // ---- SDRAM 接口 ----
    output wire        sdram_clk,
    output wire        sdram_cke,
    output wire        sdram_cs_n,
    output wire        sdram_ras_n,
    output wire        sdram_cas_n,
    output wire        sdram_we_n,
    output wire [1:0]  sdram_ba,
    output wire [12:0] sdram_addr,
    output wire [1:0]  sdram_dqm,
    inout  wire [15:0] sdram_data,

    // ---- SD 卡接口（SPI）----
    output wire        sd_cs,
    output wire        sd_clk,
    output wire        sd_mosi,
    input  wire        sd_miso,

    // ---- 调试 LED ----
    output wire [3:0]  led
);

// ---- 内部信号 ----
wire        clk_30m;      // PLL 输出的 30MHz 像素时钟
wire        pll_locked;   // PLL 锁定指示
wire        rst_n_int;    // 内部复位：外部复位释放 且 PLL 锁定后才释放
wire [9:0]  pixel_x;      // 当前像素横坐标
wire [8:0]  pixel_y;      // 当前像素纵坐标
wire        pixel_valid;  // 有效区指示
wire [15:0] pixel_data;   // 像素数据 RGB565
wire [20:0] sdr_addr;     // SDRAM 读地址
wire        sdr_rd;       // SDRAM 读请求
wire [15:0] sdr_rdata;    // SDRAM 读数据
wire        sdr_rvalid;   // SDRAM 读有效
wire        sdr_busy;     // SDRAM 忙
wire        pop;          // line_buffer 弹出
wire [15:0] pixel;        // line_buffer 像素
wire        lb_valid;     // line_buffer 有效
wire        line_start;   // 行起始脉冲（displayer -> line_buffer）

// ---- SD 卡 + transfer ----
wire [31:0] sd_addr;      // SD 扇区号
wire        sd_rd;        // SD 读请求
wire [7:0]  sd_data;      // SD 字节
wire        sd_valid;     // SD 字节有效
wire        sd_busy;      // SD 忙
wire [20:0] tr_sdr_addr;  // transfer 写地址
wire        tr_sdr_wr;    // transfer 写请求
wire [15:0] tr_sdr_wdata; // transfer 写数据
wire        transfer_done;// transfer 完成
reg         transfer_active; // 写阶段标志
reg  [19:0] start_cnt;    // 启动延时计数
reg         start;        // transfer 启动脉冲（单拍）

// PLL 锁定 + 外部复位：两者都满足才释放内部复位
assign rst_n_int = sys_rst_n & pll_locked;

// ---- 上电延时 10ms 后启动 transfer（等 PLL + SDRAM + SD 初始化）----
always @(posedge sys_clk or negedge rst_n_int) begin
    if (!rst_n_int) begin
        start_cnt <= 20'd0;
        start     <= 1'b0;
    end else if (start_cnt < 20'd500_000) begin
        start_cnt <= start_cnt + 20'd1;
        start     <= (start_cnt == 20'd499_999);   // 10ms 时拉高 1 拍
    end else
        start <= 1'b0;
end

// ---- 写阶段标志：上电后先写 SDRAM，写完切读 ----
always @(posedge sys_clk or negedge rst_n_int) begin
    if (!rst_n_int)
        transfer_active <= 1'b1;
    else if (transfer_done)
        transfer_active <= 1'b0;
end

// ---- 仲裁：写阶段 transfer 驱动，读阶段 line_buffer 驱动 ----
wire [20:0] sdr_addr_arb = transfer_active ? tr_sdr_addr : sdr_addr;
wire        sdr_rd_arb   = transfer_active ? 1'b0        : sdr_rd;
wire        sdr_wr_arb   = transfer_active ? tr_sdr_wr   : 1'b0;

// ---- 加载期黑屏：传输图片期间不显示未初始化 RAM 的随机内容 ----
wire [15:0] pixel_data_blanked = transfer_active ? 16'h0000 : pixel_data;

// ---- 调试 LED：定位卡在哪一步 ----
reg sdr_init_done;
reg sd_init_done;
reg transfer_done_latch;
always @(posedge sys_clk or negedge rst_n_int) begin
    if (!rst_n_int) begin
        sdr_init_done      <= 1'b0;
        sd_init_done       <= 1'b0;
        transfer_done_latch<= 1'b0;
    end else begin
        if (!sdr_busy)      sdr_init_done       <= 1'b1;  // SDRAM 就绪
        if (!sd_busy)       sd_init_done        <= 1'b1;  // SD 就绪
        if (transfer_done)  transfer_done_latch <= 1'b1;  // 图片传输完成
    end
end

assign led[0] = sdr_init_done;        // SDRAM 初始化完成
assign led[1] = sd_init_done;         // SD 卡初始化完成
assign led[2] = transfer_done_latch;  // 图片传输完成
assign led[3] = rst_n_int;            // FPGA 在跑（复位已释放）

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
    .pixel_data  (pixel_data_blanked),
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
    .clk         (clk_30m),
    .rst_n       (rst_n_int),
    .pixel_valid (pixel_valid),
    .pixel_x     (pixel_x),
    .pixel_y     (pixel_y),
    .pixel_data  (pixel_data),
    // line_buffer 接口
    .pixel       (pixel),
    .valid       (lb_valid),
    .pop         (pop),
    .line_start  (line_start)
);

// ---- SDRAM 驱动（50MHz）----
sramdriver u_sramdriver (
    .clk         (sys_clk),    // 50MHz
    .rst_n       (rst_n_int),
    // 上层接口（仲裁：写阶段 transfer，读阶段 line_buffer）
    .addr        (sdr_addr_arb),
    .wr          (sdr_wr_arb),
    .wdata       (tr_sdr_wdata),
    .rd          (sdr_rd_arb),
    .rdata       (sdr_rdata),
    .rvalid      (sdr_rvalid),
    .busy        (sdr_busy),
    // SDRAM 物理接口
    .sdram_clk   (sdram_clk),
    .sdram_cke   (sdram_cke),
    .sdram_cs_n  (sdram_cs_n),
    .sdram_ras_n (sdram_ras_n),
    .sdram_cas_n (sdram_cas_n),
    .sdram_we_n  (sdram_we_n),
    .sdram_ba    (sdram_ba),
    .sdram_addr  (sdram_addr),
    .sdram_dqm   (sdram_dqm),
    .sdram_data  (sdram_data)
);

// ---- 行双缓冲（50MHz 读 SDRAM，30MHz 给显示）----
line_buffer u_line_buffer (
    .clk_50m    (sys_clk),
    .clk_30m    (clk_30m),
    .rst_n      (rst_n_int),
    // SDRAM 读接口
    .sdr_addr   (sdr_addr),
    .sdr_rd     (sdr_rd),
    .sdr_rdata  (sdr_rdata),
    .sdr_rvalid (sdr_rvalid),
    .sdr_busy   (sdr_busy),
    .en         (!transfer_active),
    // 显示接口
    .pop        (pop),
    .line_start (line_start),
    .pixel      (pixel),
    .valid      (lb_valid)
);

// ---- SD 卡驱动（50MHz，三段式：statemachine + sendmodule + recievmodule）----
statemachine u_sd_stm (
    .clk      (sys_clk),
    .rst_n    (rst_n_int),
    .sd_addr  (sd_addr),
    .sd_rd    (sd_rd),
    .sd_data  (sd_data),
    .sd_valid (sd_valid),
    .sd_busy  (sd_busy),
    .sd_cs    (sd_cs),
    .sd_clk   (sd_clk),
    .sd_mosi  (sd_mosi),
    .sd_miso  (sd_miso)
);

// ---- SD → SDRAM 桥接（50MHz）----
transfer u_transfer (
    .clk          (sys_clk),
    .rst_n        (rst_n_int),
    .start        (start),
    .start_sector (32'd1000),   // 图片在 SD 卡的起始扇区
    .sector_num   (11'd1350),   // 720*480*2/512 = 1350 扇区
    .base_addr    (21'd0),      // 写入 SDRAM 起始地址
    .done         (transfer_done),
    // 到 sd_ctrl
    .sd_addr      (sd_addr),
    .sd_rd        (sd_rd),
    .sd_data      (sd_data),
    .sd_valid     (sd_valid),
    .sd_busy      (sd_busy),
    // 到 sramdriver
    .sdr_addr     (tr_sdr_addr),
    .sdr_wr       (tr_sdr_wr),
    .sdr_wdata    (tr_sdr_wdata),
    .sdr_busy     (sdr_busy)
);

endmodule
