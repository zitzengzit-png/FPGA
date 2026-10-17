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
reg         rst_n_int;    // 内部复位：外部复位释放 且 PLL 锁定后，同步释放并展宽
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
wire        row_start;    // 图像区每行行首前 1 拍（行锁定开行）

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
reg  [22:0] start_cnt;    // 启动延时计数（23 位可存 500 万，够 100ms）
reg         start;        // transfer 启动脉冲（单拍）

// PLL 锁定 + 外部复位：两者都满足才释放内部复位
// 【本次改进】复位时序：
//   · 异步拉低：rst_raw_n = sys_rst_n & pll_locked
//   · 同步释放：在 50MHz 域用 2 级同步器消除亚稳
//   · 展宽 ~256 拍（约 5us）：保证 30MHz 域与「不受 rst 控制的 dcfifo 指针相位」
//     都有充足时间稳定（配合 line_buffer 的精确读空，复位后相位是确定的）
wire rst_raw_n = sys_rst_n & pll_locked;
reg  [1:0] rst_sync;
reg  [7:0] rst_stretch;
always @(posedge sys_clk or negedge rst_raw_n) begin
    if (!rst_raw_n) begin
        rst_sync    <= 2'b00;
        rst_stretch <= 8'd0;
        rst_n_int   <= 1'b0;
    end
    else begin
        rst_sync <= {rst_sync[0], 1'b1};          // 2 级同步
        if (rst_stretch != 8'hFF) begin
            rst_stretch <= rst_stretch + 8'd1;
            rst_n_int   <= 1'b0;                  // 展宽期间保持复位
        end
        else
            rst_n_int   <= rst_sync[1];           // 展宽结束后同步释放
    end
end

// ---- 上电延时 100ms 后启动 transfer（等 sd_init 约 20ms 完成初始化）----
always @(posedge sys_clk or negedge rst_n_int) begin
    if (!rst_n_int) begin
        start_cnt <= 23'd0;
        start     <= 1'b0;
    end else if (start_cnt < 23'd5_000_000) begin
        start_cnt <= start_cnt + 23'd1;
        start     <= (start_cnt == 23'd4_999_999);   // 100ms 时拉高 1 拍
    end else
        start <= 1'b0;
end

// ---- 写阶段标志：上电后先写 SDRAM，写完切读 ----
// 【修复】增加「传输看门狗」：transfer_active 是唯一会把屏幕强制拉黑的信号
//        （pixel_data_blanked = transfer_active ? 0 : pixel_data），
//        而它只由 transfer_done 清除，而 transfer_done 依赖整条
//        SD 卡初始化 → CMD17 读 1350 个扇区 → 写 SDRAM 链路全部成功。
//        该链路任一步卡死（卡未插好、CMD8/CMD17 无响应、找不到 0xFE 数据令牌…）
//        transfer_done 就永远不会来 → 屏幕永久黑屏，且无法从画面判断故障点。
//        处理：start 之后 2 秒仍没收到 transfer_done，就强制结束写阶段，
//              把显示通路放开，并显示纯红（见 pixel_data_dbg）：
//              屏幕变红 = LCD 显示通路正常、图片传输失败（再看 LED 定位 SD 状态）。
//        注意：看门狗一旦触发即为「终止」状态（wdog_fired 保持到复位），
//              因为此时 SDRAM 里的图片是不完整的，继续写会把画面写花。
// 图片传输完成（锁存）：看门狗与指示灯都要用它，先声明（ModelSim 对顺序比 Quartus 严格）
reg transfer_done_latch;
localparam SELF_TEST = 1'b1; // 0=正常显示；1=片上自检（屏幕显示绿/红误差图）
localparam WDOG_MAX = 27'd100_000_000;   // 2s @50MHz
reg [26:0] wdog_cnt;
reg        wdog_fired;

always @(posedge sys_clk or negedge rst_n_int) begin
    if (!rst_n_int) begin
        wdog_cnt   <= 27'd0;
        wdog_fired <= 1'b0;
    end
    else if (transfer_done_latch) begin
        wdog_cnt   <= 27'd0;             // 传输已完成，看门狗停止计时
        wdog_fired <= 1'b0;
    end
    else if ((wdog_cnt < WDOG_MAX) && !SELF_TEST)
        wdog_cnt   <= wdog_cnt + 27'd1;
    else
        wdog_fired <= 1'b1;              // 超时：终止写阶段，放出显示
end

always @(posedge sys_clk or negedge rst_n_int) begin
    if (!rst_n_int)
        transfer_active <= 1'b1;
    else if (transfer_done || wdog_fired)
        transfer_active <= 1'b0;         // 正常完成 或 看门狗超时 → 进入显示阶段
end

// ============================================================
// 可选：片上自检（SELF_TEST 已在上面声明，默认 0 不影响正常功能）
//   FPGA 自己把「每个像素编码自己的(行,列)」的图案写进 SDRAM：
//       字 = {行[5:0], 列[9:0]}   (行 0..479, 列 0..719, 按行连续存)
//   然后走完全相同的 line_buffer → displayer → lcd_driver 通路显示，
//   并在显示侧逐像素比对，把结果显示出来：
//       绿 = 该像素数据正确；红 = 错误
//   于是整屏就是一张误差图：全绿 = 数据通路完全正确；
//   红点分布规律（每 8 像素一个/成片/整体偏移）直接暴露问题。
//   做完把 SELF_TEST 改回 0 即恢复正常显示。
// ============================================================
reg        st_run;      // 1 = 正在写自检图案
reg [18:0] st_pix;      // 已写点数 0..345599
reg [9:0]  st_row, st_col;
reg [20:0] st_addr;
reg [2:0]  st_tick;
wire [15:0] st_data_w = {st_row[5:0], st_col[9:0]};
wire        st_wr     = SELF_TEST && st_run && (st_tick == 3'd7) && !sdr_busy;
wire        blank_phase = SELF_TEST ? st_run : transfer_active;

always @(posedge sys_clk or negedge rst_n_int) begin
    if (!rst_n_int) begin
        st_run  <= SELF_TEST;   // 自检模式下复位后自动开始写图案
        st_pix  <= 19'd0;
        st_row  <= 10'd0;
        st_col  <= 10'd0;
        st_addr <= 21'd0;
        st_tick <= 3'd0;
    end
    else if (st_run) begin
        st_tick <= st_tick + 3'd1;
        // 每 8 拍照一次写请求；只有被 SDRAM 接受（!sdr_busy）才前进
        if ((st_tick == 3'd7) && !sdr_busy) begin
            st_pix  <= st_pix + 19'd1;
            st_addr <= st_addr + 21'd1;
            if (st_col == 10'd719) begin
                st_col <= 10'd0;
                if (st_row == 9'd479) st_run <= 1'b0;   // 整幅写完
                else                  st_row <= st_row + 10'd1;
            end
            else
                st_col <= st_col + 10'd1;
        end
    end
end

// ---- 仲裁：写阶段 transfer 驱动，读阶段 line_buffer 驱动 ----
wire [20:0] sdr_addr_arb = SELF_TEST ? (st_run ? st_addr : sdr_addr)
                                     : (transfer_active ? tr_sdr_addr : sdr_addr);
wire        sdr_rd_arb   = SELF_TEST ? (st_run ? 1'b0 : sdr_rd)
                                     : (transfer_active ? 1'b0 : sdr_rd);
wire        sdr_wr_arb   = SELF_TEST ? st_wr
                                     : (transfer_active ? tr_sdr_wr : 1'b0);
wire [15:0] sdr_wdata_arb = SELF_TEST ? st_data_w : tr_sdr_wdata;

// ---- 加载期黑屏：传输图片期间不显示未初始化 RAM 的随机内容 ----
// ---- 看门狗超时提示：显示纯红（RGB565 = F800），证明 LCD 通路正常、图片传输失败 ----
// 自检比对：显示值 == {行[5:0], 列[9:0]} ? 绿 : 红
wire [9:0]  img_x      = (pixel_x >= 10'd40) ? (pixel_x - 10'd40) : 10'd0;
// 注意：pixel_data 比 LCD 上显示的像素早 1 拍（lcd_driver 是寄存输出），
// 所以比对时要用「下一列」的期望值，否则每像素都会判成"列号不对"（全黄/全红）。
wire [15:0] exp_word   = {pixel_y[5:0], (img_x + 10'd1)};
wire        in_img_area = pixel_valid && (pixel_x >= 10'd40) &&
                          (pixel_x < 10'd760) && (pixel_y < 9'd480);
// 三级判定（便于区分"整体水平偏移"和"数据无效"）：
//   绿 0x07E0 = 值完全等于 {本行, 本列}          → 完全正确
//   黄 0xFFE0 = 行号对、列号错                   → 数据是真的，只是纯水平偏移
//   红 0xF800 = 行号也不对（或值是 0/垃圾）      → 行错位或数据无效
wire [15:0] map_pixel  = (pixel_data === exp_word)             ? 16'h07E0 :
                         (pixel_data[15:10] === pixel_y[5:0]) ? 16'hFFE0 : 16'hF800;
wire [15:0] pixel_data_dbg     = wdog_fired ? 16'hF800 : pixel_data;
wire [15:0] pixel_data_sel = SELF_TEST ? (in_img_area ? map_pixel : 16'h0000)
                                       : pixel_data_dbg;
wire [15:0] pixel_data_blanked = blank_phase ? 16'h0000 : pixel_data_sel;

// ---- 调试 LED：定位卡在哪一步 ----
reg sdr_init_done;
reg sd_init_done;
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
// ---- led[2]：传输状态 ----
//   熄灭   = 图片传输还没完成（正常上电后的 0.3s 内属于正常）
//   常亮   = 图片传输完成（正常流程；此时屏幕应显示图片）
//   0.5s 闪 = 看门狗超时（SD 卡没插好/初始化失败/读数据卡死 → 屏幕显示纯红）
reg [24:0] blink_cnt;
reg        blink;
always @(posedge sys_clk or negedge rst_n_int) begin
    if (!rst_n_int) begin
        blink_cnt <= 25'd0;
        blink     <= 1'b0;
    end
    else if (blink_cnt == 25'd25_000_000 - 1) begin   // 0.5s @50MHz
        blink_cnt <= 25'd0;
        blink     <= ~blink;
    end
    else
        blink_cnt <= blink_cnt + 25'd1;
end
assign led[2] = wdog_fired ? blink : transfer_done_latch;
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
    .line_start  (line_start),
    .row_start   (row_start)
);

// ---- SDRAM 驱动（50MHz）----
sramdriver u_sramdriver (
    .clk         (sys_clk),    // 50MHz
    .rst_n       (rst_n_int),
    // 上层接口（仲裁：写阶段 transfer，读阶段 line_buffer）
    .addr        (sdr_addr_arb),
    .wr          (sdr_wr_arb),
    .wdata       (sdr_wdata_arb),
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
    .en         (!blank_phase),
    // 显示接口
    .pop        (pop),
    .line_start (line_start),
    .row_start  (row_start),
    .pixel      (pixel),
    .valid      (lb_valid)
);

// ---- SD 卡驱动（教程第52章 sd_init + sd_read 适配层 sd_ctrl）----
sd_ctrl u_sd_ctrl (
    .clk      (sys_clk),
    .rst_n    (rst_n_int),
    .sd_addr  (sd_addr),
    .sd_rd    (sd_rd),
    .sd_data  (sd_data),
    .sd_valid (sd_valid),
    .sd_busy  (sd_busy),
    .sd_miso  (sd_miso),
    .sd_cs    (sd_cs),
    .sd_clk   (sd_clk),
    .sd_mosi  (sd_mosi)
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
