// ============================================================
// 模块：sd_ctrl —— SD 卡控制器（适配层）
// 组合教程第52章的 sd_init（初始化）+ sd_read（读数据），
// 提供当前工程 transfer 使用的字节接口：
//   sd_addr/sd_rd/sd_data/sd_valid/sd_busy
// ============================================================
`timescale 1ns / 1ps

module sd_ctrl (
    input  wire        clk,        // 系统时钟 50MHz
    input  wire        rst_n,      // 复位（低有效）
    input  wire [31:0] sd_addr,    // 扇区号
    input  wire        sd_rd,      // 读请求（脉冲）
    output wire [7:0]  sd_data,    // 读出的字节
    output wire        sd_valid,   // 字节有效
    output wire        sd_busy,    // 忙（初始化中或读数据中）
    input  wire        sd_miso,    // 卡输出
    output wire        sd_cs,      // 片选
    output wire        sd_clk,     // SPI 时钟
    output wire        sd_mosi     // 主机输出
);

// ---- sd_init / sd_read 输出线 ----
wire        init_done;
wire        init_sd_clk, init_sd_cs, init_sd_mosi;
wire        rd_sd_cs, rd_sd_mosi;
wire        rd_busy, rd_val_en;
wire [15:0] rd_val_data;
// 50MHz 反相时钟（教程的 clk_ref_180deg，用于 sd_read 采样 + 读数据时钟）
wire        clk_180 = ~clk;

// ---- SD 卡初始化（教程第52章 sd_init，内部 250kHz 分频）----
sd_init u_sd_init (
    .clk_ref      (clk),
    .rst_n        (rst_n),
    .sd_miso      (sd_miso),
    .sd_clk       (init_sd_clk),
    .sd_cs        (init_sd_cs),
    .sd_mosi      (init_sd_mosi),
    .sd_init_done (init_done)
);

// ---- SD 卡读数据（教程第52章 sd_read，内部 6.25MHz 分频）----
sd_read u_sd_read (
    .clk_ref       (clk),
    .clk_ref_180deg(clk_180),           // 50MHz 反相
    .rst_n         (rst_n),
    .sd_miso       (sd_miso),
    .sd_cs         (rd_sd_cs),
    .sd_mosi       (rd_sd_mosi),
    .rd_start_en   (sd_rd & init_done), // 初始化完成后才响应读请求
    .rd_sec_addr   (sd_addr),
    .rd_busy       (rd_busy),
    .rd_val_en     (rd_val_en),
    .rd_val_data   (rd_val_data)
);

// ---- 引脚仲裁：初始化时 sd_init 驱动，初始化后 sd_read 驱动 ----
assign sd_cs   = init_done ? rd_sd_cs   : init_sd_cs;
assign sd_mosi = init_done ? rd_sd_mosi : init_sd_mosi;
assign sd_clk  = init_done ? clk_180    : init_sd_clk;

// ---- 忙：初始化中 或 读数据中 ----
assign sd_busy = !init_done || rd_busy;

// ---- 16→8 字节转换：先低字节后高字节（本工程必须如此，勿改）----
// 【实机结论】SD 卡里的图片 bin 是「低字节在前」（小端）：每个 RGB565 像素的
//   两个字节在文件中先存低字节。而 transfer 会把「先到的字节」当作高字节
//   打包成 {先到, 后到}。因此这里必须先输出低字节，打包后写进 SDRAM 的
//   16bit 字才是 {高字节, 低字节} —— 颜色才正确。
//   如果把顺序改成「先高字节」，整体颜色会反掉（曾经改过一版，实机偏色）。
reg [7:0]  data_buf;   // 暂存低字节
reg        hi_flag;    // 1=本拍输出低字节
reg        byte_valid;

assign sd_data  = hi_flag ? data_buf : rd_val_data[15:8];
assign sd_valid = byte_valid;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        data_buf   <= 8'd0;
        hi_flag    <= 1'b0;
        byte_valid <= 1'b0;
    end
    else begin
        byte_valid <= 1'b0;
        if(rd_val_en) begin
            // 收到 16 位：锁存低字节，并让下一拍先送出低字节
            data_buf   <= rd_val_data[7:0];
            hi_flag    <= 1'b1;
            byte_valid <= 1'b1;
        end
        else if(hi_flag) begin
            // 再送出高字节
            hi_flag    <= 1'b0;
            byte_valid <= 1'b1;
        end
    end
end

endmodule