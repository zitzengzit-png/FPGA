// ============================================================
// 模块：transfer —— SD → SDRAM 桥接
// 功能：从 sd_ctrl 读扇区（字节流），2 字节打包成 1 个 16 位字，
//       按递增地址写进 sramdriver
// ============================================================
`timescale 1ns / 1ps

module transfer (
    input  wire        clk,
    input  wire        rst_n,

    // ---- 启动控制 ----
    input  wire        start,          // 启动脉冲
    input  wire [31:0] start_sector,   // 起始扇区号
    input  wire [10:0] sector_num,     // 读多少个扇区（1 扇区 = 512 字节 = 256 字）
    input  wire [20:0] base_addr,      // 写入 SDRAM 的起始字地址
    output reg         done,           // 完成脉冲

    // ---- 到 sd_ctrl ----
    output reg  [31:0] sd_addr,        // SD 扇区号
    output reg         sd_rd,          // 读请求
    input  wire [7:0]  sd_data,        // SD 字节
    input  wire        sd_valid,       // 字节有效
    input  wire        sd_busy,        // SD 忙

    // ---- 到 sramdriver ----
    output reg  [20:0] sdr_addr,       // SDRAM 字地址
    output reg         sdr_wr,         // 写请求
    output reg  [15:0] sdr_wdata,      // 写数据
    input  wire        sdr_busy        // SDRAM 忙
);

localparam S_IDLE   = 2'd0;
localparam S_REQ    = 2'd1;   // 请求一个扇区
localparam S_COLL   = 2'd2;   // 收集字节 + 打包写
localparam S_FINISH = 2'd3;   // 等最后一字写完

reg [1:0]  state;
reg [9:0]  byte_cnt;     // 当前扇区字节计数 0~511
reg [10:0] sector_cnt;   // 当前扇区计数
reg [7:0]  byte_buf;     // 暂存高字节
reg [31:0] cur_sector;   // 当前 SD 扇区号
reg [20:0] word_addr;    // 当前 SDRAM 字地址
reg        word_valid;   // 有字待写
reg [15:0] word_data;    // 待写字
reg [20:0] word_addr_r;  // 待写字地址

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= S_IDLE;
        done        <= 1'b0;
        sd_rd       <= 1'b0;
        sdr_wr      <= 1'b0;
        byte_cnt    <= 10'd0;
        sector_cnt  <= 10'd0;
        byte_buf    <= 8'd0;
        cur_sector  <= 32'd0;
        word_addr   <= 21'd0;
        word_valid  <= 1'b0;
        word_data   <= 16'd0;
        word_addr_r <= 21'd0;
        sd_addr     <= 32'd0;
        sdr_addr    <= 21'd0;
        sdr_wdata   <= 16'd0;
    end
    else begin
        done   <= 1'b0;
        sd_rd  <= 1'b0;
        sdr_wr <= 1'b0;

        // ==== 通用：写 SDRAM（有字待写且 SDRAM 空闲）====
        if (word_valid && !sdr_busy) begin
            sdr_addr   <= word_addr_r;
            sdr_wdata  <= word_data;
            sdr_wr     <= 1'b1;
            word_valid <= 1'b0;
        end

        case (state)
            // ---- 等待启动 ----
            S_IDLE: begin
                if (start) begin
                    cur_sector <= start_sector;
                    word_addr  <= base_addr;
                    sector_cnt <= 10'd0;
                    byte_cnt   <= 10'd0;
                    word_valid <= 1'b0;
                    state      <= S_REQ;
                end
            end

            // ---- 请求一个扇区 ----
            S_REQ: begin
                sd_addr  <= cur_sector;
                sd_rd    <= 1'b1;
                byte_cnt <= 10'd0;
                state    <= S_COLL;
            end

            // ---- 收集字节、打包写 ----
            S_COLL: begin
                if (sd_valid) begin
                    if (byte_cnt[0] == 1'b0) begin     // 偶数字节：存高字节
                        byte_buf <= sd_data;
                        byte_cnt <= byte_cnt + 10'd1;
                    end else begin                     // 奇数字节：打包一个字
                        word_valid  <= 1'b1;
                        word_data   <= {byte_buf, sd_data};
                        word_addr_r <= word_addr;
                        word_addr   <= word_addr + 21'd1;
                        if (byte_cnt == 10'd511) begin // 本扇区最后一个字节
                            if (sector_cnt == sector_num - 1'b1) begin
                                state <= S_FINISH;     // 收完，等最后一字写完
                            end else begin
                                sector_cnt <= sector_cnt + 10'd1;
                                cur_sector <= cur_sector + 32'd1;
                                state      <= S_REQ;   // 读下一个扇区
                            end
                        end else
                            byte_cnt <= byte_cnt + 10'd1;
                    end
                end
            end

            // ---- 等最后一字写完，给完成脉冲 ----
            S_FINISH: begin
                if (!word_valid) begin
                    done  <= 1'b1;
                    state <= S_IDLE;
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
