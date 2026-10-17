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

// ============ 扇区读卡死保护（本次新增）============
// 原版 S_COLL 无限等 sd_valid：若某次读因卡顿/超时被上层放弃（见 sd_read 的超时保护），
// sd_valid 就再也不会来，transfer 会永久停在 S_COLL → done 永不产生
// → 顶层 transfer_active 永久为 1 → 屏幕永久黑屏。
// 现在：S_COLL 里 1 秒收不到字节，就回退到「本扇区起始字地址」重新读一次
//      （重读是幂等的：同一扇区的数据重写一遍，不会破坏已写入的内容）。
// 用 parameter（不是 localparam）便于 testbench defparam 缩短后快速验证
// 【本次调整】1s → 150ms：坏读重试更快，2s 看门狗内可容错多次。
// 仍保持「大于 sd_read 的 50ms 超时」，避免 sd_read 还在读时就被上层重发请求。
parameter STALL_TIME = 24'd7_500_000;   // 150ms @50MHz（> sd_read 的 50ms 超时）
reg [23:0] stall_cnt;
reg [20:0] sector_base;   // 本扇区在 SDRAM 中的起始字地址

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= S_IDLE;
        stall_cnt   <= 24'd0;
        sector_base <= 21'd0;
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

            // ---- 请求一个扇区（等 sd_read 空闲再发读请求）----
            S_REQ: begin
                if (!sd_busy) begin
                    sd_addr     <= cur_sector;
                    sd_rd       <= 1'b1;
                    byte_cnt    <= 10'd0;
                    sector_base <= word_addr;   // 【新增】记住本扇区起始字地址，便于超时重读
                    stall_cnt   <= 24'd0;       // 【新增】
                    state       <= S_COLL;
                end
                // sd_busy=1（sd_read 忙）时保持等待
            end

            // ---- 收集字节、打包写 ----
            S_COLL: begin
                if (sd_valid) begin
                    stall_cnt <= 24'd0;   // 【新增】收到数据就清零看护计数
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
                // 【新增】1 秒收不到数据（上层读已被放弃/卡无响应）→
                //         回退到本扇区起始地址重新读，避免永久卡在 S_COLL
                else if (stall_cnt >= STALL_TIME) begin
                    word_addr <= sector_base;
                    byte_cnt  <= 10'd0;
                    stall_cnt <= 24'd0;
                    state     <= S_REQ;
                end
                else
                    stall_cnt <= stall_cnt + 24'd1;
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
