// ============================================================
// 模块：sramdriver —— SDRAM 驱动（W9825G6KH，128Mbit，16bit）
// 功能：上电初始化(200us) + 定时刷新 + 单字读写 + 仲裁
// 注：文件夹叫 SRAM，实际芯片是 SDRAM（同步动态存储器，需刷新）
// ============================================================
`timescale 1ns / 1ps

module sramdriver (
    input  wire        clk,        // 系统时钟 50MHz
    input  wire        rst_n,      // 复位（低有效）

    // ---- 上层接口（地址 + 数据 + 读写，物理层）----
    input  wire [20:0] addr,       // 地址（读/写共用）
    input  wire        wr,         // 写请求脉冲
    input  wire [15:0] wdata,      // 写数据
    input  wire        rd,         // 读请求脉冲（突发读 8 个字）
    output reg  [15:0] rdata,      // 读数据（突发连续 8 个字）
    output reg         rvalid,     // 读数据有效（8 拍脉冲）
    output wire        busy,       // 忙（初始化/读写/刷新中）

    // ---- SDRAM 物理接口（W9825G6KH，128Mbit，16bit）----
    output wire        sdram_clk,   // 时钟：直通 clk(50MHz)，命令/数据都同步到它
    output wire        sdram_cke,   // 时钟使能：恒 1，时钟始终有效
    output wire        sdram_cs_n,  // 片选（低有效）= cmd[3]
    output wire        sdram_ras_n, // 行选通（低有效）= cmd[2]
    output wire        sdram_cas_n, // 列选通（低有效）= cmd[1]
    output wire        sdram_we_n,  // 写使能（低有效）= cmd[0]，与 RAS/CAS 组合成命令
    output wire [1:0]  sdram_ba,    // Bank 地址：固定 2'b00，只用 bank0
    output wire [12:0] sdram_addr,  // 行/列复用地址；ACTIVE 给行地址，读写给列地址(A10=1 自动预充电)
    output wire [1:0]  sdram_dqm,   // 数据掩码：恒 2'b00，不掩码（高/低字节均有效）
    inout  wire [15:0] sdram_data   // 双向数据总线：写时输出 wdata_reg，否则高阻
);

// ============ 参数（按 50MHz 时钟，20ns/周期）============
localparam INIT_WAIT_CNT = 10000;   // 上电等待 200us = 10000 时钟
// 刷新间隔（依据《新起点之FPGA开发指南》46.1.1 节）：
//   本板 SDRAM = W9825G6KH：4 Bank × 8192 行（行地址 13 位，A0~A12）
//                            × 512 列（列地址 9 位，A0~A8） × 16bit = 256Mbit = 32MB
//   要求 64ms 内把 8192 行全部刷新一遍 → 刷新间隔 = 64ms/8192 = 7.8125us
//   本设计 SDRAM 时钟 = 50MHz（20ns/周期）→ 7.8125us / 20ns = 390.6 → 取 390 拍
localparam REFRESH_CNT   = 390;     // 刷新间隔 7.8125us = 390 拍（64ms/8192 行）
localparam CL            = 2;       // CAS 延迟 = 2
localparam MODE_REG      = 12'h223; // 模式寄存器：CL=2, 读突发=8, 写单字(A9=1), 顺序

// ============ SDRAM 命令编码 ============
localparam CMD_NOP = 4'b1111;   // 空操作（CS 拉高，芯片不选中）
localparam CMD_ACT = 4'b0011;   // 激活行
localparam CMD_RD  = 4'b0101;   // 读
localparam CMD_WR  = 4'b0100;   // 写
localparam CMD_PRE = 4'b0010;   // 预充电
localparam CMD_REF = 4'b0001;   // 自动刷新
localparam CMD_MRS = 4'b0000;   // 加载模式寄存器

// ============ 状态机状态 ============
localparam S_INIT_WAIT = 4'd0;   // 上电等待 200us
localparam S_INIT_PRE  = 4'd1;   // 预充电
localparam S_INIT_REF  = 4'd2;   // 初始化刷新 x8
localparam S_INIT_MRS  = 4'd3;   // 加载模式寄存器
localparam S_IDLE      = 4'd4;   // 空闲（仲裁）
localparam S_REFRESH   = 4'd5;   // 自动刷新
localparam S_ACTIVE    = 4'd6;   // 激活行
localparam S_READ      = 4'd7;   // 读
localparam S_WRITE     = 4'd8;   // 写

reg [3:0]  state;
reg [15:0] delay_cnt;      // 通用延时计数器
reg [2:0]  init_ref_cnt;   // 初始化刷新计数（8 次）
reg        rd_wr_n;        // 0=读 1=写（记录当前操作）
reg        ref_req;        // 刷新请求（粘性标志）
reg [20:0] cur_addr;       // 锁存的当前地址
reg [15:0] wdata_reg;      // 锁存的写数据

// ============ 命令（组合逻辑，由状态机直接产生）============
reg [3:0] cmd;
always @(*) begin
    case (state)
        S_INIT_PRE : cmd = CMD_PRE;
        S_INIT_REF : cmd = (delay_cnt == 0) ? CMD_REF : CMD_NOP;
        S_INIT_MRS : cmd = CMD_MRS;
        S_REFRESH  : cmd = (delay_cnt == 0) ? CMD_REF : CMD_NOP;
        S_ACTIVE   : cmd = CMD_ACT;
        S_READ     : cmd = (delay_cnt == 0) ? CMD_RD : CMD_NOP;
        S_WRITE    : cmd = CMD_WR;
        default    : cmd = CMD_NOP;
    endcase
end

// ============ SDRAM 输出 ============
assign sdram_clk   = clk;
assign sdram_cke   = 1'b1;
assign sdram_cs_n  = cmd[3];
assign sdram_ras_n = cmd[2];
assign sdram_cas_n = cmd[1];
assign sdram_we_n  = cmd[0];
assign sdram_ba    = 2'b00;                             // 固定 bank0
assign sdram_dqm   = 2'b00;                             // 不掩码
assign sdram_data  = (state == S_WRITE) ? wdata_reg : 16'hZZZZ;

// 忙：不在空闲态就是忙（初始化/读写/刷新）
assign busy = (state != S_IDLE);

// 地址复用：ACTIVE 给行地址，READ/WRITE 给列地址（A10=1 自动预充电）
// 说明（本设计的地址规划，与文档的 8192 行 × 512 列 器件规格配套）：
//   器件按文档是 4 Bank × 8192 行 × 512 列 = 2^22 字/Bank；
//   本设计的字地址是 21 位（2^21 字 = 4MB，够存 720×480×2B=691200B 的图片），
//   因此取 bank0 里一个 2^21 字的子空间：
//       row = a[20:9]（12 位行地址，A0~A11），col = a[8:0]（9 位列地址，A0~A8）
//   写侧（transfer）和读侧（line_buffer）用的是同一套映射，地址一一对应，功能正确。
//   A12 固定为 0 是该地址规划的自然结果（行号只用 0~4095），
//   所以 Quartus 报的 "sdram_addr[12] Stuck at GND" 是预期行为，不是错误。
//   若以后要把 21 位地址铺满 8192 行，可改为 row = a[20:8]（13 位）、col = a[7:0]。
// 地址复用：ACTIVE 给行地址；READ 给 7 位列地址(突发4)，WRITE 给 9 位列地址(单字)
assign sdram_addr = (state == S_ACTIVE) ? {1'b0, cur_addr[20:9]} :
                    (state == S_INIT_PRE) ? 13'b0010000000000 :
                    (state == S_INIT_MRS) ? {1'b0, MODE_REG} :
                    (state == S_READ)  ? {2'b00, 1'b1, 1'b0, cur_addr[8:3], 3'b000} :  // A10=1, 突发8列地址(6位)
                    (state == S_WRITE) ? {2'b00, 1'b1, 1'b0, cur_addr[8:0]} :          // A10=1 自动预充电
                    13'd0;

// ============ 刷新定时器（自由运行）============
reg [15:0] ref_cnt;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ref_cnt <= 16'd0;
    else if (ref_cnt >= REFRESH_CNT - 1)
        ref_cnt <= 16'd0;
    else
        ref_cnt <= ref_cnt + 16'd1;
end

// ============ 主状态机 ============
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= S_INIT_WAIT;
        delay_cnt    <= 16'd0;
        init_ref_cnt <= 3'd0;
        rd_wr_n      <= 1'b0;
        ref_req      <= 1'b0;
        cur_addr     <= 21'd0;
        wdata_reg    <= 16'd0;
        rdata        <= 16'd0;
        rvalid       <= 1'b0;
    end
    else begin
        rvalid <= 1'b0;    // 默认低，读到时给 1 拍脉冲

        if (ref_cnt >= REFRESH_CNT - 1)
            ref_req <= 1'b1;   // 刷新请求置位（粘性）

        case (state)
            S_INIT_WAIT: begin
                if (delay_cnt == INIT_WAIT_CNT - 1) begin
                    delay_cnt <= 16'd0;
                    state     <= S_INIT_PRE;
                end
                else
                    delay_cnt <= delay_cnt + 16'd1;
            end

            S_INIT_PRE: begin
                state <= S_INIT_REF;
            end

            S_INIT_REF: begin
                if (delay_cnt >= 2) begin
                    delay_cnt <= 16'd0;
                    if (init_ref_cnt == 3'd7) begin
                        init_ref_cnt <= 3'd0;
                        state        <= S_INIT_MRS;
                    end
                    else
                        init_ref_cnt <= init_ref_cnt + 3'd1;
                end
                else
                    delay_cnt <= delay_cnt + 16'd1;
            end

            S_INIT_MRS: begin
                state <= S_IDLE;
            end

            S_IDLE: begin
                if (ref_req) begin
                    ref_req <= 1'b0;
                    state   <= S_REFRESH;
                end
                else if (rd) begin
                    rd_wr_n  <= 1'b0;
                    cur_addr <= addr;
                    state    <= S_ACTIVE;
                end
                else if (wr) begin
                    rd_wr_n   <= 1'b1;
                    cur_addr  <= addr;
                    wdata_reg <= wdata;   // 锁存写数据
                    state     <= S_ACTIVE;
                end
            end

            S_REFRESH: begin
                if (delay_cnt >= 2) begin
                    delay_cnt <= 16'd0;
                    state     <= S_IDLE;
                end
                else
                    delay_cnt <= delay_cnt + 16'd1;
            end

            S_ACTIVE: begin
                state     <= rd_wr_n ? S_WRITE : S_READ;
                delay_cnt <= 16'd0;
            end

            // ---- 读：发读命令，等 CL 后连续采 8 个字（突发8）----
            S_READ: begin
                // 【本次修复】采样窗口必须从 CL+1 开始：
                //   读命令在 delay_cnt==0 这一拍发出，SDRAM 在 CL=2 拍后输出第一个字
                //   （配合 1 拍的命令寄存，数据落在 delay_cnt==3 起的 8 拍上）。
                //   原来从 CL+2 开始采 → 每组 8 个字采成 [w1..w7, 下一拍无效值]：
                //   表现为图像每 8 像素错位 1 个点 + 每 8 像素 1 个串色点，
                //   即「大面积颜色不对」。全链路仿真里能直接看到 8 个像素出现一次 zzzz。
                if (delay_cnt >= CL + 1 && delay_cnt <= CL + 8) begin
                    rdata  <= sdram_data;   // 采样一个字
                    rvalid <= 1'b1;
                end
                if (delay_cnt == CL + 8) begin
                    state <= S_IDLE;        // 8 个字采完
                end else
                    delay_cnt <= delay_cnt + 16'd1;
            end

            S_WRITE: begin
                state <= S_IDLE;
            end

            default: state <= S_INIT_WAIT;
        endcase
    end
end

endmodule