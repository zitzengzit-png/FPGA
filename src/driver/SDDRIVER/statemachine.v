// ============================================================
// 模块：statemachine —— SD 卡 SPI 状态机（三段式状态机）
// 功能：SPI 时钟分频 + 上电初始化(CMD0/CMD8/ACMD41/CMD58) + 读扇区(CMD17)
// 对外接口与原 sd_ctrl 完全一致，顶层可直接替换
//
// 三段式结构（状态判断与任务执行解耦）：
//   第一段：状态寄存器（当前状态 ← 次态，只做寄存）
//   第二段：次态逻辑（纯组合，只判断"下一步去哪"，不做任何事）
//   第三段：任务执行（时序，按当前状态驱动 send/recv 模块与 IO）
//
// 本模块负责：
//   - SPI 时钟分频（慢 390kHz 初始化 / 快 6.25MHz 数据），
//     spi_clk 同时送给 sendmodule / recievmodule 作为位节奏参考
//   - 字节级调度：给 sendmodule 发 start+data、给 recievmodule 发 start，
//     用两者的 done 脉冲推进流程
// ============================================================
`timescale 1ns / 1ps

module statemachine (
    input  wire        clk,        // 系统时钟 50MHz
    input  wire        rst_n,      // 复位（低有效）

    // ---- 上层接口（与原 sd_ctrl 完全一致）----
    input  wire [31:0] sd_addr,    // 扇区号
    input  wire        sd_rd,      // 读请求脉冲（读一个扇区 512 字节）
    output reg  [7:0]  sd_data,    // 读出的字节
    output reg         sd_valid,   // 数据有效脉冲
    output reg         sd_busy,    // 忙（初始化中或正在读）

    // ---- SD 卡物理接口 ----
    output reg         sd_cs,      // 片选（低有效）
    output wire        sd_clk,     // SPI 时钟（本模块分频产生）
    input  wire        sd_miso,    // 卡输出
    output wire        sd_mosi     // 主机输出（来自 sendmodule）
);

// ============ SPI 参数 ============
localparam SPI_DIV_SLOW = 128;   // 初始化：50MHz/128 ≈ 390kHz（<400kHz）
localparam SPI_DIV_FAST = 8;     // 数据：50MHz/8 = 6.25MHz
localparam R1_TIMEOUT   = 200;   // 等 R1 响应的超时字节数
localparam PWR_CYCLES   = 640;   // 上电发 ≥74 个 SCK（原设计 80×8 拍）

// ============ SD 命令 ============
localparam CMD0   = 6'h00;   // 复位
localparam CMD8   = 6'h08;   // 电压检测
localparam CMD55  = 6'h37;   // 前缀（ACMD 前必发）
localparam ACMD41 = 6'h29;   // 初始化（SDHC 参数 0x40000000）
localparam CMD58  = 6'h3A;   // 读 OCR
localparam CMD17  = 6'h11;   // 读单块 512 字节

// ============ 状态定义 ============
localparam S_POWER_ON  = 4'd0;   // 上电 74+ 时钟
localparam S_SEND_CMD  = 4'd1;   // 发命令（48 位）
localparam S_GET_R1    = 4'd2;   // 收 R1 响应
localparam S_INIT_DONE = 4'd3;   // 初始化完成（等读请求）
localparam S_READ_CMD  = 4'd4;   // 发 CMD17
localparam S_READ_WAIT = 4'd5;   // 等数据令牌 0xFE
localparam S_READ_DATA = 4'd6;   // 读 512 字节

// ============ 内部寄存器/线 ============
reg  [3:0]  state;       // 当前状态
reg  [3:0]  next_state;  // 次态（组合逻辑驱动）
reg  [7:0]  spi_div_cnt; // SPI 分频计数
reg         spi_clk;     // 分频产生的 SPI 时钟
reg         slow_mode;   // 1=慢时钟（初始化）0=快时钟（数据）
reg  [5:0]  cmd_reg;     // 当前命令
reg  [31:0] arg_reg;     // 当前命令参数
reg  [2:0]  init_step;   // 初始化步骤：0=CMD0 1=CMD8 2=CMD55 3=ACMD41 4=CMD58
reg  [2:0]  byte_cnt;    // 命令字节计数 0~5
reg  [9:0]  data_cnt;    // 数据字节计数 0~511
reg  [9:0]  pwr_cnt;     // 上电 SCK 周期计数
reg  [8:0]  resp_cnt;    // R1 超时字节计数（0~199）
reg  [7:0]  tx_data;     // 待发送字节
reg         tx_start;    // 发送启动脉冲
reg         rx_start;    // 接收启动脉冲

wire [7:0]  rx_data;     // 收到的字节
wire        rx_done;     // 接收完成脉冲
wire        rx_busy;
wire        tx_done;     // 发送完成脉冲
wire        tx_busy;
wire        tx_mosi;

// ---- 分频参数与辅助信号 ----
wire [7:0] spi_div  = slow_mode ? SPI_DIV_SLOW : SPI_DIV_FAST;
wire       div_wrap = (spi_div_cnt == spi_div - 1'b1);
wire [7:0] cmd_crc  = (cmd_reg == CMD0) ? 8'h95 :
                      (cmd_reg == CMD8) ? 8'h87 : 8'hFF;

// ---- 命令 6 字节：{01,cmd} + arg[31:0] + CRC ----
function [7:0] cmd_byte;
    input [2:0] idx;
    begin
        case (idx)
            3'd0: cmd_byte = {2'b01, cmd_reg};
            3'd1: cmd_byte = arg_reg[31:24];
            3'd2: cmd_byte = arg_reg[23:16];
            3'd3: cmd_byte = arg_reg[15:8];
            3'd4: cmd_byte = arg_reg[7:0];
            default: cmd_byte = cmd_crc;
        endcase
    end
endfunction

// ============ SPI 时钟分频（自由运行，SCK 持续翻转）============
// 与状态无关，只按 slow_mode 切换频率；send/recv 模块按边沿自行对齐
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        spi_div_cnt <= 8'd0;
    else if (div_wrap)
        spi_div_cnt <= 8'd0;
    else
        spi_div_cnt <= spi_div_cnt + 8'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        spi_clk <= 1'b0;
    else
        spi_clk <= (spi_div_cnt >= spi_div[7:1]);   // 半个周期高
end

assign sd_clk = spi_clk;

// ============ 第一段：状态寄存器 ============
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state <= S_POWER_ON;
    else
        state <= next_state;
end

// ============ 第二段：次态逻辑（纯组合，只判断，不做事）============
always @(*) begin
    next_state = state;
    case (state)
        S_POWER_ON:
            if (pwr_cnt == PWR_CYCLES - 1 && div_wrap)
                next_state = S_SEND_CMD;

        S_SEND_CMD:
            if (tx_done && byte_cnt == 3'd5)
                next_state = S_GET_R1;

        S_GET_R1: begin
            if (rx_done) begin
                if (rx_data[7]) begin                  // 0xFF：卡未响应
                    if (resp_cnt == R1_TIMEOUT - 1)
                        next_state = S_POWER_ON;       // 超时重新初始化
                end
                else begin                             // 收到 R1
                    if (init_step == 3'd4)
                        next_state = S_INIT_DONE;      // CMD58 完成
                    else
                        next_state = S_SEND_CMD;       // 继续发下一条命令
                end
            end
        end

        S_INIT_DONE:
            if (sd_rd)
                next_state = S_READ_CMD;

        S_READ_CMD:
            if (tx_done && byte_cnt == 3'd5)
                next_state = S_READ_WAIT;

        S_READ_WAIT:
            if (rx_done && rx_data == 8'hFE)
                next_state = S_READ_DATA;

        S_READ_DATA:
            if (rx_done && data_cnt == 10'd511)
                next_state = S_INIT_DONE;

        default: next_state = S_POWER_ON;
    endcase
end

// ============ 第三段：任务执行（按当前状态驱动收发与 IO）============
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        slow_mode <= 1'b1;
        cmd_reg   <= CMD0;
        arg_reg   <= 32'd0;
        init_step <= 3'd0;
        byte_cnt  <= 3'd0;
        data_cnt  <= 10'd0;
        pwr_cnt   <= 10'd0;
        resp_cnt  <= 9'd0;
        tx_data   <= 8'hFF;
        tx_start  <= 1'b0;
        rx_start  <= 1'b0;
        sd_cs     <= 1'b1;
        sd_data   <= 8'd0;
        sd_valid  <= 1'b0;
        sd_busy   <= 1'b1;
    end
    else begin
        tx_start <= 1'b0;
        rx_start <= 1'b0;
        sd_valid <= 1'b0;

        case (state)
            // ---- 上电：CS 高，发 ≥74 个 SCK，然后发 CMD0 ----
            S_POWER_ON: begin
                sd_cs <= 1'b1;
                if (div_wrap) begin
                    if (pwr_cnt == PWR_CYCLES - 1) begin
                        cmd_reg  <= CMD0;
                        arg_reg  <= 32'd0;
                        byte_cnt <= 3'd0;
                        tx_data  <= {2'b01, CMD0};   // 第一个命令字节
                        tx_start <= 1'b1;
                    end
                    else
                        pwr_cnt <= pwr_cnt + 10'd1;
                end
            end

            // ---- 发命令（初始化命令与 CMD17 共用）----
            S_SEND_CMD, S_READ_CMD: begin
                sd_cs <= 1'b0;
                if (tx_done) begin
                    if (byte_cnt < 3'd5) begin
                        byte_cnt <= byte_cnt + 3'd1;
                        tx_data  <= cmd_byte(byte_cnt + 3'd1);
                        tx_start <= 1'b1;
                    end
                    else
                        rx_start <= 1'b1;   // 命令发完，交给收状态（R1/令牌）
                end
            end

            // ---- 收 R1 响应，推进初始化步骤 ----
            S_GET_R1: begin
                sd_cs <= 1'b0;
                if (rx_done) begin
                    if (rx_data[7]) begin          // 0xFF：卡未响应
                        resp_cnt <= resp_cnt + 9'd1;
                        if (resp_cnt == R1_TIMEOUT - 1) begin
                            // 超时：回上电重新初始化
                            resp_cnt  <= 9'd0;
                            pwr_cnt   <= 10'd0;
                            init_step <= 3'd0;
                            cmd_reg   <= CMD0;
                            arg_reg   <= 32'd0;
                            slow_mode <= 1'b1;
                            sd_cs     <= 1'b1;
                        end
                        else
                            rx_start <= 1'b1;      // 继续收
                    end
                    else begin                     // 收到 R1
                        resp_cnt <= 9'd0;
                        case (init_step)
                            3'd0: begin            // CMD0 完成 → CMD8
                                init_step <= 3'd1;
                                cmd_reg   <= CMD8;
                                arg_reg   <= 32'h1AA;
                                byte_cnt  <= 3'd0;
                                tx_data   <= {2'b01, CMD8};
                                tx_start  <= 1'b1;
                            end
                            3'd1: begin            // CMD8 完成 → CMD55
                                init_step <= 3'd2;
                                cmd_reg   <= CMD55;
                                arg_reg   <= 32'd0;
                                byte_cnt  <= 3'd0;
                                tx_data   <= {2'b01, CMD55};
                                tx_start  <= 1'b1;
                            end
                            3'd2: begin            // CMD55 完成 → ACMD41
                                init_step <= 3'd3;
                                cmd_reg   <= ACMD41;
                                arg_reg   <= 32'h40FF8000;
                                byte_cnt  <= 3'd0;
                                tx_data   <= {2'b01, ACMD41};
                                tx_start  <= 1'b1;
                            end
                            3'd3: begin            // ACMD41 完成：看 idle 位
                                if (rx_data[0]) begin
                                    // 卡还在 idle：重发 CMD55+ACMD41
                                    init_step <= 3'd2;
                                    cmd_reg   <= CMD55;
                                    arg_reg   <= 32'd0;
                                    byte_cnt  <= 3'd0;
                                    tx_data   <= {2'b01, CMD55};
                                    tx_start  <= 1'b1;
                                end
                                else begin
                                    // 就绪：发 CMD58
                                    init_step <= 3'd4;
                                    cmd_reg   <= CMD58;
                                    arg_reg   <= 32'd0;
                                    byte_cnt  <= 3'd0;
                                    tx_data   <= {2'b01, CMD58};
                                    tx_start  <= 1'b1;
                                end
                            end
                            default: begin         // CMD58 完成：初始化结束
                                slow_mode <= 1'b0; // 切快时钟
                                init_step <= 3'd0;
                                sd_busy   <= 1'b0;
                                sd_cs     <= 1'b1;
                            end
                        endcase
                    end
                end
            end

            // ---- 初始化完成，等读请求 ----
            S_INIT_DONE: begin
                sd_cs <= 1'b1;
                if (sd_rd) begin
                    sd_busy  <= 1'b1;
                    cmd_reg  <= CMD17;
                    arg_reg  <= sd_addr;
                    byte_cnt <= 3'd0;
                    tx_data  <= {2'b01, CMD17};
                    tx_start <= 1'b1;
                end
                else
                    sd_busy <= 1'b0;
            end

            // ---- 等数据令牌 0xFE（R1/填充字节被跳过）----
            S_READ_WAIT: begin
                sd_cs <= 1'b0;
                if (rx_done) begin
                    rx_start <= 1'b1;      // 继续收；若刚收到 0xFE，收的就是第一个数据字节
                    if (rx_data == 8'hFE)
                        data_cnt <= 10'd0;
                end
            end

            // ---- 读 512 字节数据 ----
            S_READ_DATA: begin
                sd_cs <= 1'b0;
                if (rx_done) begin
                    sd_data  <= rx_data;
                    sd_valid <= 1'b1;
                    if (data_cnt < 10'd511) begin
                        data_cnt <= data_cnt + 10'd1;
                        rx_start <= 1'b1;
                    end
                end
            end

            default: begin
                sd_cs   <= 1'b1;
                sd_busy <= 1'b1;
            end
        endcase
    end
end

// ============ 子模块例化 ============
sendmodule u_send (
    .clk     (clk),
    .rst_n   (rst_n),
    .spi_clk (spi_clk),
    .start   (tx_start),
    .data    (tx_data),
    .mosi    (tx_mosi),
    .done    (tx_done),
    .busy    (tx_busy)
);

recievmodule u_recv (
    .clk     (clk),
    .rst_n   (rst_n),
    .spi_clk (spi_clk),
    .start   (rx_start),
    .miso    (sd_miso),
    .data    (rx_data),
    .done    (rx_done),
    .busy    (rx_busy)
);

assign sd_mosi = tx_mosi;

endmodule
