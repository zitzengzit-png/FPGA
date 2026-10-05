// ============================================================
// 模块：sendmodule —— SPI 发送模块（字节级，SPI 模式 0）
// 接口只暴露：clk（工作时钟）/ spi_clk（位节奏参考）/ busy / 数据
// 本模块不参与分频与状态机：状态机给 start + data，本模块按
// spi_clk 边沿把字节移出去，一个字节呈现完给 done 脉冲
//
// 设计要点：
// 1. 本模块跑在系统时钟 50MHz，spi_clk 只作为边沿参考信号
//    （状态机寄存器输出的同步信号），整个设计保持单时钟域，
//    避免用分频时钟直接驱动模块带来的跨时钟域问题
// 2. 状态机保证在上一字节 done 之后才发 start，因此 start 到来时
//    本字节已呈现完毕，无需等待 busy 释放；
//    背靠背字节靠 pending 锁存无缝衔接（48 位命令中间无空位）
// 3. SPI 模式 0：数据在 SCK 下降沿变化、上升沿被采样，
//    所以每来一个下降沿就呈现新的一位
// ============================================================
`timescale 1ns / 1ps

module sendmodule (
    input  wire        clk,        // 系统时钟 50MHz
    input  wire        rst_n,      // 复位（低有效）
    input  wire        spi_clk,    // 状态机分频产生的 SPI 时钟（只做边沿参考）
    input  wire        start,      // 启动脉冲：锁存 data，下一个下降沿开始发送
    input  wire [7:0]  data,       // 待发送字节
    output reg         mosi,       // 串行数据输出（空闲为高）
    output reg         done,       // 一个字节发送完成脉冲（第 8 位已呈现）
    output reg         busy        // 忙：正在发送一个字节
);

// ---- SPI 时钟下降沿检测（spi_clk 是 50MHz 域内的同步信号）----
reg spi_clk_d;
always @(posedge clk) spi_clk_d <= spi_clk;
wire spi_fall = spi_clk_d & ~spi_clk;

// ---- 发送移位逻辑 ----
reg [7:0]  shift_reg;   // 移位寄存器
reg        pending;     // 已锁存待发字节，等下一个下降沿呈现
reg [7:0]  pend_data;   // 待发字节
reg [3:0]  bit_cnt;     // 已呈现位数：1~8，8=本字节呈现完毕（需 4 位存 8）

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mosi      <= 1'b1;
        done      <= 1'b0;
        busy      <= 1'b0;
        shift_reg <= 8'hFF;
        pending   <= 1'b0;
        pend_data <= 8'hFF;
        bit_cnt   <= 4'd0;
    end
    else begin
        done <= 1'b0;

        // 锁存待发字节（无需看 busy：状态机保证只在 done 之后发请求）
        if (start) begin
            pend_data <= data;
            pending   <= 1'b1;
        end

        if (spi_fall) begin
            if (pending) begin
                // 新字节：本下降沿呈现 bit7，下一个上升沿被卡采样
                shift_reg <= pend_data;
                mosi      <= pend_data[7];
                pending   <= 1'b0;
                busy      <= 1'b1;
                bit_cnt   <= 4'd1;
            end
            else if (busy) begin
                if (bit_cnt == 4'd7) begin
                    // 呈现最后一位 bit0（此刻 shift_reg[6] 正是 d0），
                    // 下一个上升沿被卡采样；同时通知状态机本字节已呈现完毕
                    shift_reg <= {shift_reg[6:0], 1'b1};
                    mosi      <= shift_reg[6];
                    bit_cnt   <= 4'd8;
                    done      <= 1'b1;
                end
                else if (bit_cnt == 4'd8) begin
                    // 没有待发字节：本字节结束，回到空闲
                    busy    <= 1'b0;
                    bit_cnt <= 4'd0;
                    mosi    <= 1'b1;
                end
                else begin
                    // 呈现下一位
                    shift_reg <= {shift_reg[6:0], 1'b1};
                    mosi      <= shift_reg[6];
                    bit_cnt   <= bit_cnt + 4'd1;
                end
            end
        end
    end
end

endmodule
