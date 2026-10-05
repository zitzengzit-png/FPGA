// ============================================================
// 模块：recievmodule —— SPI 接收模块（字节级，SPI 模式 0）
// 接口只暴露：clk（工作时钟）/ spi_clk（位节奏参考）/ busy / 数据
// 本模块不参与分频与状态机：状态机给 start，本模块按 spi_clk
// 边沿采样 miso，采满一个字节给 done 脉冲 + 数据
//
// 设计要点：
// 1. 本模块跑在系统时钟 50MHz，spi_clk 只作为边沿参考信号
//    （与 sendmodule 相同，保持单时钟域）
// 2. 收到 start 后先锁存请求，等下一个 SCK 下降沿才正式开始
//    采样——这样字节边界与发送端/卡的位节奏对齐：
//    下降沿是"新字节的起点"，之后第 1~8 个上升沿采 bit7~bit0
// 3. SPI 模式 0：数据在 SCK 下降沿变化、上升沿被采样，
//    所以只在上升沿采样，下降沿只做字节对齐
// 4. start 只在空闲时有效（状态机保证在 done 之后才发请求）
// ============================================================
`timescale 1ns / 1ps

module recievmodule (
    input  wire        clk,        // 系统时钟 50MHz
    input  wire        rst_n,      // 复位（低有效）
    input  wire        spi_clk,    // 状态机分频产生的 SPI 时钟（只做边沿参考）
    input  wire        start,      // 启动脉冲：锁存请求，下一个下降沿开始采样
    input  wire        miso,       // 卡输出数据线
    output reg  [7:0]  data,       // 收到的字节（done 脉冲时有效）
    output reg         done,       // 一个字节接收完成脉冲
    output reg         busy        // 忙：正在接收一个字节
);

// ---- SPI 时钟边沿检测 ----
reg spi_clk_d;
always @(posedge clk) spi_clk_d <= spi_clk;
wire spi_fall = spi_clk_d & ~spi_clk;   // 下降沿：新字节对齐点
wire spi_rise = ~spi_clk_d & spi_clk;   // 上升沿：采样点

// ---- 接收移位逻辑 ----
reg        pending;     // 已锁存接收请求，等下一个下降沿正式开始
reg [2:0]  bit_cnt;     // 已采样位数：0~7
reg [7:0]  shift_reg;   // 移位寄存器

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        data      <= 8'd0;
        done      <= 1'b0;
        busy      <= 1'b0;
        pending   <= 1'b0;
        bit_cnt   <= 3'd0;
        shift_reg <= 8'd0;
    end
    else begin
        done <= 1'b0;

        // 锁存请求（只在空闲时有效，防止打断进行中的字节）
        if (start && !busy)
            pending <= 1'b1;

        if (spi_fall) begin
            // 下降沿 = 新字节起点；pending||start 覆盖"请求与下降沿同拍到达"
            // 的紧时序情况（快时钟模式下状态机反应链只有几拍）
            if (pending || start) begin
                busy    <= 1'b1;
                pending <= 1'b0;
                bit_cnt <= 3'd0;
            end
        end

        if (spi_rise && busy) begin
            // 上升沿采样
            shift_reg <= {shift_reg[6:0], miso};
            if (bit_cnt == 3'd7) begin
                // 第 8 位采完，输出字节
                data    <= {shift_reg[6:0], miso};
                done    <= 1'b1;
                busy    <= 1'b0;
                bit_cnt <= 3'd0;
            end
            else
                bit_cnt <= bit_cnt + 3'd1;
        end
    end
end

endmodule
