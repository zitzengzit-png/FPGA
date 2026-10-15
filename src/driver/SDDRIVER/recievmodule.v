// ============================================================
// 模块：recievmodule —— SPI 接收模块（字节级）
// 接口：clk（工作时钟）/ spi_clk（位节奏参考）/ align_start（对齐方式）/ busy / 数据
// 本模块不参与分频与状态机：状态机给 start，本模块按 spi_clk 采样 miso，
// 采满一个字节给 done 脉冲 + 数据
//
// 设计要点：
// 1. 本模块跑在系统时钟 50MHz，spi_clk 只作为边沿参考信号
//    （与 sendmodule 相同，保持单时钟域）
// 2. 【原 bug 修复】align_start 选择两种字节对齐方式：
//    · align_start=1：检测 MISO 拉低（R1 响应最高位固定为 0）作为字节
//      起点，用于收 R1。原来固定用「SCK 下降沿」对齐字节边界，但真实
//      SD 卡回 R1 前有 NCR 延迟(8~64 SCK，不确定)，R1 起始位不一定落在
//      下降沿决定的字节边界上，导致整串响应错位、初始化失败。
//      改为「检测 MISO 拉低」，字节边界自适应 R1 起始位。
//    · align_start=0：固定用「SCK 下降沿」对齐字节边界，用于收数据
//      （数据令牌/数据字节的起始位不固定为 0，不能靠检测 MISO 拉低）。
// 3. SPI 模式：数据在 SCK 下降沿变化、上升沿被采样，所以只在上升沿采样；
//    起始位检测也在上升沿采样点进行
// 4. start 只在空闲时有效（状态机保证在 done 之后才发请求）
// ============================================================
`timescale 1ns / 1ps

module recievmodule (
    input  wire        clk,         // 系统时钟 50MHz
    input  wire        rst_n,       // 复位（低有效）
    input  wire        spi_clk,     // 状态机分频产生的 SPI 时钟（只做边沿参考）
    input  wire        start,       // 启动脉冲：锁存请求
    input  wire        align_start, // 1=检测起始位(MISO拉低)对齐；0=固定字节边界
    input  wire        miso,        // 卡输出数据线
    output reg  [7:0]  data,        // 收到的字节（done 脉冲时有效）
    output reg         done,        // 一个字节接收完成脉冲
    output reg         busy         // 忙：正在接收一个字节
);

// ---- SPI 时钟边沿检测 ----
reg spi_clk_d;
always @(posedge clk) spi_clk_d <= spi_clk;
wire spi_fall = spi_clk_d & ~spi_clk;   // 下降沿：固定边界模式的字节对齐点
wire spi_rise = ~spi_clk_d & spi_clk;   // 上升沿：采样点

// ---- 接收移位逻辑 ----
reg        pending;     // 已锁存接收请求
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

        if (align_start) begin
            // ---- 检测起始位模式（用于收 R1 响应）----
            // 在 SCK 上升沿采样点检测 MISO 拉低（R1 最高位固定为 0）
            if (spi_rise) begin
                if (pending) begin
                    if (miso == 1'b0) begin
                        // 检测到起始位：起始位即 bit7，开始收本字节
                        shift_reg <= {shift_reg[6:0], miso};
                        bit_cnt   <= 3'd1;
                        pending   <= 1'b0;
                        busy      <= 1'b1;
                    end
                    // miso==1（空闲/填充位）：继续等起始位
                end
                else if (busy) begin
                    // 继续收剩余 7 位
                    shift_reg <= {shift_reg[6:0], miso};
                    if (bit_cnt == 3'd7) begin
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
        else begin
            // ---- 固定字节边界模式（用于收数据）----
            if (spi_fall) begin
                // 下降沿 = 新字节起点
                if (pending || start) begin
                    busy    <= 1'b1;
                    pending <= 1'b0;
                    bit_cnt <= 3'd0;
                end
            end

            if (spi_rise && busy) begin
                shift_reg <= {shift_reg[6:0], miso};
                if (bit_cnt == 3'd7) begin
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
end

endmodule
