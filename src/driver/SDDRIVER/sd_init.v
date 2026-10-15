// ============================================================
// 模块：sd_init —— SD 卡初始化模块（完全按正点原子教程第52章）
// 功能：完成 SD 卡 SPI 模式上电初始化
//   CMD0 → CMD8 → CMD55 → ACMD41，成功后 sd_init_done 拉高
// 时序：CPOL=1（SCK 空闲高）、初始化时钟 250kHz、上电延时 20ms
// ============================================================
`timescale 1ns / 1ps

module sd_init(
    input          clk_ref      ,  // 时钟信号 50MHz
    input          rst_n        ,  // 复位信号，低电平有效

    input          sd_miso      ,  // SD 卡 SPI 串行输入数据信号
    output         sd_clk       ,  // SD 卡 SPI 时钟信号
    output  reg    sd_cs        ,  // SD 卡 SPI 片选信号
    output  reg    sd_mosi      ,  // SD 卡 SPI 串行输出数据信号
    output  reg    sd_init_done    // SD 卡初始化完成信号
);

// ============ 参数定义 ============
// SD 卡软件复位命令，命令号及参数为固定值，CRC 也为固定值 CRC=8'h95
parameter CMD0   = {8'h40, 8'h00, 8'h00, 8'h00, 8'h00, 8'h95};
// 接口状态命令，发送主设备电压范围，区分 SD 卡版本（2.0 及以后才支持）
parameter CMD8   = {8'h48, 8'h00, 8'h00, 8'h01, 8'haa, 8'h87};
// 告诉 SD 卡接下来的命令是应用相关命令，不需要 CRC
parameter CMD55  = {8'h77, 8'h00, 8'h00, 8'h00, 8'h00, 8'hff};
// 发送操作寄存器(OCR)内容，不需要 CRC
parameter ACMD41 = {8'h69, 8'h40, 8'h00, 8'h00, 8'h00, 8'hff};
// 时钟分频系数，初始化时降低时钟频率：50M/250K = 200
parameter DIV_FREQ      = 200;
// 上电至少等待 74 个同步时钟周期（等待期间 sd_cs=1, sd_mosi=1）
parameter POWER_ON_NUM  = 5000;
// 发送命令后等待响应的最大时间 T=100ms：100_000us/4us = 25000
parameter OVER_TIME_NUM = 25000;

// ============ 状态定义（独热码）============
parameter st_idle        = 7'b000_0001;  // 默认状态，上电等待 SD 卡稳定
parameter st_send_cmd0   = 7'b000_0010;  // 发送软件复位命令
parameter st_wait_cmd0   = 7'b000_0100;  // 等待 SD 卡响应
parameter st_send_cmd8   = 7'b000_1000;  // 发送主设备电压范围
parameter st_send_cmd55  = 7'b001_0000;  // 告诉 SD 卡下一条是应用命令
parameter st_send_acmd41 = 7'b010_0000;  // 发送操作寄存器(OCR)内容
parameter st_init_done   = 7'b100_0000;  // SD 卡初始化完成

// ============ 寄存器定义 ============
reg  [7:0]  cur_state;
reg  [7:0]  next_state;

reg  [7:0]  div_cnt;        // 分频计数器
reg         div_clk;        // 分频后的时钟
reg  [12:0] poweron_cnt;    // 上电等待稳定计数器
reg         res_en;         // 接收 SD 卡返回数据有效信号
reg  [47:0] res_data;       // 接收 SD 卡返回数据
reg         res_flag;       // 开始接收返回数据的标志
reg  [5:0]  res_bit_cnt;    // 接收位数据计数器

reg  [5:0]  cmd_bit_cnt;    // 发送指令位计数器
reg  [15:0] over_time_cnt;  // 超时计数器
reg         over_time_en;   // 超时使能信号

wire        div_clk_180deg; // 时钟相位和 div_clk 相差 180 度

// ============ 时钟分频 ============
assign sd_clk         = ~div_clk;       // SD_CLK
assign div_clk_180deg = ~div_clk;       // 相位和 div_clk 相差 180 度

// 时钟分频，div_clk = 250KHz
always @(posedge clk_ref or negedge rst_n) begin
    if(!rst_n) begin
        div_clk <= 1'b0;
        div_cnt <= 8'd0;
    end
    else begin
        if(div_cnt == DIV_FREQ/2-1'b1) begin
            div_clk <= ~div_clk;
            div_cnt <= 8'd0;
        end
        else
            div_cnt <= div_cnt + 1'b1;
    end
end

// ============ 上电等待稳定计数器 ============
always @(posedge div_clk or negedge rst_n) begin
    if(!rst_n)
        poweron_cnt <= 13'd0;
    else if(cur_state == st_idle) begin
        if(poweron_cnt < POWER_ON_NUM)
            poweron_cnt <= poweron_cnt + 1'b1;
    end
    else
        poweron_cnt <= 13'd0;
end

// ============ 接收 SD 卡返回的响应数据 ============
// 在 div_clk_180deg（sd_clk）的上升沿锁存数据
always @(posedge div_clk_180deg or negedge rst_n) begin
    if(!rst_n) begin
        res_en      <= 1'b0;
        res_data    <= 48'd0;
        res_flag    <= 1'b0;
        res_bit_cnt <= 6'd0;
    end
    else begin
        // sd_miso = 0 开始接收响应数据
        if(sd_miso == 1'b0 && res_flag == 1'b0) begin
            res_flag    <= 1'b1;
            res_data    <= {res_data[46:0], sd_miso};
            res_bit_cnt <= res_bit_cnt + 6'd1;
            res_en      <= 1'b0;
        end
        else if(res_flag) begin
            // R1 返回 1 字节，R3/R7 返回 5 字节
            // 统一按 6 字节接收，多出的 1 字节为 NOP（8 个时钟延时）
            res_data    <= {res_data[46:0], sd_miso};
            res_bit_cnt <= res_bit_cnt + 6'd1;
            if(res_bit_cnt == 6'd47) begin
                res_flag    <= 1'b0;
                res_bit_cnt <= 6'd0;
                res_en      <= 1'b1;
            end
        end
        else
            res_en <= 1'b0;
    end
end

// ============ 状态寄存器 ============
always @(posedge div_clk or negedge rst_n) begin
    if(!rst_n)
        cur_state <= st_idle;
    else
        cur_state <= next_state;
end

// ============ 次态逻辑 ============
always @(*) begin
    next_state = st_idle;
    case(cur_state)
        st_idle : begin
            // 上电至少等待 74 个同步时钟周期
            if(poweron_cnt == POWER_ON_NUM)
                next_state = st_send_cmd0;
            else
                next_state = st_idle;
        end
        st_send_cmd0 : begin
            if(cmd_bit_cnt == 6'd47)
                next_state = st_wait_cmd0;
            else
                next_state = st_send_cmd0;
        end
        st_wait_cmd0 : begin
            if(res_en) begin
                if(res_data[47:40] == 8'h01)  // SD 卡返回复位成功
                    next_state = st_send_cmd8;
                else
                    next_state = st_idle;
            end
            else if(over_time_en)             // 响应超时
                next_state = st_idle;
            else
                next_state = st_wait_cmd0;
        end
        st_send_cmd8 : begin
            if(res_en) begin
                // 返回操作电压 [19:16] = 4'b0001 (2.7V~3.6V)
                if(res_data[19:16] == 4'b0001)
                    next_state = st_send_cmd55;
                else
                    next_state = st_idle;
            end
            else
                next_state = st_send_cmd8;
        end
        st_send_cmd55 : begin
            if(res_en) begin
                if(res_data[47:40] == 8'h01)  // SD 卡返回空闲状态
                    next_state = st_send_acmd41;
                else
                    next_state = st_send_cmd55;
            end
            else
                next_state = st_send_cmd55;
        end
        st_send_acmd41 : begin
            if(res_en) begin
                if(res_data[47:40] == 8'h00)  // 初始化完成
                    next_state = st_init_done;
                else
                    next_state = st_send_cmd55;  // 初始化未完成，重新发起
            end
            else
                next_state = st_send_acmd41;
        end
        st_init_done : next_state = st_init_done;
        default      : next_state = st_idle;
    endcase
end

// ============ 任务执行 ============
// SD 卡在 sd_clk 上升沿锁存数据，因此在 sd_clk 下降沿输出数据
// 为统一在 always 块中使用上升沿触发，此处使用和 sd_clk 相位相差 180 度的 div_clk
always @(posedge div_clk or negedge rst_n) begin
    if(!rst_n) begin
        sd_cs         <= 1'b1;
        sd_mosi       <= 1'b1;
        sd_init_done  <= 1'b0;
        cmd_bit_cnt   <= 6'd0;
        over_time_cnt <= 16'd0;
        over_time_en  <= 1'b0;
    end
    else begin
        over_time_en <= 1'b0;
        case(cur_state)
            st_idle : begin
                sd_cs   <= 1'b1;   // 上电稳定期间 sd_cs=1
                sd_mosi <= 1'b1;   // sd_mosi=1
            end
            st_send_cmd0 : begin
                cmd_bit_cnt <= cmd_bit_cnt + 6'd1;
                sd_cs       <= 1'b0;
                sd_mosi     <= CMD0[6'd47 - cmd_bit_cnt];  // 先发 CMD0 高位
                if(cmd_bit_cnt == 6'd47)
                    cmd_bit_cnt <= 6'd0;
            end
            st_wait_cmd0 : begin
                sd_mosi <= 1'b1;
                if(res_en)          // SD 卡返回响应
                    sd_cs <= 1'b1;  // 接收完成后拉高，进入 SPI 模式
                over_time_cnt <= over_time_cnt + 1'b1;
                if(over_time_cnt == OVER_TIME_NUM - 1'b1)
                    over_time_en <= 1'b1;
                if(over_time_en)
                    over_time_cnt <= 16'd0;
            end
            st_send_cmd8 : begin
                if(cmd_bit_cnt <= 6'd47) begin
                    cmd_bit_cnt <= cmd_bit_cnt + 6'd1;
                    sd_cs       <= 1'b0;
                    sd_mosi     <= CMD8[6'd47 - cmd_bit_cnt];
                end
                else begin
                    sd_mosi <= 1'b1;
                    if(res_en) begin
                        sd_cs       <= 1'b1;
                        cmd_bit_cnt <= 6'd0;
                    end
                end
            end
            st_send_cmd55 : begin
                if(cmd_bit_cnt <= 6'd47) begin
                    cmd_bit_cnt <= cmd_bit_cnt + 6'd1;
                    sd_cs       <= 1'b0;
                    sd_mosi     <= CMD55[6'd47 - cmd_bit_cnt];
                end
                else begin
                    sd_mosi <= 1'b1;
                    if(res_en) begin
                        sd_cs       <= 1'b1;
                        cmd_bit_cnt <= 6'd0;
                    end
                end
            end
            st_send_acmd41 : begin
                if(cmd_bit_cnt <= 6'd47) begin
                    cmd_bit_cnt <= cmd_bit_cnt + 6'd1;
                    sd_cs       <= 1'b0;
                    sd_mosi     <= ACMD41[6'd47 - cmd_bit_cnt];
                end
                else begin
                    sd_mosi <= 1'b1;
                    if(res_en) begin
                        sd_cs       <= 1'b1;
                        cmd_bit_cnt <= 6'd0;
                    end
                end
            end
            st_init_done : begin
                sd_init_done <= 1'b1;
                sd_cs        <= 1'b1;
                sd_mosi      <= 1'b1;
            end
            default : begin
                sd_cs   <= 1'b1;
                sd_mosi <= 1'b1;
            end
        endcase
    end
end

endmodule
