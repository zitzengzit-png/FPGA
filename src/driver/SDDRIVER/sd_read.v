// ============================================================
// 模块：sd_read —— SD 卡读数据模块（正点原子教程第52章原版）
// 功能：发送 CMD17 读单块，接收 512 字节（256 个 16 位数据）
// 时钟：clk_ref(50MHz) 发送/控制，clk_ref_180deg(50MHz 反相) 采样
//       两者相位差 180 度，SD 卡时钟 = clk_ref_180deg（50MHz）
// ============================================================
`timescale 1ns / 1ps

module sd_read(
    input                clk_ref      ,  // 时钟 50MHz
    input                clk_ref_180deg, // 时钟 50MHz 反相（与 clk_ref 相差 180 度）
    input                rst_n        ,  // 复位，低有效
    input                sd_miso      ,  // SD 卡 SPI 输入
    output  reg          sd_cs        ,  // SD 卡片选
    output  reg          sd_mosi      ,  // SD 卡 SPI 输出
    input                rd_start_en  ,  // 开始读 SD 卡数据
    input        [31:0]  rd_sec_addr  ,  // 读数据扇区地址
    output  reg          rd_busy      ,  // 读数据忙信号
    output  reg          rd_val_en    ,  // 读数据有效信号
    output  reg  [15:0]  rd_val_data     // 读数据（16 位）
);

// ============ 寄存器 ============
reg        rd_en_d0;
reg        rd_en_d1;
reg        res_en;
reg [7:0]  res_data;
reg        res_flag;
reg [5:0]  res_bit_cnt;

reg        rx_en_t;
reg [15:0] rx_data_t;
reg        rx_flag;
reg [3:0]  rx_bit_cnt;
reg [8:0]  rx_data_cnt;
reg        rx_finish_en;

reg [3:0]  rd_ctrl_cnt;
reg [47:0] cmd_rd;
reg [5:0]  cmd_bit_cnt;
reg        rd_data_flag;

// ============ 超时保护（本次新增，教程原版没有）============
// 原版在「等 R1 响应」(rd_ctrl_cnt=1) 和「等 0xFE 数据令牌/收 512 字节」(rd_ctrl_cnt=2)
// 两处都是无限等待：卡不响应（未插好、卡不兼容、读时钟太快）时 rd_busy 会永久为 1，
// 于是 sd_ctrl 的 sd_busy 永久为 1 → transfer 永远请求不到扇区 → transfer_done 永远不来
// → 顶层 transfer_active 永久为 1 → 屏幕永久黑屏（这就是黑屏的直接机理之一）。
// 现在超时后放弃本次读、回到空闲并释放 rd_busy，让上层可以重新发起读请求。
// 说明：用 parameter（可在仿真里 defparam 缩短）而非 localparam，
//       便于 testbench 快速验证「超时后不再永久卡死」的行为。
// 【本次调整】把超时从 500ms 缩到 50ms：
//   正常读一个扇区只要 83us（512B@50MHz），50ms 余量极大；
//   而超时越短，单次“坏读”的重试就越快 —— 顶层看门狗 2s 内能容纳十几次重试，
//   避免出现「坏读一次就把启动窗口耗光 → 这次上电黑屏」的随机失败。
parameter OVER_TIME = 24'd2_500_000;    // 50ms @50MHz
reg [23:0] over_cnt;

wire       pos_rd_en;

// ============ rd_start_en 上升沿检测 ============
assign pos_rd_en = (~rd_en_d1) & rd_en_d0;

always @(posedge clk_ref or negedge rst_n) begin
    if(!rst_n) begin
        rd_en_d0 <= 1'b0;
        rd_en_d1 <= 1'b0;
    end
    else begin
        rd_en_d0 <= rd_start_en;
        rd_en_d1 <= rd_en_d0;
    end
end


// ============ 接收响应数据（clk_ref_180deg 上升沿采样）============
always @(posedge clk_ref_180deg or negedge rst_n) begin
    if(!rst_n) begin
        res_en      <= 1'b0;
        res_data    <= 8'd0;
        res_flag    <= 1'b0;
        res_bit_cnt <= 6'd0;
    end
    else begin
        // sd_miso = 0 开始接收响应数据（R1 起始位）
        if(sd_miso == 1'b0 && res_flag == 1'b0) begin
            res_flag    <= 1'b1;
            res_data    <= {res_data[6:0], sd_miso};
            res_bit_cnt <= res_bit_cnt + 6'd1;
            res_en      <= 1'b0;
        end
        else if(res_flag) begin
            res_data    <= {res_data[6:0], sd_miso};
            res_bit_cnt <= res_bit_cnt + 6'd1;
            if(res_bit_cnt == 6'd7) begin
                res_flag    <= 1'b0;
                res_bit_cnt <= 6'd0;
                res_en      <= 1'b1;
            end
        end
        else
            res_en <= 1'b0;
    end
end

// ============ 接收有效数据（clk_ref_180deg 上升沿采样）============
always @(posedge clk_ref_180deg or negedge rst_n) begin
    if(!rst_n) begin
        rx_en_t      <= 1'b0;
        rx_data_t    <= 16'd0;
        rx_flag      <= 1'b0;
        rx_bit_cnt   <= 4'd0;
        rx_data_cnt  <= 9'd0;
        rx_finish_en <= 1'b0;
    end
    else begin
        rx_en_t      <= 1'b0;
        rx_finish_en <= 1'b0;

        if(rd_data_flag) begin
            if(rx_flag == 1'b0) begin
                // 检测数据头 0xFE（8'hFE 最后一位为 0，检测第一个低电平）
                if(sd_miso == 1'b0) begin
                    rx_flag     <= 1'b1;
                    rx_bit_cnt  <= 4'd0;
                    rx_data_cnt <= 9'd0;
                end
            end
            else begin
                // 接收 16 位数据
                rx_data_t  <= {rx_data_t[14:0], sd_miso};
                rx_bit_cnt <= rx_bit_cnt + 4'd1;
                if(rx_bit_cnt == 4'd15) begin
                    rx_bit_cnt  <= 4'd0;
                    rx_data_cnt <= rx_data_cnt + 9'd1;
                    rx_en_t     <= 1'b1;
                    if(rx_data_cnt == 9'd255) begin
                        rx_finish_en <= 1'b1;
                        rx_flag      <= 1'b0;
                        rx_data_cnt  <= 9'd0;
                    end
                end
            end
        end
        // 【新增】非数据阶段（rd_data_flag=0）清零接收状态机：
        // 若某次读因超时被放弃，rx_flag/rx_data_cnt 残留会让下一次读
        // 在错误位置开始收数据（甚至提前凑满 256 字 → 假完成、整扇区错位）。
        else begin
            rx_flag     <= 1'b0;
            rx_bit_cnt  <= 4'd0;
            rx_data_cnt <= 9'd0;
        end
    end
end

// ============ 寄存输出数据 ============
always @(posedge clk_ref or negedge rst_n) begin
    if(!rst_n) begin
        rd_val_en   <= 1'b0;
        rd_val_data <= 16'd0;
    end
    else begin
        if(rx_en_t) begin
            rd_val_en   <= 1'b1;
            rd_val_data <= rx_data_t;
        end
        else
            rd_val_en <= 1'b0;
    end
end

// ============ 读控制流程 ============
always @(posedge clk_ref or negedge rst_n) begin
    if(!rst_n) begin
        sd_cs        <= 1'b1;
        sd_mosi      <= 1'b1;
        rd_ctrl_cnt  <= 4'd0;
        cmd_rd       <= 48'd0;
        cmd_bit_cnt  <= 6'd0;
        rd_busy      <= 1'b0;
        rd_data_flag <= 1'b0;
        over_cnt     <= 24'd0;
    end
    else begin
        case(rd_ctrl_cnt)
            4'd0 : begin
                rd_busy <= 1'b0;
                sd_cs   <= 1'b1;
                sd_mosi <= 1'b1;
                over_cnt <= 24'd0;
                if(pos_rd_en) begin
                    cmd_rd      <= {8'h51, rd_sec_addr, 8'hff};  // CMD17
                    rd_ctrl_cnt <= rd_ctrl_cnt + 4'd1;
                    rd_busy     <= 1'b1;
                end
            end
            4'd1 : begin
                if(cmd_bit_cnt <= 6'd47) begin
                    cmd_bit_cnt <= cmd_bit_cnt + 6'd1;
                    sd_cs       <= 1'b0;
                    sd_mosi     <= cmd_rd[6'd47 - cmd_bit_cnt];
                end
                else begin
                    sd_mosi <= 1'b1;
                    if(res_en) begin
                        rd_ctrl_cnt <= rd_ctrl_cnt + 4'd1;
                        cmd_bit_cnt <= 6'd0;
                        over_cnt    <= 24'd0;
                    end
                    // 【新增】等 R1 响应超时：放弃本次读，回空闲（释放 rd_busy）
                    else if(over_cnt >= OVER_TIME) begin
                        rd_ctrl_cnt <= 4'd0;
                        sd_cs       <= 1'b1;
                        cmd_bit_cnt <= 6'd0;
                        over_cnt    <= 24'd0;
                    end
                    else
                        over_cnt <= over_cnt + 24'd1;
                end
            end
            4'd2 : begin
                rd_data_flag <= 1'b1;
                if(rx_finish_en) begin
                    rd_ctrl_cnt  <= rd_ctrl_cnt + 4'd1;
                    rd_data_flag <= 1'b0;
                    sd_cs        <= 1'b1;
                    over_cnt     <= 24'd0;
                end
                // 【新增】等数据令牌/收数据超时：放弃本次读，回空闲（释放 rd_busy）
                else if(over_cnt >= OVER_TIME) begin
                    rd_ctrl_cnt  <= 4'd0;
                    rd_data_flag <= 1'b0;
                    sd_cs        <= 1'b1;
                    over_cnt     <= 24'd0;
                end
                else
                    over_cnt <= over_cnt + 24'd1;
            end
            default : begin
                // 空闲等待
                sd_cs       <= 1'b1;
                rd_ctrl_cnt <= rd_ctrl_cnt + 4'd1;
            end
        endcase
    end
end

endmodule
