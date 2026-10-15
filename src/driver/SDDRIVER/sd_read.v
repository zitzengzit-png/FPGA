// ============================================================
// 模块：sd_read —— SD 卡读数据模块（按正点原子教程第52章，内部分频适配）
// 功能：发送 CMD17 读单块，接收 512 字节（256 个 16 位数据）
// 时序：检测 MISO 起始位收 R1，检测数据头 0xFE，收 256×16bit
// 适配说明：教程原版用外部 clk_ref_180deg(50MHz)采样，本版改为内部分频
//           6.25MHz（div_clk + div_clk_180deg），只需 clk_ref(50MHz) 一个时钟
// ============================================================
`timescale 1ns / 1ps

module sd_read(
    input                clk_ref      ,  // 时钟 50MHz
    input                rst_n        ,  // 复位，低有效
    input                sd_miso      ,  // SD 卡 SPI 输入
    output  reg          sd_cs        ,  // SD 卡片选
    output  reg          sd_mosi      ,  // SD 卡 SPI 输出
    output  wire         sd_clk       ,  // SD 卡时钟（内部分频，供顶层切换）
    input                rd_start_en  ,  // 开始读 SD 卡数据
    input        [31:0]  rd_sec_addr  ,  // 读数据扇区地址
    output  reg          rd_busy      ,  // 读数据忙信号
    output  reg          rd_val_en    ,  // 读数据有效信号
    output  reg  [15:0]  rd_val_data     // 读数据（16 位）
);

// 分频参数：50MHz/8 = 6.25MHz（初始化后快时钟）
parameter DIV_FREQ = 8;

// ============ 寄存器 ============
reg [3:0]  div_cnt;
reg        div_clk;
wire       div_clk_180deg = ~div_clk;

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

wire       pos_rd_en;

// ============ 分频 ============
assign sd_clk = ~div_clk;

always @(posedge clk_ref or negedge rst_n) begin
    if(!rst_n) begin
        div_clk <= 1'b0;
        div_cnt <= 4'd0;
    end
    else begin
        if(div_cnt == DIV_FREQ/2-1'b1) begin
            div_clk <= ~div_clk;
            div_cnt <= 4'd0;
        end
        else
            div_cnt <= div_cnt + 4'd1;
    end
end

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

// ============ 接收响应数据（div_clk_180deg 上升沿采样）============
always @(posedge div_clk_180deg or negedge rst_n) begin
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

// ============ 接收有效数据（div_clk_180deg 上升沿采样）============
always @(posedge div_clk_180deg or negedge rst_n) begin
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
    end
    else begin
        case(rd_ctrl_cnt)
            4'd0 : begin
                rd_busy <= 1'b0;
                sd_cs   <= 1'b1;
                sd_mosi <= 1'b1;
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
                    end
                end
            end
            4'd2 : begin
                rd_data_flag <= 1'b1;
                if(rx_finish_en) begin
                    rd_ctrl_cnt  <= rd_ctrl_cnt + 4'd1;
                    rd_data_flag <= 1'b0;
                    sd_cs        <= 1'b1;
                end
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
