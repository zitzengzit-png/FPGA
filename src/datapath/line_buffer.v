// ============================================================
// line_buffer —— 双 FIFO 乒乓行缓冲
//   写侧(50MHz)：一次填满一块（720 字 = 90 个 8 字突发），填完切到另一块
//   读侧(30MHz)：每行从「最近填满的那一块」顺序弹 720 字
//   ⇒ 显示永远读到完整的一行；行边界由“换槽”决定，与突发边界彻底解耦
//   背压：写侧填某块前，必须等读侧已经离开该块（rd_sel != wr_sel）
// ============================================================
`timescale 1ns / 1ps

module line_buffer (
    input  wire        clk_50m,
    input  wire        clk_30m,
    input  wire        rst_n,
    output reg  [20:0] sdr_addr,
    output wire        sdr_rd,
    input  wire [15:0] sdr_rdata,
    input  wire        sdr_rvalid,
    input  wire        sdr_busy,
    input  wire        pop,
    input  wire        row_start,
    input  wire        line_start,
    output wire [15:0] pixel,
    output wire        valid
);

localparam LINE_WORDS = 720;
// 【注意】rd_ptr 是 SDRAM 的「字地址」（16bit/字），不是字节地址：
//   图像 = 720 x 480 = 345,600 像素 = 345,600 字 = 691,200 字节。
//   transfer 写入的也是 1350 扇区 x 512B / 2 = 345,600 字。
//   因此必须在一帧的 345,600 字处回绕；若写成 691,200（字节数），
//   第二帧会去读 SDRAM 中从未写入的 345,600~691,199，画面按帧交替花屏。
localparam IMG_WORDS  = 21'd345600;   // 480 行 x 720 字/行 = 345,600 字

// ---------------- 双 FIFO（复用同一 IP 两次，每块只用 720 字）----------------
wire [16:0] aq, bq;

reg         wr_sel;    // 写侧：当前要填的块（0=A 1=B）
reg         rd_sel;    // 读侧：当前在读的块
wire        done_sel = ~wr_sel;   // 最近填满的块

reg  [9:0]  wcnt;      // 本行已写字数 0..719
reg  [2:0]  rv_cnt;    // 本突发已收字数
reg         rd_active;
reg  [20:0] rd_ptr;    // 图像流指针（8 对齐，每突发 +8）
reg         line_busy; // 正在填一行

wire        we_a = sdr_rvalid && (wr_sel == 1'b0);
wire        we_b = sdr_rvalid && (wr_sel == 1'b1);
wire        rp_a = pop && (rd_sel == 1'b0);
wire        rp_b = pop && (rd_sel == 1'b1);

ffifo u_fifo_a (
    .aclr(~rst_n), .data({1'b0, sdr_rdata}),
    .wrclk(clk_50m), .wrreq(we_a), .wrfull(), .wrusedw(), .wrempty(),
    .rdclk(clk_30m), .rdreq(rp_a), .q(aq), .rdempty(), .rdfull(), .rdusedw()
);
ffifo u_fifo_b (
    .aclr(~rst_n), .data({1'b0, sdr_rdata}),
    .wrclk(clk_50m), .wrreq(we_b), .wrfull(), .wrusedw(), .wrempty(),
    .rdclk(clk_30m), .rdreq(rp_b), .q(bq), .rdempty(), .rdfull(), .rdusedw()
);

assign sdr_rd = rd_active && !sdr_busy;
assign pixel  = (rd_sel == 1'b0) ? aq[15:0] : bq[15:0];

// ---------------- 跨时钟同步 ----------------
reg dsel_m, dsel_s;      // done_sel : 50M -> 30M
reg rsel_m, rsel_s;      // rd_sel   : 30M -> 50M
always @(posedge clk_30m or negedge rst_n)
    if (!rst_n) begin dsel_m <= 1'b1; dsel_s <= 1'b1; end
    else        begin dsel_m <= done_sel; dsel_s <= dsel_m; end
always @(posedge clk_50m or negedge rst_n)
    if (!rst_n) begin rsel_m <= 1'b1; rsel_s <= 1'b1; end
    else        begin rsel_m <= rd_sel;  rsel_s <= rsel_m; end

// done_sel 一旦翻转，表示"又有一整行填完了"；用它做边沿检测，
// 读侧只在"确实有新行"时才换块，避免锁存到正在被写的那一块（竞争）
reg  dsel_s_d;
wire new_line = (dsel_s != dsel_s_d);

// ---------------- 写侧：按行填满一块（8 字突发）----------------
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        wr_sel    <= 1'b0;
        wcnt      <= 10'd0;
        rv_cnt    <= 3'd0;
        rd_active <= 1'b0;
        rd_ptr    <= 21'd0;
        line_busy <= 1'b0;
        sdr_addr  <= 21'd0;
    end
    else if (!line_busy) begin
        // 开始填新的一块：必须等读侧离开这一块（背压）
        if (!rd_active && (wr_sel != rsel_s)) begin
            line_busy <= 1'b1;
            wcnt      <= 10'd0;
        end
    end
    else begin
        if (!rd_active && !sdr_busy && (wcnt < LINE_WORDS)) begin
            sdr_addr  <= rd_ptr;
            rd_active <= 1'b1;
            rv_cnt    <= 3'd0;
        end
        if (sdr_rvalid) begin
            wcnt   <= wcnt + 10'd1;
            rv_cnt <= rv_cnt + 3'd1;
            if (rv_cnt == 3'd7) begin          // 一个 8 字突发结束
                rd_active <= 1'b0;
                rv_cnt    <= 3'd0;
                if ((rd_ptr + 21'd8) >= IMG_WORDS) rd_ptr <= 21'd0;
                else                               rd_ptr <= rd_ptr + 21'd8;
            end
            if (wcnt == LINE_WORDS - 10'd1) begin  // 本行 720 字填满
                line_busy <= 1'b0;
                wr_sel    <= ~wr_sel;              // 切到另一块
            end
        end
    end
end

// ---------------- 读侧：每行读“最近填满的那一块” ----------------
// 【帧同步】wait_frame：等待“图像第 0 行”与“屏幕帧首(y=0)”对齐。
// 原因：transfer 完成、line_buffer 开始取第 0 行是任意时刻，
//       若在“第一行刚准备好”的下一个 row_start 就开显，
//       第 0 行会落在屏幕的任意 y=K 上；由于 SDRAM 读指针是
//       345600 字循环，整幅图就变成“下移 K 行、底部 K 行卷到顶部”。
// 处理：复位后/加载完成后先等 line_start(y=0, x=39)，且确认
//       A 块（图像第 0 行）已填满(dsel_s==0)，才从屏幕第 0 行开显。
reg        line_act;
reg  [9:0] pcnt;
reg        wait_frame;
assign valid = line_act;

always @(posedge clk_30m or negedge rst_n) begin
    if (!rst_n) begin
        rd_sel     <= 1'b1;      // 复位时让写侧可以先填 A
        line_act   <= 1'b0;
        pcnt       <= 10'd0;
        dsel_s_d   <= dsel_s;
        wait_frame <= 1'b1;      // 先等帧首
    end
    else if (wait_frame) begin
        line_act <= 1'b0;
        pcnt     <= 10'd0;
        // 图像第 0 行在 A 块（dsel_s==0）且已填满，才在屏幕 y=0 开显
        if (line_start && (dsel_s == 1'b0)) begin
            rd_sel     <= 1'b0;  // A = 图像第 0 行
            line_act   <= 1'b1;
            pcnt       <= 10'd0;
            dsel_s_d   <= dsel_s;
            wait_frame <= 1'b0;
        end
    end
    else if (row_start) begin
        dsel_s_d <= dsel_s;
        if (new_line) begin
            rd_sel   <= dsel_s;    // 只在“有新行”时换块（避免锁到正在写的块）
            line_act <= 1'b1;
            pcnt     <= 10'd0;
        end
        else begin
            line_act <= 1'b0;      // 没有新行可用：本行黑，绝不读脏数据
            pcnt     <= 10'd0;
        end
    end
    else if (line_act && pop) begin
        pcnt <= pcnt + 10'd1;
        if (pcnt == LINE_WORDS - 10'd1)
            line_act <= 1'b0;  // 本行 720 字读完
    end
end

endmodule
