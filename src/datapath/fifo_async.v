// ============================================================
// fifo_async —— 自带复位的异步 FIFO（替代 IP: ipcores/FIFO30M50M/FIFO）
//
// 为什么要自己写：
//   原 IP 生成时未启用异步清零（见 ipcores/FIFO30M50M/FIFO.v：dcfifo_component
//   的 .aclr 悬空、dc_aclr=0），读写指针不受 rst_n 控制。这样一来：
//     · 复位后 FIFO 里残留的“占用数/相位”无法确定 → 每次复位画面都不一样；
//     · 一旦软清空把读指针推过写指针，计数器会绕成“满” → 写侧被 fifo_room
//       永久卡死 → 之后每次复位都是黑屏（复位也救不回来）—— 正是现场现象。
//   本模块读写指针都有复位：复位后一定是「空、相位确定」。
//
// 端口/行为与原 IP 一致，可直接替换：
//   · 深度 2048 × 16bit
//   · lpm_showahead = OFF：q 在 rdreq 后 1 拍有效（保持原有时序，不动 displayer 对齐）
//   · 提供 wrfull/wrempty/wrusedw、rdempty/rdfull/rdusedw
//   · aclr：异步清零（在 50MHz/30MHz 两个域都复位指针；释放端由顶层复位展宽保证干净）
// ============================================================
`timescale 1ns / 1ps

module fifo_async #(
    parameter AW = 11,   // 深度 = 2^AW = 2048
    parameter DW = 16
)(
    input  wire          aclr,      // 异步清零（低有效信号请外部反相为高有效）
    // ---- 写侧 ----
    input  wire          wrclk,
    input  wire          wrreq,
    input  wire [DW-1:0] data,
    output wire          wrfull,
    output wire          wrempty,
    output wire [AW-1:0] wrusedw,
    // ---- 读侧 ----
    input  wire          rdclk,
    input  wire          rdreq,
    output reg  [DW-1:0] q,
    output wire          rdempty,
    output wire          rdfull,
    output wire [AW-1:0] rdusedw
);

localparam DEPTH = (1 << AW);

// ---------------- 存储体 ----------------
reg [DW-1:0] mem [0:DEPTH-1];

// ---------------- 指针（多 1 位用于满/空判断）----------------
reg  [AW:0] wbin, wgray;
reg  [AW:0] rbin, rgray;
// 跨时钟域同步
reg  [AW:0] wgray_s1, wgray_s2;   // 写指针 → 读域
reg  [AW:0] rgray_s1, rgray_s2;   // 读指针 → 写域

// 格雷码转换
function [AW:0] bin2gray(input [AW:0] b);
    bin2gray = b ^ (b >> 1);
endfunction
function [AW:0] gray2bin(input [AW:0] g);
    integer i;
    begin
        gray2bin[AW] = g[AW];
        for (i = AW-1; i >= 0; i = i - 1)
            gray2bin[i] = gray2bin[i+1] ^ g[i];
    end
endfunction

// ---------------- 写侧 ----------------
wire [AW:0] wbin_nxt = wbin + ((wrreq && !wrfull) ? 1'b1 : 1'b0);
wire [AW:0] wgray_nxt = bin2gray(wbin_nxt);
wire [AW:0] rgray_sync = rgray_s2;
// 满：写指针的下一个格雷码 == 读指针（高两位取反、其余相同）
assign wrfull  = (wgray_nxt == {~rgray_sync[AW:AW-1], rgray_sync[AW-2:0]});
assign wrempty = (wgray == rgray_sync);
// 已用字数：与原 IP 一致（11bit，满时饱和到 2047，不会绕成 0）
wire [AW:0] wr_used = wbin - gray2bin(rgray_sync);
assign wrusedw = wr_used[AW] ? {AW{1'b1}} : wr_used[AW-1:0];

always @(posedge wrclk or posedge aclr) begin
    if (aclr) begin
        wbin     <= {AW+1{1'b0}};
        wgray    <= {AW+1{1'b0}};
        rgray_s1 <= {AW+1{1'b0}};
        rgray_s2 <= {AW+1{1'b0}};
    end
    else begin
        if (wrreq && !wrfull) begin
            mem[wbin[AW-1:0]] <= data;   // 写入存储体
            wbin  <= wbin_nxt;
            wgray <= wgray_nxt;
        end
        rgray_s1 <= rgray;
        rgray_s2 <= rgray_s1;
    end
end

// ---------------- 读侧 ----------------
wire [AW:0] rbin_nxt = rbin + ((rdreq && !rdempty) ? 1'b1 : 1'b0);
wire [AW:0] rgray_nxt = bin2gray(rbin_nxt);
wire [AW:0] wgray_sync = wgray_s2;
assign rdempty = (rgray == wgray_sync);
assign rdfull  = (rgray_nxt == {~wgray_sync[AW:AW-1], wgray_sync[AW-2:0]});
wire [AW:0] rd_used = gray2bin(wgray_sync) - rbin;
assign rdusedw = rd_used[AW] ? {AW{1'b1}} : rd_used[AW-1:0];

always @(posedge rdclk or posedge aclr) begin
    if (aclr) begin
        rbin     <= {AW+1{1'b0}};
        rgray    <= {AW+1{1'b0}};
        q        <= {DW{1'b0}};
        wgray_s1 <= {AW+1{1'b0}};
        wgray_s2 <= {AW+1{1'b0}};
    end
    else begin
        // showahead = OFF：先给出 rdreq 之前指针处的数据（比 rdreq 晚 1 拍），再推进指针
        q     <= mem[rbin[AW-1:0]];
        rbin  <= rbin_nxt;
        rgray <= rgray_nxt;
        wgray_s1 <= wgray;
        wgray_s2 <= wgray_s1;
    end
end

endmodule
