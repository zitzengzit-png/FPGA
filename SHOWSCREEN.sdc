# ============================================================
# 时序约束文件：SHOWSCREEN.sdc
# 主时钟：板载 50MHz（周期 20ns）
# 说明：PLL 输出 30MHz 像素时钟由 Quartus 根据 altpll 自动派生
# ============================================================

# ---- 主时钟约束：50MHz ----
create_clock -name sys_clk -period 20.000 [get_ports {sys_clk}]

# ---- 派生 PLL 输出时钟 ----
derive_pll_clocks

# ---- 自动计算时钟不确定性 ----
derive_clock_uncertainty
