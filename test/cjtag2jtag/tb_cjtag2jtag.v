/*****************************************************************************\
|  cjtag2jtag 自检测试平台 (默认 2 线模式, 无激活序列)                            |
|                                                                             |
|  探针模型 (对应 riscv-openocd src/jtag/drivers/ftdi.c 的 OScan1 数据流时序):   |
|   * 每个 JTAG 周期 3 个 TCKC 拍:                                            |
|       A 拍: 驱动 TMSC = ~TDI  (下降沿就已摆好)                               |
|       B 拍: 驱动 TMSC = TMS                                                  |
|       C 拍: 释放 TMSC, 在本拍上升沿采样 TDO (POS_EDGE_IN)                    |
|   * 不发 cJTAG 激活序列 (escape + OAC/EC/CP): 直接开始扫描                     |
|                                                                             |
|  被测: cjtag2jtag + 真正的 hazard3_jtag_dtm                                  |
|  检查:                                                                       |
|   1) 复位后未活动时不自己驱动 TMSC / TCK 保持低                                |
|   2) 空闲后直接扫描: TLR/IR 回读/IDCODE/DTMCS 都与 DTM 定义一致               |
|   3) 多次操作 (每次之间有空闲) 都能正确扫描                                    |
|   4) 失步 (故意发半组) 后, 等一次空闲应能自愈重新对齐                           |
|   5) 全程 TMSC 上不出现 'x' (推挽冲突)                                        |
|                                                                             |
|  用法 (在本目录下):                                                           |
|   make -f Makefile.2p sim            # 期望输出 "=== 全部通过 (errors=0) ==="   |
|  也可手工编译:                                                               |
|   iverilog -g2012 -o tb.vvp -I . tb_cjtag2jtag.v cjtag2jtag.v \              |
|            ../../../../../hdl/debug/dtm/hazard3_jtag_dtm.v ... (见 Makefile)  |
\*****************************************************************************/

`timescale 1ns/1ps
`default_nettype none

module tb_cjtag2jtag;

	// ------------------------------------------------------------------
	// 时钟/复位
	// ------------------------------------------------------------------
	reg clk_dmi = 1'b0;      // DMI 侧时钟 (仅 DTM 内部跨时钟桥使用)
	always #5 clk_dmi = ~clk_dmi;

	reg clk_sys = 1'b0;      // 辅助时钟: 空闲检测 (12MHz, 与 SoC 一致)
	always #41.667 clk_sys = ~clk_sys;

	reg rst_n = 1'b1;        // 先高, 稍后给一个真正的下降沿 (异步复位需要边沿)
	reg tckc  = 1'b0;

	// TCKC 时序: 仿真取 2MHz (clk_sys=12MHz 的 1/6)
	localparam integer T_SETUP = 250;  // 数据建立 (上升沿之前) + TCKC 低电平宽度
	localparam integer T_HIGH  = 250;  // TCKC 高电平宽度
	localparam integer T_IDLE  = 8000; // 一次"操作之间的空闲" (>= IDLE_CYCLES)

	// ------------------------------------------------------------------
	// TMSC 双向总线: 探针与 DUT 分时驱动, 'x' 表示两者冲突
	// ------------------------------------------------------------------
	tri  tmsc;
	pulldown pd_tmsc (tmsc);          // 模拟 XDC 里的 PULLTYPE PULLDOWN

	reg  probe_oen = 1'b0;
	reg  probe_out = 1'b0;

	wire dut_tmsc_oen, dut_tmsc_out;

	assign tmsc = probe_oen ? probe_out : 1'bz;
	assign tmsc = dut_tmsc_oen ? dut_tmsc_out : 1'bz;

	// ------------------------------------------------------------------
	// DUT: cjtag2jtag + 4 线 DTM
	// ------------------------------------------------------------------
	wire jtag_tck, jtag_tms, jtag_tdi, jtag_tdo, jtag_trst_n;
	localparam [31:0] TB_IDCODE = 32'h1a2b_3c4d;

`ifdef OLD_MODULE
	cjtag2jtag dut (
`else
	cjtag2jtag #(
		.IDLE_CYCLES (64)
	) dut (
		.clk           (clk_sys),
`endif
		.tckc          (tckc),
		.tmsc_out_en   (dut_tmsc_oen),
		.tmsc_out_data (dut_tmsc_out),
		.tmsc_in_data  (tmsc),
		.rst_n         (rst_n),
		.tck           (jtag_tck),
		.tms            (jtag_tms),
		.tdi           (jtag_tdi),
		.tdo           (jtag_tdo),
		.trst_n        (jtag_trst_n)
	);

	hazard3_jtag_dtm #(
		.IDCODE (TB_IDCODE)
	) dtm (
		.tck              (jtag_tck),
		.trst_n           (jtag_trst_n),
		.tms              (jtag_tms),
		.tdi              (jtag_tdi),
		.tdo              (jtag_tdo),

		.dmihardreset_req (),

		.clk_dmi          (clk_dmi),
		.rst_n_dmi        (rst_n),

		.dmi_psel         (),
		.dmi_penable      (),
		.dmi_pwrite       (),
		.dmi_paddr        (),
		.dmi_pwdata       (),
		.dmi_prdata       (32'h0000_0000),
		.dmi_pready       (1'b0),
		.dmi_pslverr      (1'b0)
	);

	// ------------------------------------------------------------------
	// 检查/统计
	// ------------------------------------------------------------------
	integer errors = 0;

	// 总线交叠检查: 探针与 DUT 在同一个下降沿处交接 (目标驱动<->探针驱动),
	// 仿真里会有一个 delta 周期的重叠, 所以等 1ns 后再看是否为 'x' (真冲突)
	always @(tmsc) begin
		#1;
		if (tmsc === 1'bx) begin
			$display("[%0t] ERROR: TMSC 总线冲突 (探针与 DUT 同时驱动)", $time);
			errors = errors + 1;
		end
	end

	// ------------------------------------------------------------------
	// 探针模型
	// ------------------------------------------------------------------
	task probe_drive(input reg en, input reg val);
		begin
			probe_oen = en;
			probe_out = val;
		end
	endtask

	// 一个 TCKC 拍: 调用前驱动状态已设置好 (上一拍下降沿处生效)
	// sample_en=1 时在上升沿采样 TMSC (= POS_EDGE_IN)
	task tckc_beat(input reg sample_en, output reg sampled);
		begin
			#T_SETUP; tckc = 1'b1;
			if (sample_en) sampled = tmsc;
			#T_HIGH;  tckc = 1'b0;
		end
	endtask

	// 一个完整 JTAG 周期: A 拍 nTDI / B 拍 TMS / C 拍释放+采样
	task probe_jtag_cycle(input reg tms_v, input reg tdi_v, output reg tdo_v);
		reg dummy;
		begin
			probe_drive(1'b1, ~tdi_v);      // A 拍: nTDI = ~TDI
			tckc_beat(1'b0, dummy);
			probe_drive(1'b1, tms_v);       // B 拍: TMS
			tckc_beat(1'b0, dummy);
			probe_drive(1'b0, 1'b0);        // B->C 下降沿处释放 TMSC
			tckc_beat(1'b1, tdo_v);         // C 拍: 上升沿采样 TDO
		end
	endtask

	// TMS 序列 (从当前状态出发, 顺序给出, LSB 先发)
	task jtag_move(input [15:0] tms_seq, input integer n);
		integer k;
		reg dummy;
		begin
			for (k = 0; k < n; k = k + 1)
				probe_jtag_cycle(tms_seq[k], 1'b0, dummy);
		end
	endtask

	// IR 扫描 (5 位, LSB 先出); 从 TLR 之后的 RTI 开始
	task ir_scan(input [4:0] ir_val, output [4:0] ir_out);
		integer k;
		reg dummy;
		begin
			// RTI -> Select-DR(1) -> Select-IR(1) -> Capture-IR(0) -> Shift-IR(0)
			jtag_move(16'b0000_0000_0000_0110, 5);
			for (k = 0; k < 5; k = k + 1) begin
				// 最后一位带 TMS=1 退出到 Exit1-IR
				probe_jtag_cycle((k == 4), ir_val[k], ir_out[k]);
			end
			jtag_move(16'b0000_0000_0000_0001, 2);   // Update-IR(1), RTI(0)
		end
	endtask

	// DR 扫描 (n 位, LSB 先出); 从 RTI 开始
	task dr_scan(input integer n, input [31:0] dr_in, output [31:0] dr_out);
		integer k;
		reg dummy;
		begin
			// RTI -> Select-DR(1) -> Capture-DR(0) -> Shift-DR(0)
			jtag_move(16'b0000_0000_0000_0001, 3);
			for (k = 0; k < n; k = k + 1) begin
				probe_jtag_cycle(((k == n - 1)), dr_in[k], dr_out[k]);
			end
			jtag_move(16'b0000_0000_0000_0001, 2);   // Update-DR(1), RTI(0)
		end
	endtask

	task tap_reset;
		integer k;
		reg dummy;
		begin
			for (k = 0; k < 5; k = k + 1)
				probe_jtag_cycle(1'b1, 1'b0, dummy);   // 5 x TMS=1 -> TLR
			probe_jtag_cycle(1'b0, 1'b0, dummy);       // TLR -> RTI
		end
	endtask

	// 一次完整操作: TLR -> IR(IDCODE) -> DR(IDCODE) -> IR(DTMCS) -> DR(DTMCS)
	task do_full_scan(input [31:0] exp_idcode, input [31:0] exp_dtmcs);
		reg [4:0]  ir_rd;
		reg [31:0] dr_rd;
		begin
			tap_reset();
			ir_scan(5'h01, ir_rd);
			if (ir_rd !== 5'h01) begin
				$display("ERROR: IR 回读 = %02x, 期望 01", ir_rd);
				errors = errors + 1;
			end
			dr_scan(32, 32'h0, dr_rd);
			if (dr_rd !== exp_idcode) begin
				$display("ERROR: IDCODE = %08x, 期望 %08x", dr_rd, exp_idcode);
				errors = errors + 1;
			end
			ir_scan(5'h10, ir_rd);
			dr_scan(32, 32'h0, dr_rd);
			if (dr_rd !== exp_dtmcs) begin
				$display("ERROR: DTMCS = %08x, 期望 %08x", dr_rd, exp_dtmcs);
				errors = errors + 1;
			end
		end
	endtask

	// ------------------------------------------------------------------
	// 主流程
	// ------------------------------------------------------------------
	// DTMCS 低 32 位 = {idle_hint=4, cmderr=0, abits=W_ADDR=ABITS=7, version=1}
	// (见 hdl/debug/dtm/hazard3_jtag_dtm_core.v 的 dtmcs_rdata 定义)
	localparam [31:0] EXP_DTMCS = (32'd4 << 12) | (32'd7 << 4) | 32'd1;

	reg [31:0] dr_rd;
	integer    i;

	initial begin
		$display("=== cjtag2jtag 自检开始 (默认 2 线模式, 无激活序列) ===");

		// 复位 (给一个真正的下降沿 -> 上升沿)
		#100 rst_n = 1'b0;
		#100 rst_n = 1'b1;
		repeat (10) @(posedge clk_dmi);

		// 1) 复位后、探针还没敲 TCKC 之前: 不应该自己驱动 TMSC, TCK 应为低
		if ((dut_tmsc_oen !== 1'b0) || (jtag_tck !== 1'b0)) begin
			$display("ERROR: 复位后未活动就占了总线 (oen=%b tck=%b)", dut_tmsc_oen, jtag_tck);
			errors = errors + 1;
		end else begin
			$display("OK: 复位后未活动时不驱动 TMSC / TCK 保持低");
		end

		// 2) 一次空闲后的完整扫描 (空闲边界对齐)
		#T_IDLE;
		do_full_scan(TB_IDCODE, EXP_DTMCS);
		if (errors == 0)
			$display("OK: 空闲边界对齐后 TLR/IR/IDCODE/DTMCS 扫描正确");

		// 3) 多次操作 (每次之间都有空闲) 都应正确
		for (i = 0; i < 3; i = i + 1) begin
			#T_IDLE;
			tap_reset();
			ir_scan(5'h01, dr_rd[4:0]);
			dr_scan(32, 32'h0, dr_rd);
			if (dr_rd !== TB_IDCODE) begin
				$display("ERROR: 第 %0d 次重复扫描 IDCODE = %08x, 期望 %08x", i, dr_rd, TB_IDCODE);
				errors = errors + 1;
			end
		end
		$display("OK: 重复操作 (空闲分隔) 扫描正确");

		// 4) 失步自愈: 先乱敲半组造成错位, 再等一次空闲, 应能重新对齐
		probe_drive(1'b1, 1'b0);
		tckc_beat(1'b0, dr_rd[0]);                // 只发 A 拍 -> 相位偏 1 拍
		#T_IDLE;                                  // 等一次空闲
		tap_reset();
		ir_scan(5'h01, dr_rd[4:0]);
		dr_scan(32, 32'h0, dr_rd);
		if (dr_rd !== TB_IDCODE) begin
			$display("ERROR: 失步自愈后 IDCODE = %08x, 期望 %08x", dr_rd, TB_IDCODE);
			errors = errors + 1;
		end else begin
			$display("OK: 失步后自愈, IDCODE = %08x", dr_rd);
		end

		if (errors == 0)
			$display("=== 全部通过 (errors=0) ===");
		else
			$display("=== 失败: errors=%0d ===", errors);

		$finish;
	end

	// 超时保护
	initial begin
		#2_000_000;
		$display("ERROR: 仿真超时");
		$finish;
	end

endmodule

`default_nettype wire
