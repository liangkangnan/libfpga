/*****************************************************************************\
|              cjtag2jtag -- IEEE 1149.7 (cJTAG) 2线 <-> 4线 JTAG 桥              |
|                                                                             |
|  目标探针: riscv-openocd 的 FTDI cJTAG 模式 (src/jtag/drivers/ftdi.c 的        |
|            "ftdi oscan1_mode on"), 适配器需要能三态驱动 TMSC, 例如            |
|            Olimex ARM-USB-TINY-H + ARM-JTAG-SWD.                            |
|            其 MPSSE 时序模式 = LSB_FIRST | POS_EDGE_IN | NEG_EDGE_OUT,        |
|            即 "下降沿换数据, 上升沿采样".                                      |
|                                                                             |
|  本模块按 "目标默认就工作在 2 线模式" 设计: 不实现 cJTAG 激活(escape/OAC/EC/CP)  |
|  序列, 探针一敲 TCKC 就响应.                                                   |
|                                                                             |
|  线上时序 (1 个 JTAG 周期 = 3 个 TCKC 拍):                                    |
|                                                                             |
|   拍:        |---- A ----------|---- B ----------|---- C ----------|          |
|   探针:       | 驱动 TMSC = ~TDI | 驱动 TMSC = TMS  | 释放 TMSC         |       |
|               | (上一拍↓沿已摆好) | (本拍↓沿摆好)     | 本拍↑沿采样 TDO   |       |
|   目标(本模块):| ↑沿采 nTDI       | ↑沿采 TMS        | 本拍↑沿之前 TMSC  |       |
|               | -> tdi_reg       | -> tms_reg       | 上必须已有 TDO;   |       |
|               |                  |                  | 本拍↓沿前释放 TMSC |       |
|   内部 TCK:   | 0                | 0                | 1                 |       |
|                                                                             |
|  相位 ph 与拍的对应关系 (关键! 错一拍则 TDI/TMS/TDO 全错):                     |
|    PH_NTDI : C 拍(读 TDO 拍). 退出(↑沿)时把 TMSC 取反存入 tdi_reg (标准 nTDI);  |
|              该相位内的 ↓沿释放 TMSC.                                         |
|    PH_TMS  : A 拍(nTDI 拍). 退出(↑沿)时把 TMSC 存入 tms_reg.                  |
|    PH_TDO  : B 拍(TMS 拍).  该相位内的 ↓沿驱动 TMSC = tdo;                    |
|              退出(↑沿)时拉高内部 TCK (TCK 在 C 拍为高).                       |
|                                                                             |
|  相位对齐 (空闲边界对齐):                                                     |
|   * 2 线数据流本身没有帧标志, 只能靠"操作之间的空闲"来定包边界. 模块检测到       |
|     TCKC 电平保持 IDLE_CYCLES 个 clk 周期不变后, 把空闲后的第一个 TCKC ↑沿      |
|     当作新包的第一拍 (A 拍) 来对齐相位, 并显式采该拍的 nTDI.                    |
|   * 复位后计数器停在 PH_NTDI, 所以如果探针在复位释放之后才开始敲钟, 第一个沿      |
|     本来就是 A 拍 -- 空闲边界对齐只是把它变成每次操作都能重新对齐 (自愈).        |
|   * TCKC 长时间不动 (>= IDLE_CYCLES) 时让出 TMSC / 停内部 TCK, 避免在探针做      |
|     其它事情时抢总线; 探针重新开始敲钟后自动恢复.                               |
|                                                                             |
|  频率关系 (adapter speed / TCKC / clk):                                       |
|   * 探针的 adapter speed (kHz) 就是 TCKC 的频率; 1 个 JTAG 周期 = 3 个 TCKC 拍,  |
|     所以内部 TCK = TCKC/3  (adapter speed 1000 -> TCKC 1MHz -> TCK ~333kHz).   |
|   * clk 是辅助时钟 (本工程 = clk_sys = 12MHz), 只用于空闲检测和跨时钟采样,       |
|     它对 TCKC 有两个边界:                                                    |
|       上界: TCKC <= clk/4        -- 3 级同步器要能采到 TCKC 的高低电平         |
|                                     (12MHz 时 <= 3MHz)                       |
|       下界: 半个 TCKC 周期 < IDLE_CYCLES/clk -- 否则正常换数据会被当成空闲      |
|                                     (64/12MHz = 5.33us => TCKC >= 约 200kHz) |
|   * 改了 clk 频率就要按 IDLE_CYCLES ~= 5us x clk(MHz) 重新取值 (7 位, <=127).   |
|   * XDC 里对 tckc 按 250ns (4MHz) 建了时钟约束, 提高 adapter speed 时要同步改.   |
|   * 探针如果仍然发激活序列: 它的 "TCKC 保持高" 窗口约 8~24 个 TCKC 周期, 要被     |
|     当成一次空闲就需要 >= IDLE_CYCLES/clk -- 所以这类探针 adapter speed 建议      |
|     <= 1MHz (1MHz 时窗口 8~24us >= 5.33us).                                 |
|                                                                             |
|  注意: 2 线接口没有 TRST, 本模块 trst_n 直接接 rst_n (与原实现一致).           |
|        探针的复位请求由 TMS 序列 (5xTMS=1) 和 DTM 自身的复位逻辑处理.          |
|        因此按键/上电复位会同时: (a) 清零本模块相位计数器 -- 正有操作在进行时那次   |
|        操作会错位 (探针的数据流不会跟着复位), 下一次操作的空闲边界会重新对齐;     |
|        (b) 复位内部 TAP, 而探针不知道. 恢复办法: 复位后补一次 reset / restart    |
|        openocd, 让探针重发 TAP reset.                                        |
\*****************************************************************************/

`default_nettype none

module cjtag2jtag #(
	// TCKC 电平保持不变 >= IDLE_CYCLES 个 clk 周期 => 认为是一次操作之间的空闲,
	// 空闲后的第一个 TCKC ↑沿作为新包的第一拍 (A 拍)
	parameter integer IDLE_CYCLES = 64
) (
	// 辅助时钟 (空闲检测用; 只做慢速电平采样, 不需与 TCKC 同源)
	input  wire clk,

	// cJTAG 接口 (2 线, 来自调试器)
	input  wire tckc,
	output wire tmsc_out_en,    // 1: 本模块驱动 TMSC
	output wire tmsc_out_data,  // 驱动值 (仅在 tmsc_out_en 有效)
	input  wire tmsc_in_data,   // TMSC 上的实际电平

	// 异步复位 (同时也接到 TAP 的 trst_n)
	input  wire rst_n,

	// JTAG 接口 (4 线, 接内部 TAP)
	output wire tck,
	output wire tms,
	output wire tdi,
	input  wire tdo,

	output wire trst_n
);

	// 包内相位 (含义见文件头说明)
	localparam [1:0] PH_NTDI = 2'd0;   // 对应 C 拍
	localparam [1:0] PH_TMS  = 2'd1;   // 对应 A 拍
	localparam [1:0] PH_TDO  = 2'd2;   // 对应 B 拍

	localparam [6:0] IDLE_CYCLES_C = IDLE_CYCLES[6:0];   // (<=127)

	reg [1:0] ph, ph_next;
	reg       tms_reg, tdi_reg, tck_reg;
	reg       tmsc_out_reg, tmsc_oen_reg;

	// ------------------------------------------------------------------
	// 空闲检测 (clk 域): TCKC 电平保持不变 >= IDLE_CYCLES 个 clk 周期
	//   tckc_s2/s3 是同步 + 延迟, 用它们的差做边沿检测
	// ------------------------------------------------------------------
	reg tckc_s1, tckc_s2, tckc_s3;
	reg [6:0] idle_cnt;
	reg       idle_level;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			tckc_s1 <= 1'b0;
			tckc_s2 <= 1'b0;
			tckc_s3 <= 1'b0;
		end else begin
			tckc_s1 <= tckc;
			tckc_s2 <= tckc_s1;
			tckc_s3 <= tckc_s2;
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			idle_cnt   <= 7'd0;
			idle_level <= 1'b0;
		end else if (tckc_s2 != tckc_s3) begin
			// TCKC 有跳变 => 探针在敲钟
			idle_cnt   <= 7'd0;
			idle_level <= 1'b0;
		end else if (idle_cnt != IDLE_CYCLES_C) begin
			idle_cnt   <= idle_cnt + 7'd1;
		end else begin
			idle_level <= 1'b1;
		end
	end

	// ------------------------------------------------------------------
	// 相位计数器 (TCKC ↑沿前进)
	// ------------------------------------------------------------------
	reg  resync_done;                               // 本次空闲已经对齐过 (吸收握手延迟)
	wire resync_now = idle_level && !resync_done;   // 本 ↑沿 = 空闲后第一个边沿

	always @(posedge tckc or negedge rst_n) begin
		if (!rst_n)           resync_done <= 1'b0;
		else if (!idle_level) resync_done <= 1'b0;   // 恢复常规活动, 允许下次对齐
		else if (resync_now)  resync_done <= 1'b1;
	end

	always @(*) begin
		case (ph)
			PH_NTDI: ph_next = PH_TMS;    // C -> A
			PH_TMS : ph_next = PH_TDO;    // A -> B
			PH_TDO : ph_next = PH_NTDI;   // B -> C
			default: ph_next = PH_NTDI;
		endcase
	end

	always @(posedge tckc or negedge rst_n) begin
		if (!rst_n)          ph <= PH_NTDI;
		else if (resync_now) ph <= PH_TMS;   // 空闲后第一拍 = A 拍
		else                 ph <= ph_next;
	end

	// ------------------------------------------------------------------
	// TMS / nTDI 采样 (TCKC ↑沿)
	//   采样动作发生在 "退出当前相位" 的边沿上, 此时线上正是该相位对应的位:
	//     PH_TMS  退出 (= A 拍结束): TMSC = TMS       -> tms_reg
	//     PH_NTDI 退出 (= C 拍结束): TMSC = ~TDI      -> tdi_reg
	//   空闲后第一个边沿上相位是"强制"的, 所以显式采 nTDI (同步时两者等价)
	// ------------------------------------------------------------------
	always @(posedge tckc or negedge rst_n) begin
		if (!rst_n) begin
			tms_reg <= 1'b1;   // JTAG 规范: 复位时 TMS 保持高电平
			tdi_reg <= 1'b0;
		end else if (resync_now) begin
			tdi_reg <= ~tmsc_in_data;
		end else begin
			case (ph)
				PH_TMS : tms_reg <= tmsc_in_data;
				PH_NTDI: tdi_reg <= ~tmsc_in_data;
				default: ;
			endcase
		end
	end

	// ------------------------------------------------------------------
	// TMSC 输出 (TCKC ↓沿; 半周期时序, 目标侧在 ↓沿驱动/释放)
	//   PH_TDO  的 ↓沿: 驱动 TMSC = tdo (B 拍中, 保证 C 拍 ↑沿前有效)
	//   PH_NTDI 的 ↓沿: 释放 TMSC      (C 拍中, 探针下一拍要接管总线)
	//   TCKC 长时间不动 (空闲) 时强制释放, 避免与探针抢总线
	// ------------------------------------------------------------------
	always @(negedge tckc or posedge idle_level or negedge rst_n) begin
		if (!rst_n) begin
			tmsc_out_reg <= 1'b0;
			tmsc_oen_reg <= 1'b0;
		end else if (idle_level) begin
			tmsc_out_reg <= 1'b0;
			tmsc_oen_reg <= 1'b0;
		end else if (ph == PH_TDO) begin
			tmsc_out_reg <= tdo;
			tmsc_oen_reg <= 1'b1;
		end else begin
			tmsc_out_reg <= 1'b0;
			tmsc_oen_reg <= 1'b0;
		end
	end

	// ------------------------------------------------------------------
	// 内部 TCK: C 拍为高 (TAP 在 ↑沿采 tms/tdi, 在 ↓沿更新 tdo)
	//   空闲后第一个边沿是重新对齐点 (A 拍), 不产生 TCK
	// ------------------------------------------------------------------
	always @(posedge tckc or negedge rst_n) begin
		if (!rst_n) tck_reg <= 1'b0;
		else        tck_reg <= (ph == PH_TDO) && !resync_now;
	end

	assign tmsc_out_en   = tmsc_oen_reg;
	assign tmsc_out_data = tmsc_out_reg;

	assign tck    = tck_reg;
	assign tms    = tms_reg;
	assign tdi    = tdi_reg;
	assign trst_n = rst_n;

endmodule

`default_nettype wire
