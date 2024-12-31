// SPDX-FileCopyrightText: 2022 Lawrie Griffiths
// SPDX-License-Identifier: BSD-2-Clause

module pio #(
	parameter NUM_MACHINES = 4
) (
	input  wire        clk,
	input  wire        rst_n,

	input  wire        apbs_psel,
	input  wire        apbs_penable,
	input  wire        apbs_pwrite,
	input  wire [15:0] apbs_paddr,
	input  wire [31:0] apbs_pwdata,
	output wire [31:0] apbs_prdata,
	output wire        apbs_pready,
	output wire        apbs_pslverr,

	input  wire [31:0] gpio_in,
	output reg  [31:0] gpio_out,
	output reg  [31:0] gpio_dir,
	output wire        irq,
	output wire [NUM_MACHINES-1:0]  pclk
);

	// Shared instructions memory
	wire [15:0] instr [0:31];

	wire [15:0] imm_instr [0:NUM_MACHINES-1];
	wire [31:0] push_data [0:NUM_MACHINES-1];

	wire [NUM_MACHINES-1:0] en;
	wire [NUM_MACHINES-1:0] auto_pull;
	wire [NUM_MACHINES-1:0] auto_push;
	wire [NUM_MACHINES-1:0] sideset_enable_bit;
	wire [NUM_MACHINES-1:0] in_shift_dir;
	wire [NUM_MACHINES-1:0] out_shift_dir;
	wire [NUM_MACHINES-1:0] status_sel;
	wire [NUM_MACHINES-1:0] out_sticky;
	wire [NUM_MACHINES-1:0] inline_out_en;
	wire [NUM_MACHINES-1:0] side_pindir;
	wire [NUM_MACHINES-1:0] exec_stalled;

	wire [NUM_MACHINES-1:0] imm;
	wire [NUM_MACHINES-1:0] push;
	wire [NUM_MACHINES-1:0] pull;
	wire [NUM_MACHINES-1:0] restart;
	wire                    restart_en;
	wire [NUM_MACHINES-1:0] clkdiv_restart;
	wire                    clkdiv_restart_en;
	wire [NUM_MACHINES-1:0] clear_rxfifo;
	wire [NUM_MACHINES-1:0] clear_rxfifo_en;
	wire [NUM_MACHINES-1:0] clear_txfifo;
	wire [NUM_MACHINES-1:0] clear_txfifo_en;

	wire [4:0]   wrap_top        [0:NUM_MACHINES-1];
	wire [4:0]   wrap_bottom     [0:NUM_MACHINES-1];
	wire [23:0]  div             [0:NUM_MACHINES-1];
	wire [4:0]   pins_in_base    [0:NUM_MACHINES-1];
	wire [4:0]   pins_out_base   [0:NUM_MACHINES-1];
	wire [4:0]   pins_set_base   [0:NUM_MACHINES-1];
	wire [4:0]   pins_side_base  [0:NUM_MACHINES-1];
	wire [5:0]   pins_out_count  [0:NUM_MACHINES-1];
	wire [2:0]   pins_set_count  [0:NUM_MACHINES-1];
	wire [2:0]   pins_side_count [0:NUM_MACHINES-1];
	wire [5:0]   isr_threshold   [0:NUM_MACHINES-1];
	wire [5:0]   osr_threshold   [0:NUM_MACHINES-1];
	wire [3:0]   status_n        [0:NUM_MACHINES-1];
	wire [4:0]   out_en_sel      [0:NUM_MACHINES-1];  
	wire [4:0]   jmp_pin         [0:NUM_MACHINES-1];

	// Output from machines and fifos
	wire [31:0] output_pins         [0:NUM_MACHINES-1];
	reg  [31:0] output_pins_prev    [0:NUM_MACHINES-1];
	wire [31:0] pin_directions      [0:NUM_MACHINES-1];
	reg  [31:0] pin_directions_prev [0:NUM_MACHINES-1];
	wire [4:0]  pc                  [0:NUM_MACHINES-1];
	wire [31:0] mdin                [0:NUM_MACHINES-1];
	wire [31:0] mdout               [0:NUM_MACHINES-1];
	wire [31:0] pdout               [0:NUM_MACHINES-1];
	wire [7:0]  irq_flags_out       [0:NUM_MACHINES-1];
	wire        irq_flags_out_write [0:NUM_MACHINES-1];
	wire [2:0]  rx_level            [0:NUM_MACHINES-1];
	wire [2:0]  tx_level            [0:NUM_MACHINES-1];

	wire [NUM_MACHINES-1:0]  mempty;
	wire [NUM_MACHINES-1:0]  mfull;
	wire [NUM_MACHINES-1:0]  mpush;
	wire [NUM_MACHINES-1:0]  mpull;
	wire [NUM_MACHINES-1:0]  tx_full;
	wire [NUM_MACHINES-1:0]  rx_empty;

	reg  [7:0] irq_pending;
	wire [7:0] irq_pending_wdata;
	wire       irq_pending_wen;
	wire       irq_pending_ren;

	wire       irq_inte_sm3_txnfull;
	wire       irq_inte_sm2_txnfull;
	wire       irq_inte_sm1_txnfull;
	wire       irq_inte_sm0_txnfull;
	wire       irq_inte_sm3_rxnempty;
	wire       irq_inte_sm2_rxnempty;
	wire       irq_inte_sm1_rxnempty;
	wire       irq_inte_sm0_rxnempty;
	wire [7:0] irq_inte_sm_int;

	wire [3:0] irq_inte_txnfull = {irq_inte_sm3_txnfull, irq_inte_sm2_txnfull,
								   irq_inte_sm1_txnfull, irq_inte_sm0_txnfull};
	wire [3:0] irq_inte_rxnempty = {irq_inte_sm3_rxnempty, irq_inte_sm2_rxnempty,
									irq_inte_sm1_rxnempty, irq_inte_sm0_rxnempty};

	wire       irq_intp_sm3_txnfull;
	wire       irq_intp_sm2_txnfull;
	wire       irq_intp_sm1_txnfull;
	wire       irq_intp_sm0_txnfull;
	wire       irq_intp_sm3_rxnempty;
	wire       irq_intp_sm2_rxnempty;
	wire       irq_intp_sm1_rxnempty;
	wire       irq_intp_sm0_rxnempty;

	wire [3:0] irq_intp_txnfull = {irq_intp_sm3_txnfull, irq_intp_sm2_txnfull,
								   irq_intp_sm1_txnfull, irq_intp_sm0_txnfull};
	wire [3:0] irq_intp_rxnempty = {irq_intp_sm3_rxnempty, irq_intp_sm2_rxnempty,
									irq_intp_sm1_rxnempty, irq_intp_sm0_rxnempty};

	reg [3:0] irq_fifo;
	reg [7:0] irq_sm;

	wire [31:0] pins_data_wdata;
	wire        pins_data_data_wen;
	wire        pins_data_data_ren;
	wire [31:0] pins_dir_wdata;
	wire        pins_dir_data_wen;
	wire        pins_dir_data_ren;
	reg  [31:0] gpio_data;

	(* mem2reg *) reg [15:0]  curr_instr      [0:NUM_MACHINES-1];

	integer i, k;
	integer gpio_idx;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			for (k = 0; k < NUM_MACHINES; k = k + 1) begin
				output_pins_prev[k] <= 0;
				pin_directions_prev[k] <= 0;
			end
			gpio_out <= 0;
			gpio_dir <= 0;
		end else begin
			for (i = 0; i < NUM_MACHINES; i = i + 1) begin
				curr_instr[i] <= instr[pc[i]];

				// Coalesce output pins, making sure the highest PIO wins
				for (gpio_idx = 0; gpio_idx < 32; gpio_idx = gpio_idx + 1) begin
					output_pins_prev[i][gpio_idx] <= output_pins[i][gpio_idx];
					if (output_pins[i][gpio_idx] != output_pins_prev[i][gpio_idx]) begin
						gpio_out[gpio_idx] <= output_pins[i][gpio_idx];
					end

					pin_directions_prev[i][gpio_idx] <= pin_directions[i][gpio_idx];
					if (pin_directions[i][gpio_idx] != pin_directions_prev[i][gpio_idx]) begin
						gpio_dir[gpio_idx] <= pin_directions[i][gpio_idx];
					end
				end
			end
			if (pins_data_data_wen) begin
				gpio_out <= pins_data_wdata;
			end
			if (pins_dir_data_wen) begin
				gpio_dir <= pins_dir_wdata;
			end
		end
	end

    // State machine IRQ
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			irq_pending <= 0;
		end else begin
			if (irq_pending_wen) begin
				for (i = 0; i < 8; i = i + 1) begin
					// APP write
					if (irq_pending_wdata[i]) begin
						irq_pending[i] <= 0;
					end
				end
			end else begin
				for (i = 0; i < NUM_MACHINES; i = i + 1) begin
					// SM write
					if (irq_flags_out_write[i]) begin
						irq_pending <= irq_flags_out[i];
					end
				end
			end
		end
	end

	// Module IRQ
	always @ (*) begin
		for (i = 0; i < NUM_MACHINES; i = i + 1) begin
			irq_fifo[i] = (irq_inte_txnfull[i] && irq_intp_txnfull[i]) || (irq_inte_rxnempty[i] && irq_intp_rxnempty[i]);
		end
		for (i = 0; i < 8; i = i + 1) begin
			irq_sm[i] = irq_inte_sm_int[i] && irq_pending[i];
		end
	end

	assign irq = (|irq_fifo) || (|irq_sm);

	// GPIO input or output data
	always @ (*) begin
		for (i = 0; i < 32; i = i + 1) begin
			// output
			if (gpio_dir[i]) begin
				gpio_data[i] = gpio_out[i];
			// input
			end else begin
				gpio_data[i] = gpio_in[i];
			end
		end
	end

	// Generate the machines and associated TX and RX fifos
	generate
		genvar j;

		for (j = 0; j < NUM_MACHINES; j = j + 1) begin : mach
			machine machine_u (
				.clk               (clk),
				.rst_n             (rst_n),
				.en                (en[j]),
				.restart           (restart_en & restart[j]),
				.mindex            (j[1:0]),
				.jmp_pin           (jmp_pin[j]),
				.input_pins        (gpio_in),
				.output_pins       (output_pins[j]),
				.pin_directions    (pin_directions[j]),
				.sideset_enable_bit(pins_side_count[j] > 0 ? sideset_enable_bit[j] : 1'b0),
				.in_shift_dir      (in_shift_dir[j]),
				.out_shift_dir     (out_shift_dir[j]),
				.div               (div[j]),
				.use_divider       (div[j] >= 24'h200),
				.instr             (imm[j] ? imm_instr[j] : curr_instr[j]),
				.imm               (imm[j]),
				.wrap_top          (wrap_top[j]),
				.wrap_bottom       (wrap_bottom[j]),
				.pins_out_base     (pins_out_base[j]),
				.pins_out_count    (pins_out_count[j]),
				.pins_set_base     (pins_set_base[j]),
				.pins_set_count    (pins_set_count[j]),
				.pins_in_base      (pins_in_base[j]),
				.pins_side_base    (pins_side_base[j]),
				.pins_side_count   (pins_side_count[j]),
				.auto_pull         (auto_pull[j]),
				.auto_push         (auto_push[j]),
				.isr_threshold     (isr_threshold[j]),
				.osr_threshold     (osr_threshold[j]),
				.side_pindir       (side_pindir[j]),
				.irq_flags_in      (irq_pending),
				.irq_flags_out     (irq_flags_out[j]),
				.irq_flags_out_write(irq_flags_out_write[j]),
				.exec_stalled      (exec_stalled[j]),
				.pc                (pc[j]),
				.din               (mdin[j]),
				.dout              (mdout[j]),
				.pull              (mpull[j]),
				.push              (mpush[j]),
				.pclk              (pclk[j]),
				.empty             (mempty[j]),
				.full              (mfull[j])
			);

			fifo fifo_tx (
				.clk   (clk),
				.rst_n (rst_n),
				.reset (clear_txfifo[j] & clear_txfifo_en[j]),
				.push  (push[j]),
				.pull  (mpull[j]),
				.din   (push_data[j]),
				.dout  (mdin[j]),
				.empty (mempty[j]),
				.full  (tx_full[j]),
				.level (tx_level[j])
			);

			fifo fifo_rx (
				.clk   (clk),
				.rst_n (rst_n),
				.reset (clear_rxfifo[j] & clear_rxfifo_en[j]),
				.push  (mpush[j]),
				.pull  (pull[j]),
				.din   (mdout[j]),
				.dout  (pdout[j]),
				.full  (mfull[j]),
				.empty (rx_empty[j]),
				.level (rx_level[j])
			);
		end
	endgenerate

	pio_regs regs (
		.clk(clk),
		.rst_n(rst_n),

		// APB Port
		.apbs_psel(apbs_psel),
		.apbs_penable(apbs_penable),
		.apbs_pwrite(apbs_pwrite),
		.apbs_paddr(apbs_paddr),
		.apbs_pwdata(apbs_pwdata),
		.apbs_prdata(apbs_prdata),
		.apbs_pready(apbs_pready),
		.apbs_pslverr(apbs_pslverr),

		// Register interfaces
		.ctrl_clkdiv_restart_o(clkdiv_restart),
		.ctrl_clkdiv_restart_wen(clkdiv_restart_en),
		.ctrl_sm_restart_o(restart),
		.ctrl_sm_restart_wen(restart_en),
		.ctrl_sm_enable_o(en),

		.clkdiv0_clkdiv_o(div[0]),
		.clkdiv1_clkdiv_o(div[1]),
		.clkdiv2_clkdiv_o(div[2]),
		.clkdiv3_clkdiv_o(div[3]),

		.irq_inte_sm3_txnfull_o(irq_inte_sm3_txnfull),
		.irq_inte_sm2_txnfull_o(irq_inte_sm2_txnfull),
		.irq_inte_sm1_txnfull_o(irq_inte_sm1_txnfull),
		.irq_inte_sm0_txnfull_o(irq_inte_sm0_txnfull),
		.irq_inte_sm3_rxnempty_o(irq_inte_sm3_rxnempty),
		.irq_inte_sm2_rxnempty_o(irq_inte_sm2_rxnempty),
		.irq_inte_sm1_rxnempty_o(irq_inte_sm1_rxnempty),
		.irq_inte_sm0_rxnempty_o(irq_inte_sm0_rxnempty),
		.irq_inte_sm_int_o(irq_inte_sm_int),
		.irq_intp_sm3_txnfull_i(!tx_full[3]),
		.irq_intp_sm3_txnfull_o(irq_intp_sm3_txnfull),
		.irq_intp_sm2_txnfull_i(!tx_full[2]),
		.irq_intp_sm2_txnfull_o(irq_intp_sm2_txnfull),
		.irq_intp_sm1_txnfull_i(!tx_full[1]),
		.irq_intp_sm1_txnfull_o(irq_intp_sm1_txnfull),
		.irq_intp_sm0_txnfull_i(!tx_full[0]),
		.irq_intp_sm0_txnfull_o(irq_intp_sm0_txnfull),
		.irq_intp_sm3_rxnempty_i(!rx_empty[3]),
		.irq_intp_sm3_rxnempty_o(irq_intp_sm3_rxnempty),
		.irq_intp_sm2_rxnempty_i(!rx_empty[2]),
		.irq_intp_sm2_rxnempty_o(irq_intp_sm2_rxnempty),
		.irq_intp_sm1_rxnempty_i(!rx_empty[1]),
		.irq_intp_sm1_rxnempty_o(irq_intp_sm1_rxnempty),
		.irq_intp_sm0_rxnempty_i(!rx_empty[0]),
		.irq_intp_sm0_rxnempty_o(irq_intp_sm0_rxnempty),
		.irq_intp_sm_int_i(irq_pending),
		.irq_intp_sm_int_o(irq_pending_wdata),
		.irq_intp_sm_int_wen(irq_pending_wen),
		.irq_intp_sm_int_ren(irq_pending_ren),

		.pins_data_data_i(gpio_data),
		.pins_data_data_o(pins_data_wdata),
		.pins_data_data_wen(pins_data_data_wen),
		.pins_data_data_ren(pins_data_data_ren),
		.pins_dir_data_i(gpio_dir),
		.pins_dir_data_o(pins_dir_wdata),
		.pins_dir_data_wen(pins_dir_data_wen),
		.pins_dir_data_ren(pins_dir_data_ren),

		.pinctrl0_out_base_o(pins_out_base[0]),
		.pinctrl0_set_base_o(pins_set_base[0]),
		.pinctrl0_side_base_o(pins_side_base[0]),
		.pinctrl0_in_base_o(pins_in_base[0]),
		.pinctrl0_out_count_o(pins_out_count[0]),
		.pinctrl0_set_count_o(pins_set_count[0]),
		.pinctrl0_side_count_o(pins_side_count[0]),
		.pinctrl1_out_base_o(pins_out_base[1]),
		.pinctrl1_set_base_o(pins_set_base[1]),
		.pinctrl1_side_base_o(pins_side_base[1]),
		.pinctrl1_in_base_o(pins_in_base[1]),
		.pinctrl1_out_count_o(pins_out_count[1]),
		.pinctrl1_set_count_o(pins_set_count[1]),
		.pinctrl1_side_count_o(pins_side_count[1]),
		.pinctrl2_out_base_o(pins_out_base[2]),
		.pinctrl2_set_base_o(pins_set_base[2]),
		.pinctrl2_side_base_o(pins_side_base[2]),
		.pinctrl2_in_base_o(pins_in_base[2]),
		.pinctrl2_out_count_o(pins_out_count[2]),
		.pinctrl2_set_count_o(pins_set_count[2]),
		.pinctrl2_side_count_o(pins_side_count[2]),
		.pinctrl3_out_base_o(pins_out_base[3]),
		.pinctrl3_set_base_o(pins_set_base[3]),
		.pinctrl3_side_base_o(pins_side_base[3]),
		.pinctrl3_in_base_o(pins_in_base[3]),
		.pinctrl3_out_count_o(pins_out_count[3]),
		.pinctrl3_set_count_o(pins_set_count[3]),
		.pinctrl3_side_count_o(pins_side_count[3]),

		.execctrl0_exec_stalled_o(exec_stalled[0]),
		.execctrl0_sideset_en_o(sideset_enable_bit[0]),
		.execctrl0_side_pindir_o(side_pindir[0]),
		.execctrl0_jmp_pin_o(jmp_pin[0]),
		.execctrl0_out_en_sel_o(out_en_sel[0]),
		.execctrl0_inline_out_en_o(inline_out_en[0]),
		.execctrl0_out_sticky_o(out_sticky[0]),
		.execctrl0_wrap_top_o(wrap_top[0]),
		.execctrl0_wrap_bottom_o(wrap_bottom[0]),
		.execctrl0_status_sel_o(status_sel[0]),
		.execctrl0_status_n_o(status_n[0]),
		.execctrl1_exec_stalled_o(exec_stalled[1]),
		.execctrl1_sideset_en_o(sideset_enable_bit[1]),
		.execctrl1_side_pindir_o(side_pindir[1]),
		.execctrl1_jmp_pin_o(jmp_pin[1]),
		.execctrl1_out_en_sel_o(out_en_sel[1]),
		.execctrl1_inline_out_en_o(inline_out_en[1]),
		.execctrl1_out_sticky_o(out_sticky[1]),
		.execctrl1_wrap_top_o(wrap_top[1]),
		.execctrl1_wrap_bottom_o(wrap_bottom[1]),
		.execctrl1_status_sel_o(status_sel[1]),
		.execctrl1_status_n_o(status_n[1]),
		.execctrl2_exec_stalled_o(exec_stalled[2]),
		.execctrl2_sideset_en_o(sideset_enable_bit[2]),
		.execctrl2_side_pindir_o(side_pindir[2]),
		.execctrl2_jmp_pin_o(jmp_pin[2]),
		.execctrl2_out_en_sel_o(out_en_sel[2]),
		.execctrl2_inline_out_en_o(inline_out_en[2]),
		.execctrl2_out_sticky_o(out_sticky[2]),
		.execctrl2_wrap_top_o(wrap_top[2]),
		.execctrl2_wrap_bottom_o(wrap_bottom[2]),
		.execctrl2_status_sel_o(status_sel[2]),
		.execctrl2_status_n_o(status_n[2]),
		.execctrl3_exec_stalled_o(exec_stalled[3]),
		.execctrl3_sideset_en_o(sideset_enable_bit[3]),
		.execctrl3_side_pindir_o(side_pindir[3]),
		.execctrl3_jmp_pin_o(jmp_pin[3]),
		.execctrl3_out_en_sel_o(out_en_sel[3]),
		.execctrl3_inline_out_en_o(inline_out_en[3]),
		.execctrl3_out_sticky_o(out_sticky[3]),
		.execctrl3_wrap_top_o(wrap_top[3]),
		.execctrl3_wrap_bottom_o(wrap_bottom[3]),
		.execctrl3_status_sel_o(status_sel[3]),
		.execctrl3_status_n_o(status_n[3]),

		.shiftctrl0_clear_rxfifo_o(clear_rxfifo[0]),
		.shiftctrl0_clear_rxfifo_wen(clear_rxfifo_en[0]),
		.shiftctrl0_clear_txfifo_o(clear_txfifo[0]),
		.shiftctrl0_clear_txfifo_wen(clear_txfifo_en[0]),
		.shiftctrl0_osr_threshold_o(osr_threshold[0]),
		.shiftctrl0_isr_threshold_o(isr_threshold[0]),
		.shiftctrl0_out_shift_dir_o(out_shift_dir[0]),
		.shiftctrl0_in_shift_dir_o(in_shift_dir[0]),
		.shiftctrl0_auto_pull_o(auto_pull[0]),
		.shiftctrl0_auto_push_o(auto_push[0]),
		.shiftctrl1_clear_rxfifo_o(clear_rxfifo[1]),
		.shiftctrl1_clear_rxfifo_wen(clear_rxfifo_en[1]),
		.shiftctrl1_clear_txfifo_o(clear_txfifo[1]),
		.shiftctrl1_clear_txfifo_wen(clear_txfifo_en[1]),
		.shiftctrl1_osr_threshold_o(osr_threshold[1]),
		.shiftctrl1_isr_threshold_o(isr_threshold[1]),
		.shiftctrl1_out_shift_dir_o(out_shift_dir[1]),
		.shiftctrl1_in_shift_dir_o(in_shift_dir[1]),
		.shiftctrl1_auto_pull_o(auto_pull[1]),
		.shiftctrl1_auto_push_o(auto_push[1]),
		.shiftctrl2_clear_rxfifo_o(clear_rxfifo[2]),
		.shiftctrl2_clear_rxfifo_wen(clear_rxfifo_en[2]),
		.shiftctrl2_clear_txfifo_o(clear_txfifo[2]),
		.shiftctrl2_clear_txfifo_wen(clear_txfifo_en[2]),
		.shiftctrl2_osr_threshold_o(osr_threshold[2]),
		.shiftctrl2_isr_threshold_o(isr_threshold[2]),
		.shiftctrl2_out_shift_dir_o(out_shift_dir[2]),
		.shiftctrl2_in_shift_dir_o(in_shift_dir[2]),
		.shiftctrl2_auto_pull_o(auto_pull[2]),
		.shiftctrl2_auto_push_o(auto_push[2]),
		.shiftctrl3_clear_rxfifo_o(clear_rxfifo[3]),
		.shiftctrl3_clear_rxfifo_wen(clear_rxfifo_en[3]),
		.shiftctrl3_clear_txfifo_o(clear_txfifo[3]),
		.shiftctrl3_clear_txfifo_wen(clear_txfifo_en[3]),
		.shiftctrl3_osr_threshold_o(osr_threshold[3]),
		.shiftctrl3_isr_threshold_o(isr_threshold[3]),
		.shiftctrl3_out_shift_dir_o(out_shift_dir[3]),
		.shiftctrl3_in_shift_dir_o(in_shift_dir[3]),
		.shiftctrl3_auto_pull_o(auto_pull[3]),
		.shiftctrl3_auto_push_o(auto_push[3]),

		.instr0_instr_o(imm_instr[0]),
		.instr0_instr_wen(imm[0]),
		.instr1_instr_o(imm_instr[1]),
		.instr1_instr_wen(imm[1]),
		.instr2_instr_o(imm_instr[2]),
		.instr2_instr_wen(imm[2]),
		.instr3_instr_o(imm_instr[3]),
		.instr3_instr_wen(imm[3]),

		.push0_o(push_data[0]),
		.push0_wen(push[0]),
		.push1_o(push_data[1]),
		.push1_wen(push[1]),
		.push2_o(push_data[2]),
		.push2_wen(push[2]),
		.push3_o(push_data[3]),
		.push3_wen(push[3]),

		.pull0_i(pdout[0]),
		.pull0_ren(pull[0]),
		.pull1_i(pdout[1]),
		.pull1_ren(pull[1]),
		.pull2_i(pdout[2]),
		.pull2_ren(pull[2]),
		.pull3_i(pdout[3]),
		.pull3_ren(pull[3]),

		.fstat0_txlevel_i(tx_level[0]),
		.fstat0_txfull_i(tx_full[0]),
		.fstat0_txempty_i(mempty[0]),
		.fstat0_rxlevel_i(rx_level[0]),
		.fstat0_rxfull_i(mfull[0]),
		.fstat0_rxempty_i(rx_empty[0]),
		.fstat1_txlevel_i(tx_level[1]),
		.fstat1_txfull_i(tx_full[1]),
		.fstat1_txempty_i(mempty[1]),
		.fstat1_rxlevel_i(rx_level[1]),
		.fstat1_rxfull_i(mfull[1]),
		.fstat1_rxempty_i(rx_empty[1]),
		.fstat2_txlevel_i(tx_level[2]),
		.fstat2_txfull_i(tx_full[2]),
		.fstat2_txempty_i(mempty[2]),
		.fstat2_rxlevel_i(rx_level[2]),
		.fstat2_rxfull_i(mfull[2]),
		.fstat2_rxempty_i(rx_empty[2]),
		.fstat3_txlevel_i(tx_level[3]),
		.fstat3_txfull_i(tx_full[3]),
		.fstat3_txempty_i(mempty[3]),
		.fstat3_rxlevel_i(rx_level[3]),
		.fstat3_rxfull_i(mfull[3]),
		.fstat3_rxempty_i(rx_empty[3]),

		.instrmem0_instr_o(instr[0]),
		.instrmem1_instr_o(instr[1]),
		.instrmem2_instr_o(instr[2]),
		.instrmem3_instr_o(instr[3]),
		.instrmem4_instr_o(instr[4]),
		.instrmem5_instr_o(instr[5]),
		.instrmem6_instr_o(instr[6]),
		.instrmem7_instr_o(instr[7]),
		.instrmem8_instr_o(instr[8]),
		.instrmem9_instr_o(instr[9]),
		.instrmem10_instr_o(instr[10]),
		.instrmem11_instr_o(instr[11]),
		.instrmem12_instr_o(instr[12]),
		.instrmem13_instr_o(instr[13]),
		.instrmem14_instr_o(instr[14]),
		.instrmem15_instr_o(instr[15]),
		.instrmem16_instr_o(instr[16]),
		.instrmem17_instr_o(instr[17]),
		.instrmem18_instr_o(instr[18]),
		.instrmem19_instr_o(instr[19]),
		.instrmem20_instr_o(instr[20]),
		.instrmem21_instr_o(instr[21]),
		.instrmem22_instr_o(instr[22]),
		.instrmem23_instr_o(instr[23]),
		.instrmem24_instr_o(instr[24]),
		.instrmem25_instr_o(instr[25]),
		.instrmem26_instr_o(instr[26]),
		.instrmem27_instr_o(instr[27]),
		.instrmem28_instr_o(instr[28]),
		.instrmem29_instr_o(instr[29]),
		.instrmem30_instr_o(instr[30]),
		.instrmem31_instr_o(instr[31])
	);

endmodule
