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
	wire [31:0]  div             [0:NUM_MACHINES-1];
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
	wire [7:0]  irq_flags_out_write [0:NUM_MACHINES-1];
	wire [2:0]  rx_level            [0:NUM_MACHINES-1];
	wire [2:0]  tx_level            [0:NUM_MACHINES-1];

	wire [NUM_MACHINES-1:0]  mempty;
	wire [NUM_MACHINES-1:0]  mfull;
	wire [NUM_MACHINES-1:0]  mpush;
	wire [NUM_MACHINES-1:0]  mpull;
	wire [NUM_MACHINES-1:0]  tx_full;
	wire [NUM_MACHINES-1:0]  rx_empty;

	wire [NUM_MACHINES-1:0]  txfifo_shadow_mode;
	wire [NUM_MACHINES-1:0]  txfifo_shadow_update;
	wire [NUM_MACHINES-1:0]  txfifo_shadow_update_wen;
	wire [NUM_MACHINES-1:0]  txfifo_shadow_update_state;

	reg  [NUM_MACHINES-1:0]  fjoin_reset_fifo;

	reg  [NUM_MACHINES-1:0]  fjoin_rx_prev;
	reg  [NUM_MACHINES-1:0]  fjoin_tx_prev;
	wire [NUM_MACHINES-1:0]  fjoin_rx;
	wire [NUM_MACHINES-1:0]  fjoin_tx;

	reg [NUM_MACHINES-1:0]  fifo_tx_push;
	reg [NUM_MACHINES-1:0]  fifo_tx_pull;
	reg [NUM_MACHINES-1:0]  fifo_tx_empty;
	reg [NUM_MACHINES-1:0]  fifo_tx_full;
	reg [3:0]   fifo_tx_level [0:NUM_MACHINES-1];
	reg [31:0]  fifo_tx_din   [0:NUM_MACHINES-1];
	reg [31:0]  fifo_tx_dout  [0:NUM_MACHINES-1];

	reg [NUM_MACHINES-1:0]  fifo_rx_push;
	reg [NUM_MACHINES-1:0]  fifo_rx_pull;
	reg [NUM_MACHINES-1:0]  fifo_rx_empty;
	reg [NUM_MACHINES-1:0]  fifo_rx_full;
	reg [3:0]   fifo_rx_level [0:NUM_MACHINES-1];
	reg [31:0]  fifo_rx_din   [0:NUM_MACHINES-1];
	reg [31:0]  fifo_rx_dout  [0:NUM_MACHINES-1];

	wire [1:0] fctrl_txfifo_push_index     [0:NUM_MACHINES-1];
	wire       fctrl_txfifo_push_index_wen [0:NUM_MACHINES-1];
	wire [1:0] fctrl_txfifo_pull_index     [0:NUM_MACHINES-1];
	wire       fctrl_txfifo_pull_index_wen [0:NUM_MACHINES-1];
	wire [1:0] fctrl_rxfifo_push_index     [0:NUM_MACHINES-1];
	wire       fctrl_rxfifo_push_index_wen [0:NUM_MACHINES-1];
	wire [1:0] fctrl_rxfifo_pull_index     [0:NUM_MACHINES-1];
	wire       fctrl_rxfifo_pull_index_wen [0:NUM_MACHINES-1];
	wire       fctrl_txfifo_read_en        [0:NUM_MACHINES-1];
	wire       fctrl_rxfifo_write_en       [0:NUM_MACHINES-1];
	wire [2:0] fctrl_txfifo_data_count     [0:NUM_MACHINES-1];
	wire       fctrl_txfifo_data_count_wen [0:NUM_MACHINES-1];
	wire [2:0] fctrl_rxfifo_data_count     [0:NUM_MACHINES-1];
	wire       fctrl_rxfifo_data_count_wen [0:NUM_MACHINES-1];

	wire [1:0] txfifo_pull_index_out [0:NUM_MACHINES-1];
	wire [1:0] txfifo_push_index_out [0:NUM_MACHINES-1];
	wire [1:0] rxfifo_pull_index_out [0:NUM_MACHINES-1];
	wire [1:0] rxfifo_push_index_out [0:NUM_MACHINES-1];

	reg  [7:0] irq_pending;
	wire [7:0] irq_pending_wdata;
	wire       irq_pending_wen;
	wire       irq_pending_ren;

	wire       irq_inte_sm3_txempty;
	wire       irq_inte_sm2_txempty;
	wire       irq_inte_sm1_txempty;
	wire       irq_inte_sm0_txempty;
	wire       irq_inte_sm3_txnfull;
	wire       irq_inte_sm2_txnfull;
	wire       irq_inte_sm1_txnfull;
	wire       irq_inte_sm0_txnfull;
	wire       irq_inte_sm3_rxnempty;
	wire       irq_inte_sm2_rxnempty;
	wire       irq_inte_sm1_rxnempty;
	wire       irq_inte_sm0_rxnempty;
	wire [7:0] irq_inte_sm_int;

	wire [3:0] irq_inte_txempty = {irq_inte_sm3_txempty, irq_inte_sm2_txempty,
								   irq_inte_sm1_txempty, irq_inte_sm0_txempty};
	wire [3:0] irq_inte_txnfull = {irq_inte_sm3_txnfull, irq_inte_sm2_txnfull,
								   irq_inte_sm1_txnfull, irq_inte_sm0_txnfull};
	wire [3:0] irq_inte_rxnempty = {irq_inte_sm3_rxnempty, irq_inte_sm2_rxnempty,
									irq_inte_sm1_rxnempty, irq_inte_sm0_rxnempty};

	wire       irq_intp_sm3_txempty;
	wire       irq_intp_sm2_txempty;
	wire       irq_intp_sm1_txempty;
	wire       irq_intp_sm0_txempty;
	wire       irq_intp_sm3_txnfull;
	wire       irq_intp_sm2_txnfull;
	wire       irq_intp_sm1_txnfull;
	wire       irq_intp_sm0_txnfull;
	wire       irq_intp_sm3_rxnempty;
	wire       irq_intp_sm2_rxnempty;
	wire       irq_intp_sm1_rxnempty;
	wire       irq_intp_sm0_rxnempty;

	wire [3:0] irq_intp_txempty = {irq_intp_sm3_txempty, irq_intp_sm2_txempty,
								   irq_intp_sm1_txempty, irq_intp_sm0_txempty};
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
	wire [31:0] pins_data_set_data;
	wire        pins_data_set_data_wen;
	wire [31:0] pins_data_clr_data;
	wire        pins_data_clr_data_wen;

	(* mem2reg *) reg [15:0]  curr_instr      [0:NUM_MACHINES-1];

	wire [127:0] fifo_rx_data_out [0:NUM_MACHINES-1];

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
			if (pins_data_set_data_wen) begin
				for (k = 0; k < 32; k = k + 1) begin
					if (pins_data_set_data[k]) begin
						gpio_out[k] <= 1'b1;
					end
				end
			end
			if (pins_data_clr_data_wen) begin
				for (k = 0; k < 32; k = k + 1) begin
					if (pins_data_clr_data[k]) begin
						gpio_out[k] <= 1'b0;
					end
				end
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
					for (k = 0; k < 8; k = k + 1) begin
						// SM write
						if (irq_flags_out_write[i][k]) begin
							irq_pending[k] <= irq_flags_out[i][k];
						end
					end
				end
			end
		end
	end

	// Module IRQ
	always @ (*) begin
		for (i = 0; i < NUM_MACHINES; i = i + 1) begin
			irq_fifo[i] = (irq_inte_txnfull[i]  && irq_intp_txnfull[i]) ||
						  (irq_inte_rxnempty[i] && irq_intp_rxnempty[i]) ||
						  (irq_inte_txempty[i]  && irq_intp_txempty[i]);
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

	always @ (*) begin
		for (i = 0; i < NUM_MACHINES; i = i + 1) begin
			fifo_tx_push[i]  = push[i];
			fifo_rx_push[i]  = mpush[i];
			fifo_tx_pull[i]  = mpull[i];
			fifo_rx_pull[i]  = pull[i];
			fifo_tx_din[i]   = push_data[i];
			fifo_rx_din[i]   = mdout[i];
			fifo_tx_dout[i]  = mdin[i];
			fifo_rx_dout[i]  = pdout[i];
			fifo_tx_empty[i] = mempty[i];
			fifo_rx_empty[i] = rx_empty[i];
			fifo_tx_full[i]  = tx_full[i];
			fifo_rx_full[i]  = mfull[i];
			fifo_tx_level[i] = tx_level[i];
			fifo_rx_level[i] = rx_level[i];

			if (fjoin_tx[i]) begin
				if (tx_full[i]) begin
					fifo_rx_push[i] = push[i];
					fifo_rx_din[i] = push_data[i];
				end
				if (mempty[i]) begin
					fifo_rx_pull[i] = mpull[i];
					fifo_tx_dout[i] = pdout[i];
				end
				fifo_tx_empty[i] = mempty[i] && rx_empty[i];
				fifo_tx_full[i] = tx_full[i] && mfull[i];
				fifo_tx_level[i] = tx_level[i] + rx_level[i];
			end
			if (fjoin_rx[i]) begin
				if (mfull[i]) begin
					fifo_tx_push[i] = mpush[i];
					fifo_tx_din[i] = mdout[i];
				end
				if (rx_empty[i]) begin
					fifo_tx_pull[i] = pull[i];
					fifo_rx_dout[i] = mdin[i];
				end
				fifo_rx_empty[i] = mempty[i] && rx_empty[i];
				fifo_rx_full[i] = tx_full[i] && mfull[i];
				fifo_rx_level[i] = tx_level[i] + rx_level[i];
			end
			if (fctrl_txfifo_read_en[i]) begin
				fifo_tx_pull[i] = pull[i];
				fifo_rx_dout[i] = mdin[i];
				fifo_rx_pull[i] = 0;
			end
			if (fctrl_rxfifo_write_en[i]) begin
				fifo_rx_push[i] = push[i];
				fifo_rx_din[i] = push_data[i];
				fifo_tx_push[i] = 0;
			end
		end
	end

	always @ (posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			for (k = 0; k < NUM_MACHINES; k = k + 1) begin
				fjoin_rx_prev[k] <= 0;
				fjoin_tx_prev[k] <= 0;
			end
		end else begin
			for (k = 0; k < NUM_MACHINES; k = k + 1) begin
				fjoin_rx_prev[k] <= fjoin_rx[k];
				fjoin_tx_prev[k] <= fjoin_tx[k];
			end
		end
	end

	always @ (*) begin
		for (k = 0; k < NUM_MACHINES; k = k + 1) begin
			fjoin_reset_fifo[k] = (fjoin_rx_prev[k] ^ fjoin_rx[k]) || (fjoin_tx_prev[k] ^ fjoin_tx[k]);
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
				.use_divider       (div[j] >= 32'h200),
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
				.din               (fifo_tx_dout[j]),
				.dout              (mdout[j]),
				.pull              (mpull[j]),
				.push              (mpush[j]),
				.pclk              (pclk[j]),
				.empty             (fifo_tx_empty[j]),
				.full              (fifo_rx_full[j])
			);

			fifo fifo_tx (
				.clk                (clk),
				.rst_n              (rst_n),
				.reset              ((clear_txfifo[j] & clear_txfifo_en[j]) || fjoin_reset_fifo[j]),
				.push               (fifo_tx_push[j]),
				.pull               (fifo_tx_pull[j]),
				.shadow_mode        (txfifo_shadow_mode[j]),
				.shadow_update      (txfifo_shadow_update[j] & txfifo_shadow_update_wen[j]),
				.shadow_update_state(txfifo_shadow_update_state[j]),
				.fjoin_tx           (fjoin_tx[j]),
				.is_tx              (1'b1),
				.pull_index_wen     (fctrl_txfifo_pull_index_wen[j]),
				.pull_index_wdata   (fctrl_txfifo_pull_index[j]),
				.push_index_wen     (fctrl_txfifo_push_index_wen[j]),
				.push_index_wdata   (fctrl_txfifo_push_index[j]),
				.count_wen          (fctrl_txfifo_data_count_wen[j]),
				.count_wdata        (fctrl_txfifo_data_count[j]),
				.fdata_in           (fifo_rx_data_out[j]),
				.din                (fifo_tx_din[j]),
				.dout               (mdin[j]),
				.pull_index_out     (txfifo_pull_index_out[j]),
				.push_index_out     (txfifo_push_index_out[j]),
				.empty              (mempty[j]),
				.full               (tx_full[j]),
				.level              (tx_level[j])
			);

			fifo fifo_rx (
				.clk                (clk),
				.rst_n              (rst_n),
				.reset              ((clear_rxfifo[j] & clear_rxfifo_en[j]) || fjoin_reset_fifo[j]),
				.push               (fifo_rx_push[j]),
				.pull               (fifo_rx_pull[j]),
				.shadow_mode        (txfifo_shadow_mode[j]),
				.shadow_update      (1'b0),
				.fjoin_tx           (fjoin_tx[j]),
				.is_tx              (1'b0),
				.pull_index_wen     (fctrl_rxfifo_pull_index_wen[j]),
				.pull_index_wdata   (fctrl_rxfifo_pull_index[j]),
				.push_index_wen     (fctrl_rxfifo_push_index_wen[j]),
				.push_index_wdata   (fctrl_rxfifo_push_index[j]),
				.count_wen          (fctrl_rxfifo_data_count_wen[j]),
				.count_wdata        (fctrl_rxfifo_data_count[j]),
				.fdata_out          (fifo_rx_data_out[j]),
				.din                (fifo_rx_din[j]),
				.dout               (pdout[j]),
				.pull_index_out     (rxfifo_pull_index_out[j]),
				.push_index_out     (rxfifo_push_index_out[j]),
				.full               (mfull[j]),
				.empty              (rx_empty[j]),
				.level              (rx_level[j])
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

		.shiftctrl0_fjoin_rx_o(fjoin_rx[0]),
		.shiftctrl0_fjoin_tx_o(fjoin_tx[0]),
		.shiftctrl1_fjoin_rx_o(fjoin_rx[1]),
		.shiftctrl1_fjoin_tx_o(fjoin_tx[1]),
		.shiftctrl2_fjoin_rx_o(fjoin_rx[2]),
		.shiftctrl2_fjoin_tx_o(fjoin_tx[2]),
		.shiftctrl3_fjoin_rx_o(fjoin_rx[3]),
		.shiftctrl3_fjoin_tx_o(fjoin_tx[3]),

		.irq_inte_sm3_txempty_o(irq_inte_sm3_txempty),
		.irq_inte_sm2_txempty_o(irq_inte_sm2_txempty),
		.irq_inte_sm1_txempty_o(irq_inte_sm1_txempty),
		.irq_inte_sm0_txempty_o(irq_inte_sm0_txempty),
		.irq_inte_sm3_txnfull_o(irq_inte_sm3_txnfull),
		.irq_inte_sm2_txnfull_o(irq_inte_sm2_txnfull),
		.irq_inte_sm1_txnfull_o(irq_inte_sm1_txnfull),
		.irq_inte_sm0_txnfull_o(irq_inte_sm0_txnfull),
		.irq_inte_sm3_rxnempty_o(irq_inte_sm3_rxnempty),
		.irq_inte_sm2_rxnempty_o(irq_inte_sm2_rxnempty),
		.irq_inte_sm1_rxnempty_o(irq_inte_sm1_rxnempty),
		.irq_inte_sm0_rxnempty_o(irq_inte_sm0_rxnempty),
		.irq_inte_sm_int_o(irq_inte_sm_int),
		.irq_intp_sm3_txempty_i(fifo_tx_empty[3]),
		.irq_intp_sm3_txempty_o(irq_intp_sm3_txempty),
		.irq_intp_sm2_txempty_i(fifo_tx_empty[2]),
		.irq_intp_sm2_txempty_o(irq_intp_sm2_txempty),
		.irq_intp_sm1_txempty_i(fifo_tx_empty[1]),
		.irq_intp_sm1_txempty_o(irq_intp_sm1_txempty),
		.irq_intp_sm0_txempty_i(fifo_tx_empty[0]),
		.irq_intp_sm0_txempty_o(irq_intp_sm0_txempty),
		.irq_intp_sm3_txnfull_i(!fifo_tx_full[3]),
		.irq_intp_sm3_txnfull_o(irq_intp_sm3_txnfull),
		.irq_intp_sm2_txnfull_i(!fifo_tx_full[2]),
		.irq_intp_sm2_txnfull_o(irq_intp_sm2_txnfull),
		.irq_intp_sm1_txnfull_i(!fifo_tx_full[1]),
		.irq_intp_sm1_txnfull_o(irq_intp_sm1_txnfull),
		.irq_intp_sm0_txnfull_i(!fifo_tx_full[0]),
		.irq_intp_sm0_txnfull_o(irq_intp_sm0_txnfull),
		.irq_intp_sm3_rxnempty_i(!fifo_rx_empty[3]),
		.irq_intp_sm3_rxnempty_o(irq_intp_sm3_rxnempty),
		.irq_intp_sm2_rxnempty_i(!fifo_rx_empty[2]),
		.irq_intp_sm2_rxnempty_o(irq_intp_sm2_rxnempty),
		.irq_intp_sm1_rxnempty_i(!fifo_rx_empty[1]),
		.irq_intp_sm1_rxnempty_o(irq_intp_sm1_rxnempty),
		.irq_intp_sm0_rxnempty_i(!fifo_rx_empty[0]),
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

		.pins_data_set_data_o(pins_data_set_data),
		.pins_data_set_data_wen(pins_data_set_data_wen),
		.pins_data_clr_data_o(pins_data_clr_data),
		.pins_data_clr_data_wen(pins_data_clr_data_wen),

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
		.shiftctrl0_txfifo_shadow_mode_o(txfifo_shadow_mode[0]),
		.shiftctrl0_txfifo_shadow_update_i(txfifo_shadow_update_state[0]),
		.shiftctrl0_txfifo_shadow_update_o(txfifo_shadow_update[0]),
		.shiftctrl0_txfifo_shadow_update_wen(txfifo_shadow_update_wen[0]),
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
		.shiftctrl1_txfifo_shadow_mode_o(txfifo_shadow_mode[1]),
		.shiftctrl1_txfifo_shadow_update_i(txfifo_shadow_update_state[1]),
		.shiftctrl1_txfifo_shadow_update_o(txfifo_shadow_update[1]),
		.shiftctrl1_txfifo_shadow_update_wen(txfifo_shadow_update_wen[1]),
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
		.shiftctrl2_txfifo_shadow_mode_o(txfifo_shadow_mode[2]),
		.shiftctrl2_txfifo_shadow_update_i(txfifo_shadow_update_state[2]),
		.shiftctrl2_txfifo_shadow_update_o(txfifo_shadow_update[2]),
		.shiftctrl2_txfifo_shadow_update_wen(txfifo_shadow_update_wen[2]),
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
		.shiftctrl3_txfifo_shadow_mode_o(txfifo_shadow_mode[3]),
		.shiftctrl3_txfifo_shadow_update_i(txfifo_shadow_update_state[3]),
		.shiftctrl3_txfifo_shadow_update_o(txfifo_shadow_update[3]),
		.shiftctrl3_txfifo_shadow_update_wen(txfifo_shadow_update_wen[3]),

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

		.pull0_i(fifo_rx_dout[0]),
		.pull0_ren(pull[0]),
		.pull1_i(fifo_rx_dout[1]),
		.pull1_ren(pull[1]),
		.pull2_i(fifo_rx_dout[2]),
		.pull2_ren(pull[2]),
		.pull3_i(fifo_rx_dout[3]),
		.pull3_ren(pull[3]),

		.fstat0_txlevel_i(fifo_tx_level[0]),
		.fstat0_txfull_i(fifo_tx_full[0]),
		.fstat0_txempty_i(fifo_tx_empty[0]),
		.fstat0_rxlevel_i(fifo_rx_level[0]),
		.fstat0_rxfull_i(fifo_rx_full[0]),
		.fstat0_rxempty_i(fifo_rx_empty[0]),
		.fstat1_txlevel_i(fifo_tx_level[1]),
		.fstat1_txfull_i(fifo_tx_full[1]),
		.fstat1_txempty_i(fifo_tx_empty[1]),
		.fstat1_rxlevel_i(fifo_rx_level[1]),
		.fstat1_rxfull_i(fifo_rx_full[1]),
		.fstat1_rxempty_i(fifo_rx_empty[1]),
		.fstat2_txlevel_i(fifo_tx_level[2]),
		.fstat2_txfull_i(fifo_tx_full[2]),
		.fstat2_txempty_i(fifo_tx_empty[2]),
		.fstat2_rxlevel_i(fifo_rx_level[2]),
		.fstat2_rxfull_i(fifo_rx_full[2]),
		.fstat2_rxempty_i(fifo_rx_empty[2]),
		.fstat3_txlevel_i(fifo_tx_level[3]),
		.fstat3_txfull_i(fifo_tx_full[3]),
		.fstat3_txempty_i(fifo_tx_empty[3]),
		.fstat3_rxlevel_i(fifo_rx_level[3]),
		.fstat3_rxfull_i(fifo_rx_full[3]),
		.fstat3_rxempty_i(fifo_rx_empty[3]),

		.fctrl0_txfifo_push_index_i(txfifo_push_index_out[0]),
		.fctrl0_txfifo_push_index_o(fctrl_txfifo_push_index[0]),
		.fctrl0_txfifo_push_index_wen(fctrl_txfifo_push_index_wen[0]),
		.fctrl0_txfifo_pull_index_i(txfifo_pull_index_out[0]),
		.fctrl0_txfifo_pull_index_o(fctrl_txfifo_pull_index[0]),
		.fctrl0_txfifo_pull_index_wen(fctrl_txfifo_pull_index_wen[0]),
		.fctrl0_txfifo_read_en_o(fctrl_txfifo_read_en[0]),
		.fctrl0_rxfifo_push_index_i(rxfifo_push_index_out[0]),
		.fctrl0_rxfifo_push_index_o(fctrl_rxfifo_push_index[0]),
		.fctrl0_rxfifo_push_index_wen(fctrl_rxfifo_push_index_wen[0]),
		.fctrl0_rxfifo_pull_index_i(rxfifo_pull_index_out[0]),
		.fctrl0_rxfifo_pull_index_o(fctrl_rxfifo_pull_index[0]),
		.fctrl0_rxfifo_pull_index_wen(fctrl_rxfifo_pull_index_wen[0]),
		.fctrl0_rxfifo_write_en_o(fctrl_rxfifo_write_en[0]),
		.fctrl0_tx_fifo_data_count_i(tx_level[0]),
		.fctrl0_tx_fifo_data_count_o(fctrl_txfifo_data_count[0]),
		.fctrl0_tx_fifo_data_count_wen(fctrl_txfifo_data_count_wen[0]),
		.fctrl0_rx_fifo_data_count_i(rx_level[0]),
		.fctrl0_rx_fifo_data_count_o(fctrl_rxfifo_data_count[0]),
		.fctrl0_rx_fifo_data_count_wen(fctrl_rxfifo_data_count_wen[0]),

		.fctrl1_txfifo_push_index_i(txfifo_push_index_out[1]),
		.fctrl1_txfifo_push_index_o(fctrl_txfifo_push_index[1]),
		.fctrl1_txfifo_push_index_wen(fctrl_txfifo_push_index_wen[1]),
		.fctrl1_txfifo_pull_index_i(txfifo_pull_index_out[1]),
		.fctrl1_txfifo_pull_index_o(fctrl_txfifo_pull_index[1]),
		.fctrl1_txfifo_pull_index_wen(fctrl_txfifo_pull_index_wen[1]),
		.fctrl1_txfifo_read_en_o(fctrl_txfifo_read_en[1]),
		.fctrl1_rxfifo_push_index_i(rxfifo_push_index_out[1]),
		.fctrl1_rxfifo_push_index_o(fctrl_rxfifo_push_index[1]),
		.fctrl1_rxfifo_push_index_wen(fctrl_rxfifo_push_index_wen[1]),
		.fctrl1_rxfifo_pull_index_i(rxfifo_pull_index_out[1]),
		.fctrl1_rxfifo_pull_index_o(fctrl_rxfifo_pull_index[1]),
		.fctrl1_rxfifo_pull_index_wen(fctrl_rxfifo_pull_index_wen[1]),
		.fctrl1_rxfifo_write_en_o(fctrl_rxfifo_write_en[1]),
		.fctrl1_tx_fifo_data_count_i(tx_level[1]),
		.fctrl1_tx_fifo_data_count_o(fctrl_txfifo_data_count[1]),
		.fctrl1_tx_fifo_data_count_wen(fctrl_txfifo_data_count_wen[1]),
		.fctrl1_rx_fifo_data_count_i(rx_level[1]),
		.fctrl1_rx_fifo_data_count_o(fctrl_rxfifo_data_count[1]),
		.fctrl1_rx_fifo_data_count_wen(fctrl_rxfifo_data_count_wen[1]),

		.fctrl2_txfifo_push_index_i(txfifo_push_index_out[2]),
		.fctrl2_txfifo_push_index_o(fctrl_txfifo_push_index[2]),
		.fctrl2_txfifo_push_index_wen(fctrl_txfifo_push_index_wen[2]),
		.fctrl2_txfifo_pull_index_i(txfifo_pull_index_out[2]),
		.fctrl2_txfifo_pull_index_o(fctrl_txfifo_pull_index[2]),
		.fctrl2_txfifo_pull_index_wen(fctrl_txfifo_pull_index_wen[2]),
		.fctrl2_txfifo_read_en_o(fctrl_txfifo_read_en[2]),
		.fctrl2_rxfifo_push_index_i(rxfifo_push_index_out[2]),
		.fctrl2_rxfifo_push_index_o(fctrl_rxfifo_push_index[2]),
		.fctrl2_rxfifo_push_index_wen(fctrl_rxfifo_push_index_wen[2]),
		.fctrl2_rxfifo_pull_index_i(rxfifo_pull_index_out[2]),
		.fctrl2_rxfifo_pull_index_o(fctrl_rxfifo_pull_index[2]),
		.fctrl2_rxfifo_pull_index_wen(fctrl_rxfifo_pull_index_wen[2]),
		.fctrl2_rxfifo_write_en_o(fctrl_rxfifo_write_en[2]),
		.fctrl2_tx_fifo_data_count_i(tx_level[2]),
		.fctrl2_tx_fifo_data_count_o(fctrl_txfifo_data_count[2]),
		.fctrl2_tx_fifo_data_count_wen(fctrl_txfifo_data_count_wen[2]),
		.fctrl2_rx_fifo_data_count_i(rx_level[2]),
		.fctrl2_rx_fifo_data_count_o(fctrl_rxfifo_data_count[2]),
		.fctrl2_rx_fifo_data_count_wen(fctrl_rxfifo_data_count_wen[2]),

		.fctrl3_txfifo_push_index_i(txfifo_push_index_out[3]),
		.fctrl3_txfifo_push_index_o(fctrl_txfifo_push_index[3]),
		.fctrl3_txfifo_push_index_wen(fctrl_txfifo_push_index_wen[3]),
		.fctrl3_txfifo_pull_index_i(txfifo_pull_index_out[3]),
		.fctrl3_txfifo_pull_index_o(fctrl_txfifo_pull_index[3]),
		.fctrl3_txfifo_pull_index_wen(fctrl_txfifo_pull_index_wen[3]),
		.fctrl3_txfifo_read_en_o(fctrl_txfifo_read_en[3]),
		.fctrl3_rxfifo_push_index_i(rxfifo_push_index_out[3]),
		.fctrl3_rxfifo_push_index_o(fctrl_rxfifo_push_index[3]),
		.fctrl3_rxfifo_push_index_wen(fctrl_rxfifo_push_index_wen[3]),
		.fctrl3_rxfifo_pull_index_i(rxfifo_pull_index_out[3]),
		.fctrl3_rxfifo_pull_index_o(fctrl_rxfifo_pull_index[3]),
		.fctrl3_rxfifo_pull_index_wen(fctrl_rxfifo_pull_index_wen[3]),
		.fctrl3_rxfifo_write_en_o(fctrl_rxfifo_write_en[3]),
		.fctrl3_tx_fifo_data_count_i(tx_level[3]),
		.fctrl3_tx_fifo_data_count_o(fctrl_txfifo_data_count[3]),
		.fctrl3_tx_fifo_data_count_wen(fctrl_txfifo_data_count_wen[3]),
		.fctrl3_rx_fifo_data_count_i(rx_level[3]),
		.fctrl3_rx_fifo_data_count_o(fctrl_rxfifo_data_count[3]),
		.fctrl3_rx_fifo_data_count_wen(fctrl_rxfifo_data_count_wen[3]),

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
