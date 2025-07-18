

module sdadc_top #(
    parameter ADC_WIDTH = 10,              // ADC Convertor Bit Precision
    parameter ACCUM_BITS = 12              // 2^ACCUM_BITS is decimation rate of accumulator
) (
	input  wire clk,
	input  wire rst_n,

	// APB Port
	input  wire apbs_psel,
	input  wire apbs_penable,
	input  wire apbs_pwrite,
	input  wire [15:0] apbs_paddr,
	input  wire [31:0] apbs_pwdata,
	output wire [31:0] apbs_prdata,
	output wire apbs_pready,
	output wire apbs_pslverr,

	input  wire cmp_in,
	output wire pwm_out,
    output wire irq
);

wire rst_fifo_wen;
wire rst_fifo_wdata;
wire int_enable;
wire enable;
wire int_pend_wen;
wire int_pend_wdata;
reg int_pending;
wire [7:0] div;
wire sample_clk_en;

wire [ADC_WIDTH-1:0] digital_out;
wire [2:0] lpf_bits;
wire data_ready;

sigmadelta_adc #(
	.ADC_WIDTH(ADC_WIDTH),
	.ACCUM_BITS(ACCUM_BITS)
) ssd_adc (
    .clk        (clk),
    .rst_n      (rst_n),
    .cmp_in     (cmp_in),
    .lpf_bits   (lpf_bits),
    .pwm_out    (pwm_out),
    .digital_out(digital_out),
    .sample_rdy (data_ready)
);

wire rxfifo_full;
wire rxfifo_empty;
wire rxfifo_wen = !rxfifo_full && data_ready;
wire [15:0] rxfifo_rdata;
wire rxfifo_ren;
wire reset_fifo = rst_fifo_wen && rst_fifo_wdata;

sync_fifo #(
	.DEPTH(4),
	.WIDTH(16)
) rxfifo (
	.clk    (clk),
	.rst_n  (rst_n || (!reset_fifo)),
	.w_data ({{16 - ADC_WIDTH{1'b0}}, digital_out}),
	.w_en   (rxfifo_wen),
	.r_data (rxfifo_rdata),
	.r_en   (rxfifo_ren),
	.full   (rxfifo_full),
	.empty  (rxfifo_empty),
	.level  ()
);

clkdiv_frac #(
	.W_DIV_INT(8),
	.W_DIV_FRAC(2)
) inst_clkdiv_frac (
	.clk      (clk),
	.rst_n    (rst_n),
	.en       (enable),
	.div_int  (div),
	.div_frac (2'b0),
	.clk_en   (sample_clk_en)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        int_pending <= 0;
    end else begin
        // gen pending
        if (!rxfifo_empty && enable) begin
            int_pending <= 1;
        end
        // write 1 to clear
        if (int_pend_wen && int_pend_wdata) begin
            int_pending <= 0;
        end
    end
end

assign irq = enable && int_enable && int_pending;

sdadc_regs regs (
    .clk              (clk),
    .rst_n            (rst_n),

    // APB Port
    .apbs_psel        (apbs_psel),
    .apbs_penable     (apbs_penable),
    .apbs_pwrite      (apbs_pwrite),
    .apbs_paddr       (apbs_paddr),
    .apbs_pwdata      (apbs_pwdata),
    .apbs_prdata      (apbs_prdata),
    .apbs_pready      (apbs_pready),
    .apbs_pslverr     (apbs_pslverr),

    // Register interfaces
    .csr_adc_ava_o    (lpf_bits),
    .csr_rst_fifo_i   (1'b0),
    .csr_rst_fifo_o   (rst_fifo_wdata),
    .csr_rst_fifo_wen (rst_fifo_wen),
    .csr_fifo_full_i  (rxfifo_full),
    .csr_fifo_empty_i (rxfifo_empty),
    .csr_int_pend_i   (int_pending),
    .csr_int_pend_o   (int_pend_wdata),
    .csr_int_pend_wen (int_pend_wen),
    .csr_int_en_o     (int_enable),
    .csr_enable_o     (enable),
	.div_o            (div),
    .data_i           (rxfifo_rdata),
    .data_ren         (rxfifo_ren)
);

endmodule
