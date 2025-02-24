// SPDX-FileCopyrightText: 2025 Blue liang
// SPDX-License-Identifier: BSD-2-Clause

module peri_reset #(
    parameter MAX_PERIS = 16
) (
	input  wire        clk,
	input  wire        rst_n,

	// APB Port
	input  wire        apbs_psel,
	input  wire        apbs_penable,
	input  wire        apbs_pwrite,
	input  wire [15:0] apbs_paddr,
	input  wire [31:0] apbs_pwdata,
	output wire [31:0] apbs_prdata,
	output wire        apbs_pready,
	output wire        apbs_pslverr,

    output wire [MAX_PERIS-1:0] reset_n_req
);

reg  [MAX_PERIS-1:0] ctrl_reset_reg;
wire [MAX_PERIS-1:0] ctrl_reset_out;
wire                 ctrl_reset_wen;
reg  [MAX_PERIS-1:0] assert_reset;
wire [MAX_PERIS-1:0] assert_done;

assign reset_n_req = assert_done;

integer i;

always @(*) begin
    for (i = 0; i < MAX_PERIS; i = i + 1) begin
        assert_reset[i] = ctrl_reset_wen && ctrl_reset_out[i];
    end
end

generate
    genvar j;

    for (j = 0; j < MAX_PERIS; j = j + 1) begin : rst
        reset_sync #(
            .N_CYCLES(2)
        ) reset_u (
            .clk       (clk),
            .rst_n_in  (!assert_reset[j]),
            .rst_n_out (assert_done[j])
        );
    end
endgenerate

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ctrl_reset_reg <= 0;
    end else begin
        if (ctrl_reset_wen) begin
            ctrl_reset_reg <= ctrl_reset_out;
        end else begin
            for (i = 0; i < MAX_PERIS; i = i + 1) begin
                if (assert_done[i]) begin
                    ctrl_reset_reg[i] <= 0;
                end
            end
        end
    end
end

perireset_regs regs (
    .clk           (clk),
    .rst_n         (rst_n),
    // APB Port
    .apbs_psel     (apbs_psel),
    .apbs_penable  (apbs_penable),
    .apbs_pwrite   (apbs_pwrite),
    .apbs_paddr    (apbs_paddr),
    .apbs_pwdata   (apbs_pwdata),
    .apbs_prdata   (apbs_prdata),
    .apbs_pready   (apbs_pready),
    .apbs_pslverr  (apbs_pslverr),
    // Register interfaces
    .ctrl_reset_i  (ctrl_reset_reg),
    .ctrl_reset_o  (ctrl_reset_out),
    .ctrl_reset_wen(ctrl_reset_wen)
);

endmodule
