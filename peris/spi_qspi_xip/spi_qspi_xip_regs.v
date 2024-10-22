/*******************************************************************************
*                          AUTOGENERATED BY REGBLOCK                           *
*                            Do not edit manually.                             *
*          Edit the source file (or regblock utility) and regenerate.          *
*******************************************************************************/

// Block name           : xip
// Bus type             : apb
// Bus data width       : 32
// Bus address width    : 16

module xip_regs (
	input wire clk,
	input wire rst_n,
	
	// APB Port
	input wire apbs_psel,
	input wire apbs_penable,
	input wire apbs_pwrite,
	input wire [15:0] apbs_paddr,
	input wire [31:0] apbs_pwdata,
	output wire [31:0] apbs_prdata,
	output wire apbs_pready,
	output wire apbs_pslverr,
	
	// Register interfaces
	output reg  csr_direct_o,
	input wire  csr_busy_i,
	output reg [7:0] txdata_o,
	output reg txdata_wen,
	input wire [7:0] rxdata_i,
	output reg rxdata_ren,
	output reg  qspi_ctrl_mode_o,
	output reg [2:0] qspi_ctrl_dummy_o
);

// APB adapter
wire [31:0] wdata = apbs_pwdata;
reg [31:0] rdata;
wire wen = apbs_psel && apbs_penable && apbs_pwrite;
wire ren = apbs_psel && apbs_penable && !apbs_pwrite;
wire [15:0] addr = apbs_paddr & 16'hc;
assign apbs_prdata = rdata;
assign apbs_pready = 1'b1;
assign apbs_pslverr = 1'b0;

localparam ADDR_CSR = 0;
localparam ADDR_TXDATA = 4;
localparam ADDR_RXDATA = 8;
localparam ADDR_QSPI_CTRL = 12;

wire __csr_wen = wen && addr == ADDR_CSR;
wire __csr_ren = ren && addr == ADDR_CSR;
wire __txdata_wen = wen && addr == ADDR_TXDATA;
wire __txdata_ren = ren && addr == ADDR_TXDATA;
wire __rxdata_wen = wen && addr == ADDR_RXDATA;
wire __rxdata_ren = ren && addr == ADDR_RXDATA;
wire __qspi_ctrl_wen = wen && addr == ADDR_QSPI_CTRL;
wire __qspi_ctrl_ren = ren && addr == ADDR_QSPI_CTRL;

wire  csr_direct_wdata = wdata[0];
wire  csr_direct_rdata;
wire  csr_busy_wdata = wdata[1];
wire  csr_busy_rdata;
wire [31:0] __csr_rdata = {30'h0, csr_busy_rdata, csr_direct_rdata};
assign csr_direct_rdata = csr_direct_o;
assign csr_busy_rdata = csr_busy_i;

wire [7:0] txdata_wdata = wdata[7:0];
wire [7:0] txdata_rdata;
wire [31:0] __txdata_rdata = {24'h0, txdata_rdata};
assign txdata_rdata = 8'h0;

wire [7:0] rxdata_wdata = wdata[7:0];
wire [7:0] rxdata_rdata;
wire [31:0] __rxdata_rdata = {24'h0, rxdata_rdata};
assign rxdata_rdata = rxdata_i;

wire  qspi_ctrl_mode_wdata = wdata[0];
wire  qspi_ctrl_mode_rdata;
wire [2:0] qspi_ctrl_dummy_wdata = wdata[3:1];
wire [2:0] qspi_ctrl_dummy_rdata;
wire [31:0] __qspi_ctrl_rdata = {28'h0, qspi_ctrl_dummy_rdata, qspi_ctrl_mode_rdata};
assign qspi_ctrl_mode_rdata = qspi_ctrl_mode_o;
assign qspi_ctrl_dummy_rdata = qspi_ctrl_dummy_o;

always @ (*) begin
	case (addr)
		ADDR_CSR: rdata = __csr_rdata;
		ADDR_TXDATA: rdata = __txdata_rdata;
		ADDR_RXDATA: rdata = __rxdata_rdata;
		ADDR_QSPI_CTRL: rdata = __qspi_ctrl_rdata;
		default: rdata = 32'h0;
	endcase
	txdata_wen = __txdata_wen;
	txdata_o = txdata_wdata;
	rxdata_ren = __rxdata_ren;
end

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		csr_direct_o <= 1'h0;
		qspi_ctrl_mode_o <= 1'h0;
		qspi_ctrl_dummy_o <= 3'h0;
	end else begin
		if (__csr_wen)
			csr_direct_o <= csr_direct_wdata;
		if (__qspi_ctrl_wen)
			qspi_ctrl_mode_o <= qspi_ctrl_mode_wdata;
		if (__qspi_ctrl_wen)
			qspi_ctrl_dummy_o <= qspi_ctrl_dummy_wdata;
	end
end

endmodule
