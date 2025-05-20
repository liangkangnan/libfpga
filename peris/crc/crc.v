 /*                                                                      
 Copyright 2025 Blue Liang, liangkangnan@163.com
                                                                         
 Licensed under the Apache License, Version 2.0 (the "License");         
 you may not use this file except in compliance with the License.        
 You may obtain a copy of the License at                                 
                                                                         
     http://www.apache.org/licenses/LICENSE-2.0                          
                                                                         
 Unless required by applicable law or agreed to in writing, software    
 distributed under the License is distributed on an "AS IS" BASIS,       
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and     
 limitations under the License.                                          
 */

module crc #(
	parameter FIFO_DEPTH = 8     // must be power of 2, >= 2
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
    output wire        apbs_pslverr
);

localparam S_IDLE  = 2'd0;
localparam S_START = 2'd1;
localparam S_CAL   = 2'd2;

reg [1:0]   state_d,     state_q;
reg [31:0]  crc_d,       crc_q;
reg [2:0]   bit_count_d, bit_count_q;

wire        enable;
wire [31:0] init_val;
wire [31:0] poly;

wire [7:0]  fifo_wdata;
wire        fifo_wen;
wire [7:0]  fifo_rdata;
reg         fifo_ren;
wire        fifo_full;
wire        fifo_empty;


always @ (*) begin
    state_d = state_q;
    crc_d = crc_q;
    bit_count_d = bit_count_q;

    fifo_ren = 1'b0;

    case (state_q)
        S_IDLE: begin
            if (enable) begin
                if (!fifo_empty) begin
                    state_d = S_START;
                end
            end else begin
                crc_d = init_val;
            end
        end

        S_START: begin
            if (!fifo_empty) begin
                fifo_ren = 1'b1;
                bit_count_d = 3'h0;
                crc_d = crc_q ^ {24'h0, fifo_rdata};
                state_d = S_CAL;
            end
        end

        S_CAL: begin
            bit_count_d = bit_count_q + 1'b1;
            if (crc_q[0]) begin
                crc_d = {1'b0, crc_q[31:1]} ^ poly;
            end else begin
                crc_d = {1'b0, crc_q[31:1]};
            end
            if (bit_count_q == 7) begin
                if (fifo_empty) begin
                    state_d = S_IDLE;
                end else begin
                    state_d = S_START;
                end
            end
        end
    endcase

    if (!enable) begin
        state_d = S_IDLE;
    end
end


always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
        state_q     <= S_IDLE;
		crc_q       <= 32'h0;
        bit_count_q <= 3'h0;
	end else begin
        state_q     <= state_d;
        crc_q       <= crc_d;
        bit_count_q <= bit_count_d;
	end
end


sync_fifo #(
	.DEPTH(FIFO_DEPTH),
	.WIDTH(8)
) datafifo (
	.clk    (clk),
	.rst_n  (rst_n),
	.w_data (fifo_wdata),
	.w_en   (fifo_wen),
	.r_data (fifo_rdata),
	.r_en   (fifo_ren),
	.full   (fifo_full),
	.empty  (fifo_empty)
);

crc_regs regs (
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
	.ctrl_enable_o    (enable),
	.stat_idle_i      (fifo_empty && (state_q == S_IDLE)),
	.stat_fifo_empty_i(fifo_empty),
	.stat_fifo_full_i (fifo_full),
	.init_init_o      (init_val),
	.poly_poly_o      (poly),
	.result_result_i  (crc_q),
	.data_o           (fifo_wdata),
	.data_wen         (fifo_wen)
);

endmodule
