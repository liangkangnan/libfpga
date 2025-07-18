
module box_ave #(
    parameter ADC_WIDTH = 8,                        // ADC Convertor Bit Precision
    parameter LPF_MAX_BITS = 3                      // ADC averager max count = 2 ^ (2 ^ LPF_MAX_BITS - 1)
) (
    input  wire                    clk,
    input  wire                    rst_n,

    input  wire                    raw_data_valid,  // raw_data_in is valid, single pulse
    input  wire [ADC_WIDTH-1:0]	   raw_data_in,		// raw_data input
    input  wire [LPF_MAX_BITS-1:0] lpf_depth_bits,  // 2^lpf_depth_bits is decimation rate of averager

    output wire                    data_out_valid,  // ave_data_out is valid, single pulse
    output reg  [ADC_WIDTH-1:0]    ave_data_out		// ave data output
);

//**********************************************************************
//
//	Internal Wire & Reg Signals
//
//**********************************************************************

wire [2**LPF_MAX_BITS:0] target_count = 1 << lpf_depth_bits;

reg [ADC_WIDTH+LPF_MAX_BITS-1:0]      accum;          // accumulator
reg [2**LPF_MAX_BITS:0]               count;          // decimation count
reg [ADC_WIDTH-1:0]                   raw_data_d1;    // pipeline register

reg sample_d1, sample_d2;                             // pipeline registers
reg result_valid;                                     // accumulator result 'valid'
wire accumulate;                                      // sample rising edge detected
wire latch_result;                                    // latch accumulator result

//***********************************************************************
//
//  Rising Edge Detection and data alignment pipelines
//
//***********************************************************************

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		sample_d1 <= 0;
		sample_d2 <= 0;
        raw_data_d1 <= 0;
		result_valid <= 0;
	end else begin
		sample_d1 <= raw_data_valid;        // capture 'sample' input
		sample_d2 <= sample_d1;             // delay for edge detection
		raw_data_d1 <= raw_data_in; 	    // pipeline 
		result_valid <= latch_result;		// pipeline for alignment with result
	end
end

assign accumulate   = sample_d1 && !sample_d2;	    // 'sample' rising_edge detect
assign latch_result = accumulate && (count == (target_count - 1));	// latch accum. per decimation count

//***********************************************************************
//
//  Accumulator Depth counter
//
//***********************************************************************

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        count <= 0;
    end else begin
        if (accumulate) begin
            count <= count + 1;         // incr. count per each sample
            if (count == target_count - 1) begin
                count <= 0;
            end
        end
    end
end

//***********************************************************************
//
//  Accumulator
//
//***********************************************************************

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		accum <= 0;
	end else begin
        if (accumulate) begin
            if (count == 0) begin               // reset accumulator
    		    accum <= raw_data_d1;           // prime with first value
            end else begin
                accum <= accum + raw_data_d1;   // accumulate
            end
        end
	end	
end

//***********************************************************************
//
//  Latch Result
//
//  ave = (summation of 'n' samples)/'n'  is right shift when 'n' is power of two
//
//***********************************************************************

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
        ave_data_out <= 0;
    end else if (latch_result) begin                  // at end of decimation period...
        ave_data_out <= accum >> lpf_depth_bits;	  // ... save accumulator/n result
    end
end

assign data_out_valid = result_valid;       // output assignment

endmodule
