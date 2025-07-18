
module sigmadelta_adc #(
    parameter ADC_WIDTH = 8,              // ADC Convertor Bit Precision
    parameter ACCUM_BITS = 10             // 2^ACCUM_BITS is decimation rate of accumulator
) (
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 cmp_in,
    input  wire [2:0]           lpf_bits,

    output wire                 pwm_out,
    output wire [ADC_WIDTH-1:0] digital_out,
    output wire                 sample_rdy
);            

//**********************************************************************
//
//	Internal Wire & Reg Signals
//
//**********************************************************************
reg                         delta;          // captured comparitor output
reg [ACCUM_BITS-1:0]	    sigma;          // running accumulator value
reg [ADC_WIDTH-1:0]	        accum;          // latched accumulator value
reg [ACCUM_BITS-1:0]	    counter;        // decimation counter for accumulator
reg							rollover;       // decimation counter terminal count
reg							accum_rdy;      // latched accumulator value 'ready' 

//***********************************************************************
//
//  SSD 'Analog' Input - PWM
//
//	External Comparator Generates High/Low Value
//
//***********************************************************************

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        delta <= 0;
    end else begin
        delta <= cmp_in;        // capture comparitor output
    end
end

assign pwm_out = delta;      // feedback to comparitor LPF

//***********************************************************************
//
//  Accumulator Stage
//
//	Adds PWM positive pulses over accumulator period
//
//***********************************************************************

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
        sigma       <= 0;
        accum       <= 0;
        accum_rdy   <= 0;
    end else begin
        if (rollover) begin
            // latch top ADC_WIDTH bits of sigma accumulator (drop LSBs)
            accum <= sigma[ACCUM_BITS-1:ACCUM_BITS-ADC_WIDTH];
            sigma <= delta;         // reset accumulator, prime with current delta value
        end else begin
            if (&sigma != 1'b1) begin   // if not saturated
                sigma <= sigma + delta; // accumulate
            end
        end
        accum_rdy <= rollover;     // latch 'rdy' (to align with accum)
    end
end

//***********************************************************************
//
//  Box filter Average
//
//	Acts as simple decimating Low-Pass Filter
//
//***********************************************************************

box_ave #(
    .ADC_WIDTH(ADC_WIDTH),
    .LPF_MAX_BITS(3)
) box_ave (
    .clk(clk),
    .rst_n(rst_n),
    .raw_data_valid(accum_rdy),
    .raw_data_in(accum),
    .lpf_depth_bits(lpf_bits),
    .ave_data_out(digital_out),
    .data_out_valid(sample_rdy)
);

//************************************************************************
//
// Sample Control - Accumulator Timing
//	
//************************************************************************

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        counter <= 0;
        rollover <= 0;
    end else begin
        counter <= counter + 1;       // running count
        rollover <= &counter;         // assert 'rollover' when counter is all 1's
    end
end

endmodule
