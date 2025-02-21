// SPDX-FileCopyrightText: 2022 Lawrie Griffiths
// SPDX-License-Identifier: BSD-2-Clause

module decoder (
    input  wire [15:0] instr,
    input  wire [ 2:0] sideset_bits,
    input  wire        sideset_enable_bit,
    output wire [ 2:0] op,
    output wire [ 2:0] op1,
    output wire [ 4:0] op2,
    output wire [ 4:0] delay,
    output wire [ 4:0] side_set,
    output wire        sideset_enabled
);

    wire [2:0] delay_bits;
    wire [2:0] side_whole_bits;

    assign side_whole_bits = sideset_enable_bit + sideset_bits;
    assign delay_bits      = 5 - side_whole_bits;

    assign op              = instr[15:13];
    assign op1             = instr[7:5];
    assign op2             = instr[4:0];
    assign delay           = (instr[12:8] << side_whole_bits) >> side_whole_bits;
    assign side_set        = (instr[12:8] << sideset_enable_bit) >> (sideset_enable_bit + delay_bits);
    assign sideset_enabled = sideset_enable_bit && instr[12];

endmodule
