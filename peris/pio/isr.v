// SPDX-FileCopyrightText: 2022 Lawrie Griffiths
// SPDX-License-Identifier: BSD-2-Clause

module isr (
    input  wire        clk,
    input  wire        penable,
    input  wire        rst_n,
    input  wire        restart,
    input  wire        stalled,
    input  wire [31:0] din,
    input  wire [ 4:0] shift,
    input  wire        dir,
    input  wire        set,
    input  wire        do_shift,
    input  wire        do_shift_con,
    input  wire [ 5:0] bit_count,
    output wire [31:0] dout,
    output wire [ 5:0] shift_count
);

    reg [31:0] shift_reg;
    reg [5:0]  count;

    // A shift value of 0 means shift 32
    wire [5:0] shift_val = shift == 0 ? 32 : shift;
    // Left align the input value and concatenate it with the shift register to produce a 64-bit value
    wire [63:0] new_shift = dir ? {din, shift_reg} >> shift_val
                            : {shift_reg, din << (32 - shift_val)} << shift_val;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg <= 0;
            count <= 0;  // Empty
        end else if (restart) begin
            shift_reg <= 0;
            count <= 0;  // Empty
        end else begin
            if (penable && !stalled) begin
                if (set) begin
                    shift_reg <= din;
                    count <= bit_count;
                end else if (do_shift_con) begin
                    shift_reg <= dir ? new_shift[31:0] : new_shift[63:32];
                    count <= shift_val;
                end else if (do_shift) begin
                    shift_reg <= dir ? new_shift[31:0] : new_shift[63:32];
                    count <= (count + shift_val) > 32 ? 32 : (count + shift_val);
                end
            end
        end
    end

    assign dout = shift_reg;
    assign shift_count = count;

endmodule
