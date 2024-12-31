// SPDX-FileCopyrightText: 2022 Lawrie Griffiths
// SPDX-License-Identifier: BSD-2-Clause

module osr (
    input  wire        clk,
    input  wire        penable,
    input  wire        rst_n,
    input  wire        restart,
    input  wire        stalled,
    input  wire [31:0] din,
    input  wire [ 4:0] shift,
    input  wire        dir,   // 0 - left, 1 - right
    input  wire        set,
    input  wire [ 5:0] set_count,
    input  wire        do_shift,
    output wire [31:0] dout,
    output wire [ 5:0] shift_count
);

    reg [31:0] shift_reg;
    reg [5:0]  count;

    // A shift value of 0 means shift 32
    wire [5:0] shift_val = shift == 0 ? 32 : shift;
    // Calculate the 64-bit value of the shift register after a shift
    wire [63:0] shift64 = dir ? {shift_reg, 32'b0} >> shift_val : {32'b0, shift_reg} << shift_val;
    // Calculate the right-aligned shifted out value
    wire [31:0] shift_out = dir ? (shift64[31:0] >> (32 - shift_val)) : shift64[63:32];
    // Calculate the new shift register value after a shift
    wire [31:0] new_shift = dir ? shift64[63:32] : shift64[31:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg <= 0;
            count <= 32;  // Empty (read to trigger auto-pull)
        end else if (restart) begin
            shift_reg <= 0;
            count <= 32;  // Empty (read to trigger auto-pull)
        end else begin
            if (penable && !stalled) begin
                if (set) begin
                    shift_reg <= din;
                    count <= set_count;
                end else if (do_shift) begin
                    shift_reg <= new_shift;
                    count <= (count + shift_val) > 32 ? 32 : (count + shift_val);
                end
            end
        end
    end

    // The output value is the amount shifted out if do_shift ia active otherwise the current shift register
    assign dout = do_shift ? shift_out : shift_reg;
    assign shift_count = count;

endmodule
