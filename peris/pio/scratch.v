// SPDX-FileCopyrightText: 2022 Lawrie Griffiths
// SPDX-License-Identifier: BSD-2-Clause

module scratch (
    input  wire        clk,
    input  wire        penable,
    input  wire        rst_n,
    input  wire        restart,
    input  wire        stalled,
    input  wire [31:0] din,
    input  wire        set,
    input  wire        dec,
    output wire [31:0] dout
);

    reg [31:0] val;

    always @ (posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            val <= 0;
        end else if (restart) begin
            val <= 0;
        end else begin
            if (penable && !stalled) begin
                if (set) begin
                    val <= din;
                end else if (dec) begin
                    val <= val - 1;
                end
            end
        end
    end

    assign dout = val;

endmodule
