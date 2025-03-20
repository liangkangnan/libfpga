// SPDX-FileCopyrightText: 2022 Lawrie Griffiths
// SPDX-License-Identifier: BSD-2-Clause

module pc (
    input  wire       clk,
    input  wire       penable,
    input  wire       rst_n,
    input  wire       restart,
    input  wire [4:0] din,
    input  wire       jmp,
    input  wire [4:0] wrap_top,
    input  wire       stalled,
    input  wire [4:0] wrap_bottom,
    output wire [4:0] dout
);

    reg [4:0] index;
    wire [4:0] jmp_addr = din + wrap_bottom;

    assign dout = (penable && !stalled) ? (jmp ? jmp_addr : index == wrap_top ? wrap_bottom : index + 1) : index;

    always @ (posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            index <= 0;
        end else if (restart) begin
            index <= 0;
        end else begin
            if (penable && !stalled) begin
                if (jmp) begin
                    index <= jmp_addr;
                end else begin
                    index <= index == wrap_top ? wrap_bottom : index + 1;
                end
            end
        end
    end

endmodule
