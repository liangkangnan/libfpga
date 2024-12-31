// SPDX-FileCopyrightText: 2022 Lawrie Griffiths
// SPDX-License-Identifier: BSD-2-Clause

module divider (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        restart,
    input  wire [23:0] div,
    input  wire        use_divider,
    output wire        penable,
    output wire        pclk
);

    reg [23:0] div_counter;
    reg        pen;
    reg        old_pen;

    always @ (posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_counter <= 0;
            pen <= 1;
            old_pen <= 0;
        end else if (restart) begin
            div_counter <= 0;
            pen <= 1;
            old_pen <= 0;
        end else begin
            if (use_divider) begin
                old_pen <= pen;
                div_counter <= div_counter + 256;
                if (div_counter >= div - 256) begin
                    div_counter <= div_counter - (div - 256);
                end
                pen <= div_counter < (div >> 1);
            end
        end
    end

    assign penable = pen & ~old_pen;
    assign pclk    = pen;

endmodule
