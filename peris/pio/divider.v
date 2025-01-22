// SPDX-FileCopyrightText: 2022 Lawrie Griffiths
// SPDX-License-Identifier: BSD-2-Clause

module divider (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        restart,
    input  wire [31:0] div,
    input  wire        use_divider,
    output wire        penable,
    output wire        pclk
);

    wire clk_en;

    clkdiv_frac #(
        .W_DIV_INT(24),
        .W_DIV_FRAC(8)
    ) clkdiv (
        .clk      (clk),
        .rst_n    (rst_n || !restart),
        .en       (use_divider),
        .div_int  (div[31:8]),
        .div_frac (div[7:0]),
        .clk_en   (clk_en)
    );

    assign penable = use_divider ? clk_en : 1'b1;
    assign pclk    = penable;

endmodule
