// SPDX-FileCopyrightText: 2025 Blue liang
// SPDX-License-Identifier: BSD-2-Clause

module pio_multi #(
	parameter NUM_PIO = 2
) (
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire [NUM_PIO-1:0]    apbs_psel,
    input  wire [NUM_PIO-1:0]    apbs_penable,
    input  wire [NUM_PIO-1:0]    apbs_pwrite,
    input  wire [NUM_PIO*16-1:0] apbs_paddr,
    input  wire [NUM_PIO*32-1:0] apbs_pwdata,
    output wire [NUM_PIO*32-1:0] apbs_prdata,
    output wire [NUM_PIO-1:0]    apbs_pready,
    output wire [NUM_PIO-1:0]    apbs_pslverr,

    input  wire [31:0]           gpio_in,
    output reg  [31:0]           gpio_out,
    output reg  [31:0]           gpio_dir,
    output wire [NUM_PIO-1:0]    irq
);

    reg  [31:0]  gpio_out_prev  [0:NUM_PIO-1];
    reg  [31:0]  gpio_dir_prev  [0:NUM_PIO-1];

    reg  [31:0]  pio_gpio_in    [0:NUM_PIO-1];
    wire [31:0]  pio_gpio_out   [0:NUM_PIO-1];
    wire [31:0]  pio_gpio_dir   [0:NUM_PIO-1];

    integer j, k;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gpio_out <= 32'h0;
            gpio_dir <= 32'h0;
            for (j = 0; j < NUM_PIO; j = j + 1) begin
                gpio_out_prev[j] <= 32'h0;
                gpio_dir_prev[j] <= 32'h0;
            end
        end else begin
            for (j = 0; j < NUM_PIO; j = j + 1) begin
                for (k = 0; k < 32; k = k + 1) begin
                    gpio_out_prev[j][k] <= pio_gpio_out[j][k];
                    if (gpio_out_prev[j][k] ^ pio_gpio_out[j][k]) begin
                        gpio_out[k] <= pio_gpio_out[j][k];
                    end

                    gpio_dir_prev[j][k] <= pio_gpio_dir[j][k];
                    if (gpio_dir_prev[j][k] ^ pio_gpio_dir[j][k]) begin
                        gpio_dir[k] <= pio_gpio_dir[j][k];
                    end
                end
            end
        end
    end

    always @(*) begin
        for (j = 0; j < NUM_PIO; j = j + 1) begin
            for (k = 0; k < 32; k = k + 1) begin
                if (gpio_dir[k]) begin
                    pio_gpio_in[j][k] = gpio_out[k];
                end else begin
                    pio_gpio_in[j][k] = gpio_in[k];
                end
            end
        end
    end

    // Generate the pios
    generate
        genvar i;

        for (i = 0; i < NUM_PIO; i = i + 1) begin
            pio pio_u (
                .clk            (clk),
                .rst_n          (rst_n),

                .apbs_psel      (apbs_psel[i]),
                .apbs_penable   (apbs_penable[i]),
                .apbs_pwrite    (apbs_pwrite[i]),
                .apbs_paddr     (apbs_paddr[16 * i +: 16]),
                .apbs_pwdata    (apbs_pwdata[32 * i +: 32]),
                .apbs_prdata    (apbs_prdata[32 * i +: 32]),
                .apbs_pready    (apbs_pready[i]),
                .apbs_pslverr   (apbs_pslverr[i]),

                .gpio_in        (pio_gpio_in[i]),
                .gpio_out       (pio_gpio_out[i]),
                .gpio_dir       (pio_gpio_dir[i]),
                .irq            (irq[i])
            );
        end
    endgenerate

endmodule
