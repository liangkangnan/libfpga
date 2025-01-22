// SPDX-FileCopyrightText: 2022 Lawrie Griffiths
// SPDX-License-Identifier: BSD-2-Clause

module fifo #(
    parameter WIDTH = 32,
    parameter COUNT = 4,
    parameter W_LEVEL = COUNT == 1 ? 1 : $clog2(COUNT) // do not change
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire             reset,
    input  wire             push,
    input  wire             pull,
    input  wire             peek_mode,
    input  wire             shadow_mode,
    input  wire             shadow_update,
    output wire             shadow_update_state,
    input  wire [WIDTH-1:0] din,
    output wire [WIDTH-1:0] dout,
    output wire             empty,
    output wire             full,
    output wire [W_LEVEL:0] level
);

    reg [WIDTH-1:0]    arr    [0:COUNT-1];
    reg [W_LEVEL-1:0]  first;
    reg [W_LEVEL-1:0]  next;
    reg [W_LEVEL:0]    count;
    reg                shadow_update_saved;

    wire do_pull = pull && (!empty || (peek_mode && shadow_mode));
    wire do_push = push && (!full || (peek_mode && shadow_mode));

    integer i;

    always @ (posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            first <= 0;
            next <= 0;
            count <= 0;
            shadow_update_saved <= 0;
        end else if (reset) begin
            first <= 0;
            next <= 0;
            count <= 0;
            shadow_update_saved <= 0;
        end else begin
            if (do_push) begin
                next <= next + 1;
                arr[next] <= din;
                if (!do_pull) count <= count + 1;
            end
            if (do_pull) begin
                first <= first + 1;
                if (peek_mode) begin
                    if (shadow_mode) begin
                        if (first == ((COUNT / 2) - 1)) begin
                            first <= 0;
                            if (shadow_update_saved) begin
                                for (i = 0; i < COUNT / 2; i = i + 1) begin
                                    arr[i] <= arr[(COUNT / 2) + i];
                                end
                                shadow_update_saved <= 0;
                            end
                        end
                    end else begin
                        shadow_update_saved <= 0;
                    end
                end else begin
                    if (!do_push) count <= count - 1;
                end
            end
            if (shadow_mode && shadow_update) begin
                shadow_update_saved <= 1;
            end
        end
    end

    assign shadow_update_state = shadow_update_saved;

    assign empty = (peek_mode && shadow_mode) ? 1'b0 : (count == 0);
    assign full  = (peek_mode && shadow_mode) ? 1'b0 : (count == COUNT);
    assign dout  = arr[first];
    assign level = count;

endmodule
