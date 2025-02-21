// SPDX-FileCopyrightText: 2022 Lawrie Griffiths
// SPDX-License-Identifier: BSD-2-Clause

module fifo #(
    parameter WIDTH = 32,
    parameter COUNT = 4,
    parameter W_LEVEL = COUNT == 1 ? 1 : $clog2(COUNT) // do not change
) (
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   reset,
    input  wire                   push,
    input  wire                   pull,
    input  wire                   shadow_mode,
    input  wire                   shadow_update,
    input  wire                   fjoin_tx,
    input  wire                   is_tx,
    input  wire                   pull_index_wen,
    input  wire [W_LEVEL-1:0]     pull_index_wdata,
    input  wire                   push_index_wen,
    input  wire [W_LEVEL-1:0]     push_index_wdata,
    input  wire                   count_wen,
    input  wire [W_LEVEL:0]       count_wdata,
    input  wire [WIDTH-1:0]       din,
    input  wire [WIDTH*COUNT-1:0] fdata_in,
    output reg  [WIDTH*COUNT-1:0] fdata_out,
    output wire [WIDTH-1:0]       dout,
    output wire [W_LEVEL-1:0]     pull_index_out,
    output wire [W_LEVEL-1:0]     push_index_out,
    output wire                   shadow_update_state,
    output wire                   empty,
    output wire                   full,
    output wire [W_LEVEL:0]       level
);

    reg [WIDTH-1:0]    arr    [0:COUNT-1];
    reg [W_LEVEL-1:0]  first;
    reg [W_LEVEL-1:0]  next;
    reg [W_LEVEL:0]    count;
    reg                shadow_update_saved;

    wire shadow_update_point = fjoin_tx ? (first == (COUNT - 1)) :
                                          (first == ((COUNT / 2) - 1));

    wire ignore_empty_and_full = shadow_mode && (((!fjoin_tx) && is_tx) || fjoin_tx);

    wire do_pull = pull && (!empty || ignore_empty_and_full);
    wire do_push = push && (!full  || ignore_empty_and_full);

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
            if (pull_index_wen) begin
                first <= pull_index_wdata;
            end
            if (push_index_wen) begin
                next <= push_index_wdata;
            end
            if (count_wen) begin
                count <= count_wdata;
            end
            if (do_push) begin
                next <= next + 1;
                arr[next] <= din;
                if (!do_pull) count <= count + 1;
            end
            if (do_pull) begin
                first <= first + 1;
                if (ignore_empty_and_full) begin
                    // first reach max value
                    if (shadow_update_point) begin
                        first <= 0;
                        // update from shadow regs
                        if (shadow_update_saved) begin
                            if (fjoin_tx) begin
                                // update from rx fifo
                                for (i = 0; i < COUNT; i = i + 1) begin
                                    arr[i] <= fdata_in[i * WIDTH +: WIDTH];
                                end
                            end else begin
                                // update from tx bottom half
                                for (i = 0; i < COUNT / 2; i = i + 1) begin
                                    arr[i] <= arr[(COUNT / 2) + i];
                                end
                            end
                        end
                        shadow_update_saved <= 0;
                    end
                end else begin
                    if (!do_push) count <= count - 1;
                end
            end
            // save but update when first reach max
            if (shadow_mode && shadow_update) begin
                shadow_update_saved <= 1;
            end
        end
    end

    assign shadow_update_state = shadow_update_saved;

    assign empty = ignore_empty_and_full ? 1'b0 : (count == 0);
    assign full  = ignore_empty_and_full ? 1'b0 : (count == COUNT);
    assign dout  = arr[first];
    assign level = ignore_empty_and_full ? ((count >= COUNT) ? COUNT : count) : count;

    assign pull_index_out = first;
    assign push_index_out = next;

    always @ (*) begin
        for (i = 0; i < COUNT; i = i + 1) begin
            fdata_out[i * WIDTH +: WIDTH] = arr[i];
        end
    end

endmodule
