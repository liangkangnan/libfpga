 /*                                                                      
 Copyright 2025 Blue Liang, liangkangnan@163.com
                                                                         
 Licensed under the Apache License, Version 2.0 (the "License");         
 you may not use this file except in compliance with the License.        
 You may obtain a copy of the License at                                 
                                                                         
     http://www.apache.org/licenses/LICENSE-2.0                          
                                                                         
 Unless required by applicable law or agreed to in writing, software    
 distributed under the License is distributed on an "AS IS" BASIS,       
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and     
 limitations under the License.                                          
 */

module tfpu_top (
    input  wire        clk,
    input  wire        rst_n,

    // APB Port
    input  wire        apbs_psel,
    input  wire        apbs_penable,
    input  wire        apbs_pwrite,
    input  wire [15:0] apbs_paddr,
    input  wire [31:0] apbs_pwdata,
    output wire [31:0] apbs_prdata,
    output wire        apbs_pready,
    output wire        apbs_pslverr
);

localparam OP_ADD = 4'd0,
           OP_MUL = 4'd1,
           OP_DIV = 4'd2,
           OP_SIN = 4'd3,
           OP_COS = 4'd4,
           OP_SQRT = 4'd5;

wire [3:0] op;
wire [31:0] op_a;
wire [31:0] op_b;
reg [31:0] res_z;
wire start_wen;
wire start_wdata;

reg busy;

reg add_start;
wire add_done;
wire [31:0] add_res;

reg mul_start;
wire mul_done;
wire [31:0] mul_res;

reg div_start;
wire div_done;
wire [31:0] div_res;

reg cordic_start;
wire cordic_done;
wire [15:0] sin_res;
wire [15:0] cos_res;

reg sqrt_start;
wire sqrt_done;
wire [31:0] sqrt_res;

always @(*) begin
    add_start = 1'b0;
    mul_start = 1'b0;
    div_start = 1'b0;
    cordic_start = 1'b0;
    sqrt_start = 1'b0;

    if (start_wdata && start_wen) begin
        case (op)
            OP_ADD: add_start = 1'b1;
            OP_MUL: mul_start = 1'b1;
            OP_DIV: div_start = 1'b1;
            OP_SIN: cordic_start = 1'b1;
            OP_COS: cordic_start = 1'b1;
            OP_SQRT: sqrt_start = 1'b1;
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        res_z <= 32'h0;
    end else begin
        if (add_done || mul_done || div_done || cordic_done || sqrt_done) begin
            case (op)
                OP_ADD: res_z <= add_res;
                OP_MUL: res_z <= mul_res;
                OP_DIV: res_z <= div_res;
                OP_SIN: res_z <= {16'h0, sin_res};
                OP_COS: res_z <= {16'h0, cos_res};
                OP_SQRT: res_z <= sqrt_res;
            endcase
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        busy <= 1'b0;
    end else begin
        if (start_wdata && start_wen) begin
            busy <= 1'b1;
        end else if (add_done || mul_done || div_done || cordic_done || sqrt_done) begin
            busy <= 1'b0;
        end
    end
end

tfpu_adder add (
    .clk     (clk),
    .rst_n   (rst_n),
    .start   (add_start),
    .input_a (op_a),
    .input_b (op_b),
    .done    (add_done),
    .output_z(add_res)
);

tfpu_multiplier mul (
    .clk     (clk),
    .rst_n   (rst_n),
    .start   (mul_start),
    .input_a (op_a),
    .input_b (op_b),
    .done    (mul_done),
    .output_z(mul_res)
);

tfpu_divider div (
    .clk     (clk),
    .rst_n   (rst_n),
    .start   (div_start),
    .input_a (op_a),
    .input_b (op_b),
    .done    (div_done),
    .output_z(div_res)
);

cordic_sin_cos cordic (
    .clk      (clk),
    .rst_n    (rst_n),
    .valid_in (cordic_start),
    .angle_in (op_a[15:0]),
    .cos_out  (cos_res),
    .sin_out  (sin_res),
    .valid_out(cordic_done)
);

sqrt #(
    .N(32)
) sqrt_u (
    .clk  (clk),
    .rst_n(rst_n),
    .start(sqrt_start),
    .in   (op_a),
    .done (sqrt_done),
    .root (sqrt_res)
);

tfpu_regs regs (
    .clk          (clk),
    .rst_n        (rst_n),

    // APB Port
    .apbs_psel    (apbs_psel),
    .apbs_penable (apbs_penable),
    .apbs_pwrite  (apbs_pwrite),
    .apbs_paddr   (apbs_paddr),
    .apbs_pwdata  (apbs_pwdata),
    .apbs_prdata  (apbs_prdata),
    .apbs_pready  (apbs_pready),
    .apbs_pslverr (apbs_pslverr),

    // Register interfaces
    .csr_op_o     (op),
    .csr_start_i  (busy),
    .csr_start_o  (start_wdata),
    .csr_start_wen(start_wen),
    .op_a_o       (op_a),
    .op_b_o       (op_b),
    .res_z_i      (res_z)
);

endmodule
