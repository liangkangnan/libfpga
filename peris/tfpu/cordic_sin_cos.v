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

module cordic_sin_cos (
    input  wire               clk,            // 时钟信号
    input  wire               rst_n,          // 复位信号，低有效
    input  wire               valid_in,       // 输入有效信号
    input  wire signed [15:0] angle_in,       // 输入角度，Q2.14格式
    output reg  signed [15:0] cos_out,        // 输出cos值，Q2.14格式
    output reg  signed [15:0] sin_out,        // 输出sin值，Q2.14格式
    output reg                valid_out       // 输出有效信号
);

    // 参数定义
    localparam ITER_NUM = 16; // 旋转次数

    // 内部信号定义
    reg signed [15:0] x; // x坐标寄存器
    reg signed [15:0] y; // y坐标寄存器
    reg signed [15:0] z; // 角度寄存器

    // 预计算的arctan值表，Q2.14格式
    wire [15:0] atan_table [0:ITER_NUM-1];
    assign atan_table[0] = 16'h3243;
    assign atan_table[1] = 16'h1DAC;
    assign atan_table[2] = 16'h0FAD;
    assign atan_table[3] = 16'h07F5;
    assign atan_table[4] = 16'h03FE;
    assign atan_table[5] = 16'h01FF;
    assign atan_table[6] = 16'h00FF;
    assign atan_table[7] = 16'h007F;
    assign atan_table[8] = 16'h003F;
    assign atan_table[9] = 16'h001F;
    assign atan_table[10] = 16'h000F;
    assign atan_table[11] = 16'h0007;
    assign atan_table[12] = 16'h0003;
    assign atan_table[13] = 16'h0001;
    assign atan_table[14] = 16'h0000;
    assign atan_table[15] = 16'h0000;

    // CORDIC增益补偿因子，Q2.14格式
    localparam [15:0] K = 16'h26DD; // 约等于0.60725 * 2^14

    reg [4:0] iter;
    reg busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 0;
            y <= 0;
            z <= 0;
            cos_out <= 0;
            sin_out <= 0;
            valid_out <= 0;
            iter <= 0;
            busy <= 0;
        end else begin
            if (valid_in) begin
                // 初始化
                x <= K;            // 初始x值为增益因子
                y <= 0;            // 初始y值为0
                z <= angle_in;     // 初始角度为输入角度
                iter <= 0;
                valid_out <= 0;
                busy <= 1;
            end else if (busy) begin
                if (iter < ITER_NUM) begin
                    if (z >= 0) begin
                        x <= x - (y >>> iter);
                        y <= y + (x >>> iter);
                        z <= z - atan_table[iter];
                    end else begin
                        x <= x + (y >>> iter);
                        y <= y - (x >>> iter);
                        z <= z + atan_table[iter];
                    end
                    iter <= iter + 1;
                end else begin
                    busy <= 0;
                    // 输出结果
                    cos_out <= x;
                    sin_out <= y;
                    valid_out <= 1;
                end
            end else begin
                valid_out <= 0;
            end
        end
    end

endmodule
