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

// 整数开平方
module sqrt #(
    parameter N = 32  // 输入数据的位宽 (必须是偶数)
) (
    input  wire                  clk,      // 时钟
    input  wire                  rst_n,    // 异步复位 (低电平有效)
    input  wire                  start,    // 开始计算信号
    input  wire [N-1:0]          in,       // 被开方数

    output reg                   done,     // 计算完成信号
    output reg  [N-1:0]          root      // 平方根
);

    // 内部寄存器
    reg [N-1:0]             rem_reg;    // 余数寄存器
    reg [N-1:0]             root_reg;   // 平方根寄存器
    reg [$clog2(N/2):0]     count;      // 迭代计数器
    reg [1:0]               state;      // 状态机状态
    reg [N-1:0]             in_data;    // 输入数据

    // 状态机状态定义
    localparam S_IDLE = 2'd0;
    localparam S_CALC = 2'd1;
    localparam S_DONE = 2'd2;

    wire [N-1:0] rem     = {rem_reg[N-3:0], in_data[N-1], in_data[N-2]};
    wire [N-1:0] divisor = {root_reg[N-3:0], 2'b01};
    wire         diff    = divisor <= rem ? 1'b1 : 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rem_reg   <= 0;
            root_reg  <= 0;
            count     <= 0;
            state     <= S_IDLE;
            done      <= 0;
            root      <= 0;
            in_data   <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin
                        // 开始信号有效，进入计算状态
                        state     <= S_CALC;
                        rem_reg   <= 0;
                        root_reg  <= 0;
                        count     <= 0;
                        in_data   <= in;
                    end
                end

                S_CALC: begin
                    // 每次左移两位
                    in_data <= {in_data[N-3:0], 2'b0};

                    // 迭代计算过程
                    if (diff) begin
                        root_reg <= {root_reg[N-2:0], 1'b1};
                        rem_reg <= rem - divisor;
                    end else begin
                        root_reg <= {root_reg[N-2:0], 1'b0};
                        rem_reg <= rem;
                    end

                    // 计数器加1
                    count <= count + 1;

                    // 检查是否完成所有迭代
                    if (count == (N/2 - 1)) begin
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    root  <= root_reg; // 结果
                    done  <= 1'b1;     // 置位完成信号
                    state <= S_IDLE;   // 返回空闲状态
                end

            endcase
        end
    end

endmodule
