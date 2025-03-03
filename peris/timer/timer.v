// Simple timer

module timer (
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
    output wire        apbs_pslverr,

    output wire        irq
);

reg [31:0]  counter;
reg         csr_en_reg;
reg         irq_pending;

wire [31:0] expire_count;
wire        csr_en_wen;
wire        csr_en_wdata;
wire        csr_mode;
wire        clk_en;
wire        csr_int_en;
wire        int_pend_wen;
wire        int_pend_wdata;

wire expired = (counter == expire_count - 1);

always @ (posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        counter <= 32'h0;
    end else begin
        if (clk_en) begin
            counter <= counter + 1'b1;
            if (expired) begin
                if (csr_mode) begin
                    counter <= 32'h0;
                end else begin
                    counter <= counter;
                end
            end
        end
        if (csr_en_wen && csr_en_wdata && !csr_en_reg) begin
            counter <= 32'h0;
        end
    end
end

always @ (posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_en_reg <= 1'b0;
    end else begin
        if (csr_en_wen) begin
            csr_en_reg <= csr_en_wdata;
        end else if (csr_en_reg && expired) begin
            if (!csr_mode) begin
                csr_en_reg <= 1'b0;
            end
        end
    end
end

always @ (posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        irq_pending <= 1'b0;
    end else begin
        if (csr_en_reg && expired) begin
            irq_pending <= 1'b1;
        end
        if (int_pend_wen && int_pend_wdata) begin
            irq_pending <= 1'b0;
        end
    end
end

assign irq = irq_pending && csr_int_en;

wire [15:0]  div_int;

clkdiv_frac #(
    .W_DIV_INT(16),
    .W_DIV_FRAC(4)
) inst_clkdiv_frac (
    .clk      (clk),
    .rst_n    (rst_n),
    .en       (csr_en_reg),
    .div_int  (div_int),
    .div_frac (4'b0),
    .clk_en   (clk_en)
);

timer_regs regs (
    .clk                    (clk),
    .rst_n                  (rst_n),

    // APB Port
    .apbs_psel              (apbs_psel),
    .apbs_penable           (apbs_penable),
    .apbs_pwrite            (apbs_pwrite),
    .apbs_paddr             (apbs_paddr),
    .apbs_pwdata            (apbs_pwdata),
    .apbs_prdata            (apbs_prdata),
    .apbs_pready            (apbs_pready),
    .apbs_pslverr           (apbs_pslverr),

    // Register interfaces
    .csr_en_i               (csr_en_reg),
    .csr_en_o               (csr_en_wdata),
    .csr_en_wen             (csr_en_wen),
    .csr_int_en_o           (csr_int_en),
    .csr_mode_o             (csr_mode),
    .csr_div_o              (div_int),
    .int_pending_pending_i  (irq_pending),
    .int_pending_pending_o  (int_pend_wdata),
    .int_pending_pending_wen(int_pend_wen),
    .expire_count_count_o   (expire_count),
    .current_count_count_i  (counter)
);

endmodule
