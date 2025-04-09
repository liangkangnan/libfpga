// DMA Controller with AHB-Lite Master and APB Slave Interface
module dma (
    // AHB-Lite Master Interface
    input  wire         ahbm_clk,
    input  wire         ahbm_rst_n,

	output reg  [31:0]  ahbm_haddr,
	output reg          ahbm_hwrite,
	output reg  [1:0]   ahbm_htrans,
	output reg  [2:0]   ahbm_hsize,
	output reg  [2:0]   ahbm_hburst,
	output reg  [3:0]   ahbm_hprot,
	output reg          ahbm_hmastlock,
	output reg  [7:0]   ahbm_hmaster,
	output reg          ahbm_hexcl,
	input  wire         ahbm_hready,
	input  wire         ahbm_hresp,
	input  wire         ahbm_hexokay,
	output reg  [31:0]  ahbm_hwdata,
	input  wire [31:0]  ahbm_hrdata,

    // APB Slave Interface
    input  wire         apbs_clk,
    input  wire         apbs_rst_n,

    input  wire         apbs_psel,
    input  wire         apbs_penable,
    input  wire         apbs_pwrite,
    input  wire [15:0]  apbs_paddr,
    input  wire [31:0]  apbs_pwdata,
    output wire [31:0]  apbs_prdata,
    output wire         apbs_pready,
    output wire         apbs_pslverr,

    // Interrupt
    output wire         irq
);

localparam IDLE                   = 3'd0;
localparam START                  = 3'd1;
localparam READ_ADDR              = 3'd2;
localparam READ_DATA              = 3'd3;
localparam READ_STATUS_DATA       = 3'd4;
localparam WRITE_STATUS_DATA      = 3'd5;
localparam WRITE_ADDR             = 3'd6;
localparam WRITE_DATA             = 3'd7;

wire [31:0] src_data_addr;
wire [31:0] src_status_addr;
wire [ 5:0] src_ready_bit;
wire        src_ready_polarity;
wire        src_addr_inc;
wire [31:0] dest_data_addr;
wire [31:0] dest_status_addr;
wire [ 5:0] dest_ready_bit;
wire        dest_ready_polarity;
wire        dest_addr_inc;
wire [31:0] tran_count;
wire        start_transfer;
wire        stop_transfer;
wire [ 2:0] data_width;
wire        start_wen;
wire        start_wdata;
wire        half_irq_en;
wire        over_irq_en;
wire        halfover_pend;
wire        over_pend;
wire        count_mode;

assign start_transfer = start_wen && start_wdata;
assign stop_transfer  = start_wen && !start_wdata;
assign irq = (over_irq_en && over_pend) || (half_irq_en && halfover_pend);

// ==============================================
// DMA State Machine
// ==============================================

reg [31:0] transfer_counter_d, transfer_counter_q;
reg [ 2:0] state_d,            state_q;
reg [ 2:0] next_state_d,       next_state_q;
reg [31:0] read_data_d,        read_data_q;
reg [31:0] read_addr_d,        read_addr_q;
reg        over_pend_d,        over_pend_q;
reg        halfover_pend_d,    halfover_pend_q;
reg        transfering_d,      transfering_q;

wire [31:0] src_addr  = src_addr_inc  ? (src_data_addr  + transfer_counter_q) : src_data_addr;
wire [31:0] dest_addr = dest_addr_inc ? (dest_data_addr + transfer_counter_q) : dest_data_addr;
wire [ 2:0] each_tran_count = 1 << data_width;
wire no_read_src_status  = src_ready_bit  >= 32;
wire no_read_dest_status = dest_ready_bit >= 32;
wire transfer_complete = transfer_counter_q == (tran_count - each_tran_count);

task ahb_read (
    input [31:0] addr
);
    begin
        ahbm_htrans = 2'b10;
        ahbm_haddr = addr;
        ahbm_hwrite = 1'b0;
    end
endtask

task ahb_write (
    input [31:0] addr
);
    begin
        ahbm_htrans = 2'b10;
        ahbm_haddr = addr;
        ahbm_hwrite = 1'b1;
    end
endtask

always @ (*) begin
    transfer_counter_d = transfer_counter_q;
    state_d = state_q;
    next_state_d = next_state_q;
    read_data_d = read_data_q;
    read_addr_d = read_addr_q;
    over_pend_d = 1'b0;
    halfover_pend_d = 1'b0;
    transfering_d = transfering_q;

    ahbm_htrans = 2'b00;
    ahbm_haddr = 32'h0;
    ahbm_hwrite = 1'b0;
    ahbm_hsize = data_width;
    ahbm_hburst = 3'h0;
    ahbm_hprot = 4'b0011;
    ahbm_hwdata = read_data_q;
    ahbm_hmastlock = 1'b0;
    ahbm_hmaster = 8'h0;
    ahbm_hexcl = 1'b0;

    if (stop_transfer) begin
        state_d = IDLE;
        transfering_d = 1'b0;
    end

    case (state_q)
        IDLE: begin
            if (start_transfer) begin
                state_d = START;
                transfer_counter_d = 32'h0;
                transfering_d = 1'b1;
            end
        end

        START: begin
            state_d = READ_ADDR;
            // 不需要读src状态（比如src为内存地址）
            if (no_read_src_status) begin
                read_addr_d = src_addr;
                next_state_d = READ_DATA;
            end else begin
                read_addr_d = src_status_addr;
                next_state_d = READ_STATUS_DATA;
            end
        end

        READ_ADDR: begin
            ahb_read(read_addr_q);
            if (ahbm_hresp) begin
                state_d = START;
            end else if (ahbm_hready) begin
                state_d = next_state_q;
            end
        end

        READ_STATUS_DATA: begin
            if (ahbm_hresp) begin
                state_d = START;
            end else if (ahbm_hready) begin
                state_d = READ_ADDR;
                // src ready
                if (ahbm_hrdata[src_ready_bit] == src_ready_polarity) begin
                    read_addr_d = src_addr;
                    next_state_d = READ_DATA;
                end else begin
                    read_addr_d = src_status_addr;
                    next_state_d = READ_STATUS_DATA;
                end
            end
        end

        WRITE_STATUS_DATA: begin
            if (ahbm_hresp) begin
                state_d = START;
            end else if (ahbm_hready) begin
                // dest ready
                if (ahbm_hrdata[dest_ready_bit] == dest_ready_polarity) begin
                    state_d = WRITE_ADDR;
                end else begin
                    state_d = READ_ADDR;
                    read_addr_d = dest_status_addr;
                    next_state_d = WRITE_STATUS_DATA;
                end
            end
        end

        READ_DATA: begin
            if (ahbm_hresp) begin
                state_d = START;
            end else if (ahbm_hready) begin
                read_data_d = ahbm_hrdata;
                // 不需要读dest状态（比如dest为内存地址）
                if (no_read_dest_status) begin
                    state_d = WRITE_ADDR;
                end else begin
                    state_d = READ_ADDR;
                    read_addr_d = dest_status_addr;
                    next_state_d = WRITE_STATUS_DATA;
                end
            end
        end

        WRITE_ADDR: begin
            ahb_write(dest_addr);
            if (ahbm_hresp) begin
                state_d = START;
            end else if (ahbm_hready) begin
                state_d = WRITE_DATA;
            end
        end

        WRITE_DATA: begin
            if (ahbm_hresp) begin
                state_d = START;
            end else if (ahbm_hready) begin
                transfer_counter_d = transfer_counter_q + each_tran_count;
                // 传输完成，循环模式
                if (transfer_complete && count_mode) begin
                    state_d = START;
                    transfer_counter_d = 32'h0;
                    over_pend_d = 1'b1;
                // 传输完成，单次模式
                end else if (transfer_complete && !count_mode) begin
                    state_d = IDLE;
                    transfering_d = 1'b0;
                    over_pend_d = 1'b1;
                end else begin
                    state_d = START;
                end
                // 传输完成一半
                if (transfer_counter_q == ({1'b0, tran_count[31:1]} - each_tran_count)) begin
                    halfover_pend_d = 1'b1;
                end
            end
        end

    endcase
end

always @ (posedge ahbm_clk or negedge ahbm_rst_n) begin
    if (!ahbm_rst_n) begin
        transfer_counter_q <= 32'h0;
        state_q            <= IDLE;
        next_state_q       <= 3'h0;
        read_data_q        <= 32'h0;
        read_addr_q        <= 32'h0;
        over_pend_q        <= 1'b0;
        halfover_pend_q    <= 1'b0;
        transfering_q      <= 1'b0;
    end else begin
        transfer_counter_q <= transfer_counter_d;
        state_q            <= state_d;
        next_state_q       <= next_state_d;
        read_data_q        <= read_data_d;
        read_addr_q        <= read_addr_d;
        over_pend_q        <= over_pend_d;
        halfover_pend_q    <= halfover_pend_d;
        transfering_q      <= transfering_d;
    end
end

// regs
dma_regs regs (
    .clk                        (apbs_clk),
    .rst_n                      (apbs_rst_n),

    // APB Port
    .apbs_psel                  (apbs_psel),
    .apbs_penable               (apbs_penable),
    .apbs_pwrite                (apbs_pwrite),
    .apbs_paddr                 (apbs_paddr),
    .apbs_pwdata                (apbs_pwdata),
    .apbs_prdata                (apbs_prdata),
    .apbs_pready                (apbs_pready),
    .apbs_pslverr               (apbs_pslverr),

    // Register interfaces
    .ctrl_count_mode_o          (count_mode),
    .ctrl_half_irq_o            (half_irq_en),
    .ctrl_over_irq_o            (over_irq_en),
    .ctrl_daddrinc_o            (dest_addr_inc),
    .ctrl_saddrinc_o            (src_addr_inc),
    .ctrl_dwidth_o              (data_width),
    .ctrl_start_i               (transfering_q),
    .ctrl_start_o               (start_wdata),
    .ctrl_start_wen             (start_wen),
    .sdaddr_o                   (src_data_addr),
    .ssaddr_o                   (src_status_addr),
    .srbit_o                    (src_ready_bit),
    .srpol_o                    (src_ready_polarity),
    .ddaddr_o                   (dest_data_addr),
    .dsaddr_o                   (dest_status_addr),
    .drbit_o                    (dest_ready_bit),
    .drpol_o                    (dest_ready_polarity),
    .count_o                    (tran_count),
    .irq_pend_halfover_pend_i   (halfover_pend_q),
    .irq_pend_halfover_pend_o   (halfover_pend),
    .irq_pend_over_pend_i       (over_pend_q),
    .irq_pend_over_pend_o       (over_pend)
);

endmodule
