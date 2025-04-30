localparam FIFO_WIDTH = 32;
localparam FIFO_COUNT = 8;
localparam FIFO_LEVEL = FIFO_COUNT == 1 ? 1 : $clog2(FIFO_COUNT); // do not change