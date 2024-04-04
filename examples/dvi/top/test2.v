  localparam DSIZE = 16;
  localparam ASIZE = 12;
  localparam AREMPTYSIZE = 1;//512-1;
  localparam AWFULLSIZE =  4096; // should not matter
  reg [DSIZE-1:0] rdata_fifo2;
  wire wfull_fifo2;
  wire rempty_fifo2;
  reg [DSIZE-1:0] wdata_fifo2;

  reg winc_fifo2,winn_fifo2;
  wire wclk_fifo2, wrst_n_fifo2;
  reg rinc_fifo2, rinn_fifo2;
  wire rclk_fifo2, rrst_n_fifo2;

  assign wrst_n_fifo2 = rst;
  assign rrst_n_fifo2 = rst;
  assign wclk_fifo2 = usb_clk;
  assign rclk_fifo2 = usb_clk;

  wire awfull_fifo2, arempty_fifo2;
  // Instantiate the FIFO
 async_fifo #( .DSIZE( DSIZE), .ASIZE(ASIZE), .AWFULLSIZE(AWFULLSIZE), .AREMPTYSIZE(AREMPTYSIZE)) dut2 (
    .winc(winc_fifo2),
    .wclk(wclk_fifo2),
    .wrst_n(wrst_n_fifo2),
    .rinc(rinc_fifo2),
    .rclk(rclk_fifo2),
    .rrst_n(rrst_n_fifo2),
    .wdata(wdata_fifo2),
    .rdata(rdata_fifo2),
    .wfull(wfull_fifo2),
    .rempty(rempty_fifo2),
    .arempty(arempty_fifo2),
    .awfull(awfull_fifo2)
  );