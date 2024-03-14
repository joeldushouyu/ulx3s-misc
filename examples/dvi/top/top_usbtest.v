module top_usbtest #(parameter x = 1920,     // pixels
                     parameter y = 1080,     // pixels
                     parameter f = 30,       // Hz 60, 50, 30
                     parameter xadjustf = 0, // or to fine-tune f
                     parameter yadjustf = 0, // or to fine-tune f
                     parameter c_ddr = 1    // 0:SDR 1:DDR
                     )
                    (input clk_25mhz,
                     input [6:0] btn,
                     output [7:0] led,
                     output [3:0] gpdi_dp,
                     output wifi_gpio0,    
                    
                     inout  [27:0] gn,
                    //  inout [27:22]gn,
                    //  inout [16:0] gn,
                    //  output [27:0]gn,
                     inout  [27:0] gp,
                    // inout wire [1:0]gp,

                     );

    function integer F_find_next_f(input integer f);
        if (25000000 > f) F_find_next_f       = 25000000;
        else if (27000000 > f) F_find_next_f  = 27000000;
        else if (40000000 > f) F_find_next_f  = 40000000;
        else if (50000000 > f) F_find_next_f  = 50000000;
        else if (54000000 > f) F_find_next_f  = 54000000;
        else if (60000000 > f) F_find_next_f  = 60000000;
        else if (65000000 > f) F_find_next_f  = 65000000;
        else if (75000000 > f) F_find_next_f  = 75000000;
        else if (80000000 > f) F_find_next_f  = 80000000;   // overclock
        else if (100000000 > f) F_find_next_f = 100000000;  // overclock
        else if (108000000 > f) F_find_next_f = 108000000;  // overclock
        else if (120000000 > f) F_find_next_f = 120000000;  // overclock
    endfunction
    
    localparam xminblank         = x / 64;  // initial estimate
    localparam yminblank         = y / 64;  // for minimal blank space
    localparam min_pixel_f       = f * (x + xminblank) * (y + yminblank);
    localparam pixel_f           = F_find_next_f(min_pixel_f);
    localparam yframe            = y + yminblank;
    localparam xframe            = pixel_f / (f * yframe);
    localparam xblank            = xframe - x;
    localparam yblank            = yframe - y;
    localparam hsync_front_porch = xblank / 3;
    localparam hsync_pulse_width = xblank / 3;
    localparam hsync_back_porch  = xblank - hsync_pulse_width - hsync_front_porch + xadjustf;
    localparam vsync_front_porch = yblank / 3;
    localparam vsync_pulse_width = yblank / 3;
    localparam vsync_back_porch  = yblank - vsync_pulse_width - vsync_front_porch + yadjustf;
    
    
    // wifi_gpio0 = 1 keeps board from rebooting
    // hold btn0 to let ESP32 take control over the board
    assign wifi_gpio0 = btn[0];
    
    // // press BTN0 to exit this bitstream
    // reg [19:0] R_delay_reload = 0;
    // always @(posedge clk_25mhz)
    //   if (R_delay_reload[19] == 0)
    //     R_delay_reload <= R_delay_reload+1;
    // assign user_programn = btn[0] | ~R_delay_reload[19];
    // clock generator
    wire clk_locked;
    wire [3:0] clocks;
    wire clk_shift = clocks[0];
    wire clk_pixel = clocks[1];
    wire usb_clk = clk_pixel;
    ecp5pll #(
    .in_hz(25000000),
    .out0_hz(pixel_f*5*(c_ddr?1:2)),
    .out1_hz(pixel_f),
    .out2_hz(100000),
    .out2_tol_hz(0),
    ) ecp5pll_inst (
    .clk_i (clk_25mhz),
    .clk_o (clocks),
    .locked(clk_locked)
    );

    // assign gp[0] = usb_clk;
    //testing purpose

    // assign  gn[26] = usb_clk;
    // assign  gn[27]=usb_clk;
    // assign  gp[14]=usb_clk;
    // assign  gp[15]=usb_clk;
    // assign  gp[16]=usb_clk;
    // assign gp[17]  =usb_clk;
    // assign gp[18] = usb_clk;
    // assign  gp[19] = usb_clk;
    // assign  gp[20] = usb_clk;
    // assign  gp[21] = usb_clk;
    // assign  gp[22] = usb_clk;
    // assign  gp[23]= usb_clk;
    // assign   gp[24] = usb_clk;
    // assign   gp[25] = usb_clk;
    // assign  gp[26] = usb_clk;
    // assign   gp[27] =usb_clk;
    

    // // // define tghings needed

    wire  [31:0] databus;
    // manually assign the wire
    // assign  gn[26] = 1'b1;
    // assign  gn[27] = 1'b1;
    // assign  gp[14] = 1'b1;
    // assign  gp[15] = 1'b1;
    // assign  gp[16] = 1'b1;
    // assign  gp[17] = 1'b1;
    // assign  gp[18] = 1'b1;
    // assign  gp[19] = 1'b1;
    // assign  gp[20] = 1'b0;
    // assign  gp[21] = 1'b0;
    // assign  gp[22] = 1'b0;
    // assign  gp[23] = 1'b0;
    // assign  gp[24] = 1'b0;
    // assign  gp[25] = 1'b0 ;
    // assign  gp[26] = 1'b0;//databus[14];
    // assign  gp[27] = 1'b0;//databus[15] ;
    assign  gn[26] =  databus[0];
    assign  gn[27] =  databus[1];
    assign  gp[14] =  databus[2];
    assign  gp[15] =  databus[3];
    assign  gp[16] =  databus[4];
    assign  gp[17] =  databus[5];
    assign  gp[18] =  databus[6];
    assign  gp[19] =  databus[7];
    assign  gp[20] =  databus[8];
    assign  gp[21] =  databus[9];
    assign  gp[22] =  databus[10];
    assign  gp[23] =  databus[11];
    assign  gp[24] =  databus[12];
    assign  gp[25] =  databus[13];
    assign  gp[26] = databus[14];
    assign  gp[27] = databus[15] ;
    // //TODO: later
    // // assign databus[31:16];

        
    // // SLCS, SLWR, SLOE, SLRD, PTKEND is output from FPGA
    // // PCLK, A1, A2 is output
    // // FLAGA,B,C,D is input only

    assign gp[0] = usb_clk;
    


    wire SLCS,SLWR, SLOE,SLRD, PKTEND;
    wire [1:0] address; //A1, A0
    wire reset = 1; //TODO:

    assign gn[13] = SLCS;
    assign gn[14] = SLWR;
    assign gn[15] = SLOE;
    assign gn[16] = SLRD;
    assign FLAGA = gn[17];
    assign FLAGB = gn[18];
    assign FLAGC = gn[19];
    assign gn[20] = PKTEND;
    assign FLAGD = gn[21];
    assign gn[25] = address[0];//1'b0;
    assign gn[24] = address[1];//1'b0;

    

    // usb_stream_in_example usb_write_instace(

    //     .clk(usb_clk),
    //     .rst_n(reset),
    //     .flag_a(FLAGA),
    //     .flag_b(FLAGB),
    //     .flag_c(FLAGC),
    //     .flag_d(FLAGD),
    //     .slcs(SLCS),
    //     .sloe(SLOE),
    //     .slrd(SLRD),
    //     .slwr(SLWR),
    //     .pktend(PKTEND),
    //     .fifo_addr(address),
    //     .usb_data(databus)

    // );

    // for read from usb
    usb_stream_out_example usb_read_instance(

        .clk(usb_clk),
        .rst_n(reset),
        .flag_a(FLAGA),
        .flag_b(FLAGB),
        .flag_c(FLAGC),
        .flag_d(FLAGD),
        .slcs(SLCS),
        .sloe(SLOE),
        .slrd(SLRD),
        .slwr(SLWR),
        .pktend(PKTEND),
        .fifo_addr(address),
        .usb_data(databus)

    );
    
    reg[31:0] databuf[5:0];


    // test case
    // //FOr fifo experiment only
    // wire                            cmd_flag;
    // wire    [31: 0]                 cmd_data; 
    // usb_fifo_example usb_fifo_instance(
    //     .clk(usb_clk),
    //     .rst_n(reset),
    //     .flag_a(FLAGA),
    //     .flag_b(FLAGB),
    //     .flag_c(FLAGC),
    //     .flag_d(FLAGD),
    //     .slcs(SLCS),
    //     .sloe(SLOE),
    //     .slrd(SLRD),
    //     .slwr(SLWR),
    //     .pktend(PKTEND),
    //     .fifo_addr(address),
    //     .usb_data(databus),
    //     .cmd_flag(cmd_flag),
    //     .cmd_data(cmd_data)
    // );
    
    // parameter master_mode_loopback = 3'b000;
    // parameter master_mode_stream_out = 3'b001;
    // parameter master_mode_stream_in = 3'b010;
    // parameter master_mode_ZLP = 3'b011;
    // parameter master_mode_partial = 3'b100;
    // parameter master_mode_idle = 3'b101;

    // // // stream in 
    // parameter stream_in_idle           = 2'b00;
    // parameter stream_in_wait_flagb     = 2'b01;
    // parameter stream_in_write          = 2'b10;
    // parameter stream_in_write_wr_delay = 2'b11;
    // // // stream_in 
    // reg [1:0] stream_in_mode;
    // reg [2:0] current_master, next_master;
    // reg [15:0] counter;
    // usb_stream_in #(
    //     .master_mode_loopback(master_mode_loopback),
    //     .master_mode_stream_out(master_mode_stream_out),
    //     .master_mode_stream_in(master_mode_stream_in),
    //     .master_mode_ZLP(master_mode_ZLP),
    //     .master_mode_partial(master_mode_partial),
    //     .master_mode_idle(master_mode_idle),
    //     .stream_in_idle(stream_in_idle),
    //     .stream_in_wait_flagb(stream_in_wait_flagb),
    //     .stream_in_write(stream_in_write),
    //     .stream_in_write_wr_delay(stream_in_write_wr_delay)
    // ) usb_stream_in_instance(

    //     .clk(usb_clk),
    //     .rst_n(reset),
    //     .master_mode(current_master),
    //     .data_out(counter),
    //     .PKTEND(PKTEND),
    //     .SLOE(SLOE),
    //     .SLRD(SLRD),
    //     .SLCS(SLCS),
    //     .SLWR(SLWR),
    //     .A(address),
    //     .FLAGA(FLAGA),
    //     .FLAGB(FLAGB),
    //     .DQ(databus),
    //     .current_stream_in_mode(stream_in_mode),

    // );

    // always @(posedge usb_clk) begin
    //     current_master <= next_master;
    //     counter <= counter+1;
    // end 
    // //Test case , write all iteration of the 16 bit letter

    
    // always @(*) begin
    //     case(current_master)
    //         master_mode_idle: begin
    //             //SLCS = 1;
    //             next_master = master_mode_stream_in;
    //         end
    //         master_mode_stream_in: begin
    //             //counter = counter + 1;
    //             next_master = master_mode_stream_in;
    //         end

    //         default: begin
    //             next_master = master_mode_idle;
    //         end
    //     endcase

    // end

    // always @(current_master) begin
    //     if(current_master == master_mode_stream_in ) begin
    //         counter <= counter+1;
    //     end
    //     else begin
    //         counter <= 0;
    //     end
    // end

endmodule


