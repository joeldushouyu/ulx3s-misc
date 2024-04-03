module top_usbtest #(parameter x = 640,     // pixels
                     parameter y = 480,     // pixels
                     parameter f = 60,       // Hz 60, 50, 30
                     parameter xadjustf = 0, // or to fine-tune f
                     parameter yadjustf = 0, // or to fine-tune f
                     parameter c_ddr = 0    // 0:SDR 1:DDR
                     )
                    (input clk_25mhz,
                     input [6:0] btn,
                     output [7:0] led,
                     output [3:0] gpdi_dp,
                     output wifi_gpio0,    
                    
                     inout  [27:0] gn,
                     inout  [27:0] gp,
                     output sdram_clk,
                     output sdram_cke,
                     output sdram_csn,
                     output sdram_wen,
                     output sdram_rasn,
                     output sdram_casn,
                     output [12:0]sdram_a,
                     output [1:0]sdram_ba,
                     output [1:0]sdram_dqm,
                     inout [15:0]sdram_d 
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
    // the reset button

    wire [6:0] btnd, btnr, btnf;
    btn_debounce
    #(
        .bits(16),
        .btns(7)
    )
    btn_debounce_i
    (
        .clk(clk_shift),
        .btn(btn),
        .debounce(btnd),
        .rising(btnr),
        .falling(btnf)
    );


    // // // press BTN0 to exit this bitstream
    // reg [19:0] R_delay_reload = 0;
    // always @(posedge clk_25mhz)
    //   if (R_delay_reload[19] == 0)
    //     R_delay_reload <= R_delay_reload+1;
    // assign user_programn = btn[0] | ~R_delay_reload[19];
    wire rst = ~btn[6];
    //wire reset = rst;//TODO:
    // reg [7:0]data ;
    // assign led = data;
    // always @(  negedge rst) begin
    //     if(~rst)begin
    //     data <= data+1;
    //     end else begin
    //         data <= data;
    //     end
 
    // end
    // clock generator
    wire clk_locked;
    wire [3:0] clocks;
    wire clk_shift = clocks[0];
    wire clk_pixel = clocks[1];
    wire sram_clock = clocks[3]; //  is 166000000-666666 = 165333334 = 165MHZ
    wire usb_clk ;
    assign usb_clk = clocks[2];
    ecp5pll #(
    .in_hz(25000000),
    .out0_hz(pixel_f*5*(c_ddr?1:2)),
    .out1_hz(pixel_f),
    .out2_hz(35000000),
    .out2_tol_hz(2222222),
    // .out3_hz(166000000),
    // .out3_tol_hz(666666),
    ) ecp5pll_inst (
    .clk_i (clk_25mhz),
    .clk_o (clocks),
    .locked(clk_locked)
    );


  // VGA signal generator
    wire [7:0] vga_r, vga_g, vga_b;
    wire vga_hsync, vga_vsync, vga_blank;
    // wire fetch_next;
    
    reg [7:0] r_i, g_i, b_i;
    reg [7:0] r_i_n, g_i_n, b_i_n;


    reg  fetch_next;
    wire clock_pixel;
    localparam cbits_vga = 11;
    reg [cbits_vga-1:0] countx, county;
    wire [cbits_vga-1:0] countx_vga, county_vga;
    vga #(
    .c_resolution_x(x),
    .c_hsync_front_porch(hsync_front_porch),
    .c_hsync_pulse(hsync_pulse_width),
    .c_hsync_back_porch(hsync_back_porch),
    .c_resolution_y(y),
    .c_vsync_front_porch(vsync_front_porch),
    .c_vsync_pulse(vsync_pulse_width),
    .c_vsync_back_porch(vsync_back_porch),
    .c_bits_x(cbits_vga),
    .c_bits_y(cbits_vga)
    ) vga_instance (
    .rst(rst),
    .clk_pixel(clk_pixel),
    .clk_pixel_ena(1'b1),
    .test_picture(1'b1),  // enable test picture generation
    .fetch_next(fetch_next),
    .vga_hsync(vga_hsync),
    .vga_vsync(vga_vsync),
    .vga_blank(vga_blank),
    .vga_x_count(countx_vga),
    .vga_y_count(county_vga),
    .pixelSignal(clock_pixel)
    );
    
    // buffer logic 
    reg bufferreset, endOfRead, endOfWrite;
    reg hasResetBuffer = 0;
    reg ableToWrite, ableToRead;
    reg [23:0] readPixel, writePixel;
    reg [20:0]readPixelX , readPixelY,  writePixelX, writePixelY;
    reg readPixelSignal = 1, writePixelSignal = 1;


    reg [1:0] pixelState, pixelNextState;
    reg changed;
    parameter red = 2'b00, green = 2'b01, blue = 2'b10;
    

  
    reg signal;
    reg enableVideo_c, enableVideo_n;




    reg [31:0] counter;
    reg[31:0] frame_delay_counter;
    // assign led = counter[19:12];
    always@( posedge clock_pixel or negedge rst ) begin
      
      if(!rst)begin
        countx<=0;
        county<=0;
        enableVideo_c <=0;
        frame_delay_counter <=0;
        //enableVideo_n <=0;
      end
      else begin
     
        if(countx == x )begin

            countx <= 0;
            if(county == y-1)begin
                county <=0;
                counter<=0;
                frame_delay_counter <= frame_delay_counter+1;
                //led <= counter/4096;
            end
            else begin
                county<= county+1;
                counter <= counter+1;
            end

        end
        else begin
          countx <= countx + 1;
          counter <= counter+1;
        end

            enableVideo_c <= enableVideo_n;
        
            r_i <= r_i_n;
            g_i <= g_i_n;
            b_i <= b_i_n;


        end    
    end
    
    // main idea:
    // 1. sacrifice first frame time
    // 2. 3 clock before start of 2nd screen enable the video frame
    always @(*) begin
        // last row of the 1(discarded) frame
        // last 3 pixel of time already used in transition
        //TODO:
        if(frame_delay_counter == 32'd60 &&  enableVideo_c == 0&& county == y-1  && countx == x)begin
            //TODO:
            enableVideo_n =1;
        end
        else begin
            enableVideo_n = enableVideo_c;// don't touch
        end  
        if(enableVideo_c)begin      
            // if(county > y/2)begin
            //         r_i_n               = 255;
            //         b_i_n               = 255;
            //         g_i_n               =255 ;
            // end
            // else begin
            //         r_i_n               = 0;
            //         b_i_n               = 0;
            //         g_i_n               =0 ;
            // end
            if(county == 0 && countx == 200)begin
                    r_i_n               = 0;
                    b_i_n               = 0;
                    g_i_n               =0 ;
            end      
            else if(county == 0)begin
                    r_i_n               = 255;
                    b_i_n               = 255;
                    g_i_n               =255 ;
            end
            else if(county == y-1)begin
                    r_i_n               = 255;
                    b_i_n               = 255;
                    g_i_n               =255 ;
            end
            else if(countx == (x/2))begin
                    r_i_n               = 255;
                    b_i_n               = 0;
                    g_i_n               = 0;
            end
            else if(countx == 0)begin
                    r_i_n               = 255;
                    b_i_n               =255;
                    g_i_n               = 255;
            end
            else if(countx == x-1)begin
                    r_i_n               = 255;
                    b_i_n               = 255;
                    g_i_n               =255 ;
            end
            else begin

                    r_i_n               = 0;
                    b_i_n               = 255;
                    g_i_n               = 0;
            end
        end 
        else begin
                r_i_n               = 0;
                b_i_n               = 0;
                g_i_n               = 0;
        end
        
    end

    
    
    // VGA to digital video converter
    wire [1:0] tmds_clock, tmds_red, tmds_green, tmds_blue;
    vga2dvid #(
    .c_ddr(c_ddr ? 1'b1 : 1'b0),
    .c_shift_clock_synchronizer(1'b0)
    ) vga2dvid_instance (
    .clk_pixel(clk_pixel),
    .clk_shift(clk_shift),
    .in_red(r_i),
    .in_green(g_i),
    .in_blue(b_i),
    .in_hsync(vga_hsync),
    .in_vsync(vga_vsync),
    .in_blank(vga_blank),
    .out_clock(tmds_clock),
    .out_red(tmds_red),
    .out_green(tmds_green),
    .out_blue(tmds_blue)
    );
    
    generate
    if (c_ddr) begin
        // vendor specific DDR modules
        // convert SDR 2-bit input to DDR clocked 1-bit output (single-ended)
        // onboard GPDI
        ODDRX1F ddr0_clock (
        .D0(tmds_clock[0]),
        .D1(tmds_clock[1]),
        .Q(gpdi_dp[3]),
        .SCLK(clk_shift),
        .RST(0)
        );
        ODDRX1F ddr0_red (
        .D0(tmds_red[0]),
        .D1(tmds_red[1]),
        .Q(gpdi_dp[2]),
        .SCLK(clk_shift),
        .RST(0)
        );
        ODDRX1F ddr0_green (
        .D0(tmds_green[0]),
        .D1(tmds_green[1]),
        .Q(gpdi_dp[1]),
        .SCLK(clk_shift),
        .RST(0)
        );
        ODDRX1F ddr0_blue (
        .D0(tmds_blue[0]),
        .D1(tmds_blue[1]),
        .Q(gpdi_dp[0]),
        .SCLK(clk_shift),
        .RST(0)
        );
        end else begin
        assign gpdi_dp[3] = tmds_clock[0];
        assign gpdi_dp[2] = tmds_red[0];
        assign gpdi_dp[1] = tmds_green[0];
        assign gpdi_dp[0] = tmds_blue[0];
    end
    endgenerate

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

    wire SLCS, SLOE,SLRD, PKTEND;
    reg SLWR;
    wire [1:0] address; //A1, A0


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

    // wire master_in_write,start_write;
    // wire k_write_finish;
    // assign master_in_write = current_fx_master == fx_master_write;

//cypress fx3 master verilog code

reg [2:0] mode; 	

reg  [31:0] data_out;
reg  [1:0] oe_delay_cnt;	
reg  [1:0] fifo_address;   
reg  [1:0] fifo_address_d;   
reg       slrd_;
reg       slcs_;       
reg       slwr_;
reg      sloe_;
wire      usb_clk;
reg[7:0] conter; 		
wire clk_out_temp;	
reg rd_oe_delay_cnt; 
reg first_time;
reg[15:0] index ;
reg[15:0] DataCount_i ;
reg slrd1_d_ ;
reg slrd2_d_ ;
wire reset_;
wire [31:0]data_out_loopback;
wire [31:0]data_out_partial;
wire [31:0]data_out_zlp;
wire [31:0]data_out_stream_in;

reg [31:0]loopback_data_from_fx3;
reg [31:0]stream_out_data_from_fx3;
reg flaga_d;
reg flagb_d;
reg flagc_d;
reg flagd_d;

reg [2:0]current_fpga_master_mode_d;

reg [2:0]current_fpga_master_mode;
reg [2:0]next_fpga_master_mode;
 


wire lock;
reg short_pkt_strob;

reg pktend_;


reg [31:0]fdata_d;


wire slwr_loopback_;
wire slwr_streamIN_;
wire slwr_zlp_;
wire slwr_partial_;
wire pktend_partial_;
wire pktend_zlp_;

wire loopback_mode_selected;   
wire partial_mode_selected;   
wire zlp_mode_selected;       
wire stream_in_mode_selected;
wire stream_out_mode_selected;


//parameters for transfers mode (fixed value)
parameter [2:0] PARTIAL    = 3'd1;   //switch position on the Board 001
parameter [2:0] ZLP        = 3'd2;   //switch position on the Board 010
parameter [2:0] STREAM_IN  = 3'd3;   //switch position on the Board 011
parameter [2:0] STREAM_OUT = 3'd4;   //switch position on the Board 100
parameter [2:0] LOOPBACK   = 3'd5;   //switch position on the Board 101


//parameters for fpga master mode state machine
parameter [2:0] fpga_master_mode_idle             = 3'd0;
parameter [2:0] fpga_master_mode_partial          = 3'd1;
parameter [2:0] fpga_master_mode_zlp              = 3'd2;
parameter [2:0] fpga_master_mode_stream_in        = 3'd3;
parameter [2:0] fpga_master_mode_stream_out       = 3'd4;
parameter [2:0] fpga_master_mode_loopback         = 3'd5;
parameter [2:0] fpga_master_mode_delay = 3'd6;
parameter [2:0] fpga_master_mode_delay_from_stream_out_to_stream_in = 3'd7;


//output signal assignment
assign SLRD = slrd_;
assign SLWR = slwr_d1_;   
assign address = fifo_address_d;
assign SLOE = sloe_;
assign databus = (slwr_d1_) ? 32'dz : data_out;	
assign PMODE = 2'b11;		
assign RESET = 1'b1;	
assign SLCS = slcs_;
assign PKTEND = pktend_d1_;
	
reg sync_d;	

// always @(posedge usb_clk or negedge rst)begin
//     if(!rst)begin
//         SLWR <= 1'b1;   
//     end
//     else begin
//         SLWR <= slwr_d1_;
//     end
// end

//assign usb_clk = clk;   //used for TB


wire reading;
wire writing;
wire waiting_enter_writing;
//instantiation of stream_in mode	
slaveFIFO2b_streamIN stream_in_inst
	(
	     .reset_(rst),
         .clk_100(usb_clk),
         .stream_in_mode_selected(stream_in_mode_selected),
         .flaga_d(flaga_d),
         .flagb_d(flagb_d),
         .data_for_output(data_out_current),
         .slwr_streamIN_(slwr_streamIN_),
         .data_out_stream_in(data_out_stream_in),
         .writing(writing),
         .waiting_enter_writing(waiting_enter_writing)
	); 

//instantiation of stream_out mode	
slaveFIFO2b_streamOUT stream_out_inst
	(
 	     .reset_(rst),
         .clk_100(usb_clk),
         .stream_out_mode_selected(stream_out_mode_selected),
         .flagc_d(flagc_d),
         .flagd_d(flagd_d),
         .stream_out_data_from_fx3(stream_out_data_from_fx3),
         .slrd_streamOUT_(slrd_streamOUT_),
         .sloe_streamOUT_(sloe_streamOUT_),
         .reading(reading)
	);


// assign reset2pll = !reset_in_;
// assign reset_ = lock;
assign reset_ = rst;


//assign loopback_data_from_fx3 = databus;
//flopping the input data

always @(posedge usb_clk or  negedge reset_)begin
	if(!reset_)begin 
		fdata_d <= 32'd0;
	end else begin
		fdata_d <= databus;
	end	
end		


//selection of input data
always@(*)begin
    if((current_fpga_master_mode == fpga_master_mode_stream_out))begin
		loopback_data_from_fx3   = 32'd0;
		stream_out_data_from_fx3 = fdata_d;
	end else begin
		loopback_data_from_fx3   = 32'd0;
		stream_out_data_from_fx3 = 32'haa;
	end
end	


//floping the INPUT mode
always @(posedge usb_clk or  negedge reset_)begin
	if(!reset_)begin 
		mode <= 3'd0;
	end else begin
		mode <= mode_p;
	end	
end


///flopping the INPUTs flags
always @(posedge usb_clk or  negedge reset_)begin
	if(!reset_)begin 
		flaga_d <= 1'd0;
		flagb_d <= 1'd0;
		flagc_d <= 1'd0;
		flagd_d <= 1'd0;
	end else begin
		flaga_d <= FLAGA;
		flagb_d <= FLAGB;
		flagc_d <= FLAGC;
		flagd_d <= FLAGD;
	end	
end



//chip selection
always@(*)begin
	if(current_fpga_master_mode == fpga_master_mode_idle)begin
		slcs_ = 1'b1;
	end else begin
		slcs_ = 1'b0;
	end	
end

//selection of slave fifo address
always@(*)begin
	if((current_fpga_master_mode == fpga_master_mode_stream_out))begin
		fifo_address = 2'b11;
	end else if((current_fpga_master_mode == fpga_master_mode_stream_in))begin
		fifo_address = 2'b00;
	end else
		fifo_address = 2'b10;
end	


// //flopping the output fifo address
// always @(posedge usb_clk or  negedge reset_)begin
// 	if(!reset_)begin 
// 		fifo_address_d <= 2'd0;
//  	end else begin
// 		fifo_address_d <= fifo_address;
// 	end	
// end

//flopping the output fifo address
always @(posedge usb_clk)begin

 
		fifo_address_d <= fifo_address;
	
end

//SLRD an SLOE signal assignments based on mode
always @(*)begin
	case(current_fpga_master_mode)
	fpga_master_mode_loopback:begin
		slrd_ = slrd_loopback_;
		sloe_ = sloe_loopback_;
	end
	fpga_master_mode_stream_out:begin
		slrd_ = slrd_streamOUT_;
		sloe_ = sloe_streamOUT_;
	end
	default:begin
		slrd_ = 1'b1;
		sloe_ = 1'b1;
	end	
	endcase
end

//SLWR signal assignment based on mode	
always @(*)begin
	case(current_fpga_master_mode)
	fpga_master_mode_stream_in:begin
		slwr_ = slwr_streamIN_;
	end

	default:begin
		slwr_ = 1'b1;
	end	
	endcase
end

reg slwr_d1_;
always @(posedge usb_clk)begin

	
		slwr_d1_ <= slwr_;
	
end

reg sloe_d1_;
always @(posedge usb_clk)begin
    sloe_d1_ <=sloe_;
end


//pktend signal assignment based on mode
always @(*)begin
	case(current_fpga_master_mode)
	fpga_master_mode_partial:begin
		pktend_ = pktend_partial_;
	end
	fpga_master_mode_zlp:begin
		pktend_ = pktend_zlp_;
	end	
	default:begin
		pktend_ = 1'b1;
	end	
	endcase
end	

reg pktend_d1_;
always @(posedge usb_clk)begin

		pktend_d1_ <= pktend_;
	
end


// fifo stuff


//mode selection
assign loopback_mode_selected   = (current_fpga_master_mode == fpga_master_mode_loopback);
assign partial_mode_selected    = (current_fpga_master_mode == fpga_master_mode_partial);
assign zlp_mode_selected        = (current_fpga_master_mode == fpga_master_mode_zlp);
assign stream_in_mode_selected  = (current_fpga_master_mode == fpga_master_mode_stream_in);
assign stream_out_mode_selected = (current_fpga_master_mode == fpga_master_mode_stream_out );


//FIFO
  localparam DSIZE = 16;
  localparam ASIZE = 13;
  localparam AREMPTYSIZE = 1;//512-1;
  localparam AWFULLSIZE =  4096; // should not matter
  reg [DSIZE-1:0] rdata_fifo1;
  wire wfull_fifo1;
  wire rempty_fifo1;
  reg [DSIZE-1:0] wdata_fifo1;

  reg winc_fifo1,winn_fifo1;
  wire wclk_fifo1, wrst_n_fifo1;
  reg rinc_fifo1, rinn_fifo1;
  wire rclk_fifo1, rrst_n_fifo1;

  assign wrst_n_fifo1 = rst;
  assign rrst_n_fifo1 = rst;
  assign wclk_fifo1 = usb_clk;
  assign rclk_fifo1 = usb_clk;

  wire awfull_fifo1, arempty_fifo1;
  // Instantiate the FIFO
 async_fifo #( .DSIZE( DSIZE), .ASIZE(ASIZE), .AWFULLSIZE(AWFULLSIZE), .AREMPTYSIZE(AREMPTYSIZE)) dut (
    .winc(winc_fifo1),
    .wclk(wclk_fifo1),
    .wrst_n(wrst_n_fifo1),
    .rinc(rinc_fifo1),
    .rclk(rclk_fifo1),
    .rrst_n(rrst_n_fifo1),
    .wdata(wdata_fifo1),
    .rdata(rdata_fifo1),
    .wfull(wfull_fifo1),
    .rempty(rempty_fifo1),
    .arempty(arempty_fifo1),
    .awfull(awfull_fifo1)
  );


reg fin_cur;
reg fin_next;
reg [7:0] led_cur, led_next;

assign led = led_cur;
//Mode select state machine
reg [31:0] buf0_c, buf1_c, buf2_c, buf3_c, buf4_c;
reg [31:0] buf0_n, buf1_n, buf2_n, buf3_n, buf4_n;
reg [31:0] current_buf_size, next_buf_size;

reg [31:0] stream_out_count,stream_out_count_next;
reg [31:0] stream_in_count,stream_in_count_next;
reg [7:0] stream_in_debug_count, stream_in_debug_count_next;
reg [2:0] fpga_master_mode_after_delay, fpga_master_mode_after_delay_next;  
reg[31:0] data_out_current, data_out_next;
reg[31:0] delay_c, delay_n;



always @(posedge usb_clk  or negedge rst)begin
    if(!rst) begin
        winc_fifo1 <=0;
        rinc_fifo1 <=0;
        current_fpga_master_mode <= fpga_master_mode_idle;
        fin_cur <= 0;
        led_cur <=8'b00000000;
        current_buf_size <=32'd0;
        data_out_current <=32'hdf;
        delay_c <=32'd0;
        fpga_master_mode_after_delay <= fpga_master_mode_idle;
        // wdata_fifo1 <= 32'haa;
        stream_in_debug_count <=0;
    end
    else begin
		current_fpga_master_mode <= next_fpga_master_mode;
        fin_cur <= fin_next;
        led_cur <= led_next;
        current_buf_size <= next_buf_size;
        data_out_current <= data_out_next;


        //wdata_fifo1 <= stream_out_count;
        winc_fifo1 <= winn_fifo1;
        rinc_fifo1 <= rinn_fifo1;
        delay_c <=delay_n;
        fpga_master_mode_after_delay <= fpga_master_mode_after_delay_next;
        


        // debug only
        stream_in_debug_count <=stream_in_debug_count_next;
    end

end

assign wdata_fifo1 = stream_out_data_from_fx3;

always @(*) begin
    fin_next = fin_cur;
    led_next = led_cur;
    data_out_next =data_out_current;
    next_buf_size = current_buf_size;

    next_fpga_master_mode = current_fpga_master_mode;
    wdata_n = wdata_fifo1;
    rinn_fifo1 = rinc_fifo1;
    winn_fifo1 = winc_fifo1;
    delay_n = delay_c;
    fpga_master_mode_after_delay_next = fpga_master_mode_after_delay;


    // debug only
    stream_in_debug_count_next = stream_in_debug_count;
	case(current_fpga_master_mode)
	fpga_master_mode_idle:begin
        led_next = 8'd0;
        // // led_next[4] = FLAGA;
        // // led_next[5] = FLAGB;
        led_next[6] = FLAGC;
        led_next[7] = FLAGD;

        // if(FLAGA == 1 && FLAGB==1)begin

        //     next_fpga_master_mode = fpga_master_mode_stream_in;
        //     stream_in_debug_count_next = stream_in_debug_count+1;
        //     fpga_master_mode_after_delay_next = fpga_master_mode_idle;
        // end
        // else begin
        //     next_fpga_master_mode = fpga_master_mode_idle;
        // end
        if(delay_c != 0)begin
            delay_n = delay_c -1;
        end
        else if(FLAGC == 1 && FLAGD == 1  && rempty_fifo1)begin
            next_fpga_master_mode = fpga_master_mode_stream_out;
            stream_in_debug_count_next = stream_in_debug_count+1;
            fpga_master_mode_after_delay_next = fpga_master_mode_idle;
            delay_n = 2+4; //need be tuned3 ? set to 0 at testing with stream_out_count
        end
        else begin
            next_fpga_master_mode = fpga_master_mode_idle;
        end
	end


    fpga_master_mode_stream_out:begin
        led_next = 8'd1;


        if(delay_c != 0)begin
            delay_n = delay_c - 1;
            next_fpga_master_mode = fpga_master_mode_stream_out;
            led_next = 8'd2;
            if(delay_c == 1)begin
                winn_fifo1 = 1 ;// turn on ahead
            end
        end
        else begin

            if(end_stream_out_cnt)begin
                
                // if(!awfull_fifo1) begin
                //     next_fpga_master_mode = fpga_master_mode_delay;
                // end
                // else begin
                    next_fpga_master_mode = fpga_master_mode_delay_from_stream_out_to_stream_in;
                    winn_fifo1 = 0;
                    delay_n = 10;
                // end
            end
            else begin
                winn_fifo1 = 1;
                led_next = 8'd3;
                next_fpga_master_mode = fpga_master_mode_stream_out;
            end
        end
       
    end
    fpga_master_mode_delay_from_stream_out_to_stream_in: begin
        //winn_fifo1 = 0;
        // if(delay_c == 0)begin
        // should be full    now
        if(!awfull_fifo1) begin
            next_fpga_master_mode = fpga_master_mode_delay;
        end
        else begin

        if(delay_c == 0)begin
            next_fpga_master_mode = fpga_master_mode_stream_in;
        end
        else begin
            delay_n = delay_c -1;
        end

        end
        // else begin
        //     delay_n = delay_c -1;
        //     next_fpga_master_mode = fpga_master_mode_delay_from_stream_out_to_stream_in;
        // end 
    end
    fpga_master_mode_delay: begin
        led_next = 8'b00001111;
        next_fpga_master_mode = fpga_master_mode_delay;
        // if(delay_c == 0)begin
        //     next_fpga_master_mode = fpga_master_mode_after_delay;
        // end
        // else begin
        //     delay_n = delay_c-1;
        //     next_fpga_master_mode = fpga_master_mode_delay;
        // end
    end
	fpga_master_mode_stream_in:begin
            led_next = 8'd5;
     
            // if(fpga_master_mode_after_delay == fpga_master_mode_idle)begin
            //     delay_n = 0;
            //     fpga_master_mode_after_delay_next = fpga_master_mode_stream_in;
            //     next_fpga_master_mode = fpga_master_mode_delay;
            // end
            // else begin
            if(writing )begin
                if(end_stream_in_cnt)begin
                    led_next = 8'd8;
                    next_fpga_master_mode = fpga_master_mode_idle;
                    if(!rempty_fifo1)begin
                        // debug, means did not finish reading
                        data_out_next = 32'hcc;
                        next_fpga_master_mode = fpga_master_mode_delay;
                    end
                    else begin
                        data_out_next = rdata_fifo1;
                    end
                    //data_out_next[15:8] = stream_in_debug_count;
                    //data_out_next [7:0] =stream_in_count [7:0]; // first one is skipped in writing for sure
                    rinn_fifo1 = 0;
                    delay_n = 10;

                end
                else begin
                    led_next = 8'd6;
                    rinn_fifo1 = 1;
                    data_out_next = rdata_fifo1;

                    if(rempty_fifo1)begin
                        data_out_next = 32'hcf;
                    end
                    //data_out_next[15:8] = stream_in_debug_count;
                    //data_out_next [7:0] =stream_in_count [7:0]; // first one is skipped in writing for sure


                    next_fpga_master_mode = fpga_master_mode_stream_in;
                end
            end
            else begin
                data_out_next = 32'hcc;
                led_next = 8'd7;
                // idle for go into read mode
                next_fpga_master_mode = fpga_master_mode_stream_in;
            end
            // end

	end
	default:begin
		next_fpga_master_mode = fpga_master_mode_idle;
	end	
	endcase

end


reg     [31: 0]                 cnt  ; 
wire  end_stream_in_cnt, end_stream_out_cnt;
   
assign  end_stream_in_cnt     =      stream_in_count == 4096-1;  
assign end_stream_out_cnt = stream_out_count == 4096-1;
//data generator counter for  StreamIN modes
always @(posedge usb_clk or negedge rst)begin
    if(!rst)begin
        cnt <= 0;
        stream_in_count <=0;
        stream_out_count <=0;
    end
    else if(slwr_d1_ == 1'b0) begin
        cnt <= cnt+1;
        stream_in_count <= stream_in_count+1;
    end
    else if( current_fpga_master_mode == fpga_master_mode_stream_out && delay_c == 0)begin
        stream_out_count <=stream_out_count+1;
    end
    else  begin
        cnt <= 0;
        stream_in_count <=0;
        stream_out_count <=0;
    end
    // else begin
    //     stream_in_count <= stream_in_count;
    //     stream_out_count <= stream_out_count;

    // end


end





//selection of data_out based on current mode
always @(*)begin
	case(current_fpga_master_mode)
	fpga_master_mode_stream_in:begin
		data_out = data_out_stream_in;
	end
	default:begin
		data_out = 32'hef;
	end	
	endcase
end	


reg [31:0]data_out_d;
always @(posedge usb_clk)begin
		data_out_d <= data_out;
end




endmodule


