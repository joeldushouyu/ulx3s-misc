module top_usbtest #(parameter x = 640,     // pixels
                     parameter y = 480,     // pixels
                     parameter f = 60,       // Hz 60, 50, 30
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
        .clk(usb_clk),
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
    ecp5pll #(
    .in_hz(25000000),
    .out0_hz(pixel_f*5*(c_ddr?1:2)),
    .out1_hz(pixel_f),
    .out2_hz(100000),
    .out2_tol_hz(0),
    .out3_hz(166000000),
    .out3_tol_hz(666666),
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
    initial r_i_n = 255;
    initial g_i_n = 0;
    initial b_i_n = 0; 

    // initial gp[0] = 0;
    // always @(posedge clk_25mhz) begin
    //     gp[0] = ~gp[0];
    // end

    //assign gp[0] = clocks[0];
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
    wire usb_clk ;
    assign usb_clk = clock_pixel;

    // always@ (posedge clk_shift)begin
    //     if(fetch_next)begin
    //         signal <=1;
    //     end
    //     else begin
    //         signal <=0;
    //     end
    // end

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
     
        if(countx == x-1 )begin

            countx <= 0;
            if(county == y-1)begin
                county <=0;
                countx<=0;
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
        
            // r_i <= r_i_n;
            // g_i <= g_i_n;
            // b_i <= b_i_n;
            // r_i <=  databus[7:0];
            // g_i <= databus[7:0];
            // b_i <= databus[7:0];

        end    
    end
    assign r_i = 0;
    assign b_i = (enableVideo_c == 1'b1 ) ? stream_out_data_from_fx3[7:0]:0;
    assign g_i = (enableVideo_c == 1'b1 ) ? stream_out_data_from_fx3[7:0]:0;
    // always@( posedge clk_pixel or negedge rst) begin
      
    //   if(!rst)begin
    //     countx<=0;
    //     county<=0;
    //     enableVideo_c <=0;
    //     //enableVideo_n <=0;
    //   end
    //   else if(fetch_next)  begin

    //     if(countx == x-1 )begin

    //         countx <= 0;
    //         if(county == y-1)begin
    //             county <=0;
    //             countx<=0;
    //         end
    //         else begin
    //             county<= county+1;
    //         end

    //     end
    //     else begin
    //       countx <= countx + 1;
    //     end

    //     enableVideo_c <= enableVideo_n;
    
    //     r_i <= r_i_n;
    //     g_i <= g_i_n;
    //     b_i <= b_i_n;
    //     // r_i <=  databus[7:0];
    //     // g_i <= databus[7:0];
    //     // b_i <= databus[7:0];

    //   end
    //   else begin
    //     // when fetch_next is false
    //   end

    // end    
    

    // main idea:
    // 1. sacrifice first frame time
    // 2. 3 clock before start of 2nd screen enable the video frame
    always @(*) begin
        // last row of the 1(discarded) frame
        // last 3 pixel of time already used in transition
        //TODO:
        if(frame_delay_counter == 32'd1 &&  enableVideo_c == 0&& county == y-1  && countx == x-1)begin
            //TODO:
            enableVideo_n =1;
        end
        else begin
            enableVideo_n = enableVideo_c;// don't touch
        end  
        // if(county > (y/2))begin
        //         r_i_n               <= 255;
        //         b_i_n               <= 0;
        //         g_i_n               <= 0;
        // end
        // else begin

        //         r_i_n               <= 0;
        //         b_i_n               <= 255;
        //         g_i_n               <= 0;
        // end
        // if(counter == 32'h4AFFF)begin
        //     led=8'b00000001;
        // end
        if(enableVideo_c == 1 )begin
                r_i_n               =0 ; //stream_out_data_from_fx3[7:0];
                b_i_n               =  stream_out_data_from_fx3[7:0];
                g_i_n               = stream_out_data_from_fx3[7:0];
                
        end
        else begin

                r_i_n               = 0;
                b_i_n               = 0;
                g_i_n               = 0;
        end
       
        
    end

    // // // LED blinky
    // // assign led[7:6] = 0;
    // // assign led[0]   = vga_vsync;
    // // assign led[1]   = vga_hsync;
    // // assign led[2]   = vga_blank;
    
    
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

    wire SLCS,SLWR, SLOE,SLRD, PKTEND;
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


//output signal assignment
assign SLRD = slrd_;
assign SLWR = slwr_d1_;   
assign address = fifo_address_d;
assign SLOE = sloe_;
assign databus = (slwr_d1_) ? 32'dz : data_out_d;	
assign PMODE = 2'b11;		
assign RESET = 1'b1;	
assign SLCS = slcs_;
assign PKTEND = pktend_d1_;
	
reg sync_d;	



//assign usb_clk = clk;   //used for TB




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
         .data_out_stream_in(data_out_stream_in)
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
         .sloe_streamOUT_(sloe_streamOUT_)
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
	if(current_fpga_master_mode == fpga_master_mode_loopback)begin
		loopback_data_from_fx3   = fdata_d;
		stream_out_data_from_fx3 = 32'd0;
	end else if(current_fpga_master_mode == fpga_master_mode_stream_out)begin
		loopback_data_from_fx3   = 32'd0;
		stream_out_data_from_fx3 = fdata_d;
	end else begin
		loopback_data_from_fx3   = 32'd0;
		stream_out_data_from_fx3 = 32'd0;
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
	fpga_master_mode_partial:begin
		slwr_ = slwr_partial_;
	end
	fpga_master_mode_zlp:begin
		slwr_ = slwr_zlp_;
	end	
	fpga_master_mode_stream_in:begin
		slwr_ = slwr_streamIN_;
	end
	fpga_master_mode_loopback:begin
		slwr_ = slwr_loopback_;
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

//mode selection
assign loopback_mode_selected   = (current_fpga_master_mode == fpga_master_mode_loopback);
assign partial_mode_selected    = (current_fpga_master_mode == fpga_master_mode_partial);
assign zlp_mode_selected        = (current_fpga_master_mode == fpga_master_mode_zlp);
assign stream_in_mode_selected  = (current_fpga_master_mode == fpga_master_mode_stream_in);
assign stream_out_mode_selected = (current_fpga_master_mode == fpga_master_mode_stream_out);


  localparam DSIZE = 16;
  localparam ASIZE = 10;

  reg [DSIZE-1:0] rdata;
  reg wfull;
  reg rempty;
  reg [DSIZE-1:0] wdata, wdata_n;

  reg winc,winn;
  wire wclk, wrst_n;
  reg rinc, rinn;
  wire rclk, rrst_n;

  assign wrst_n = rst;
  assign rrst_n = rst;
  assign wclk = usb_clk;
  assign rclk = usb_clk;

  // Instantiate the FIFO
  async_fifo1 #(DSIZE, ASIZE) dut (
    .winc(winc),
    .wclk(wclk),
    .wrst_n(wrst_n),
    .rinc(rinc),
    .rclk(rclk),
    .rrst_n(rrst_n),
    .wdata(wdata),
    .rdata(rdata),
    .wfull(wfull),
    .rempty(rempty)

  );


reg fin_cur;
reg fin_next;
reg [7:0] led_cur, led_next;
assign led = led_cur;
//Mode select state machine
reg [31:0] buf0_c, buf1_c, buf2_c, buf3_c, buf4_c;
reg [31:0] buf0_n, buf1_n, buf2_n, buf3_n, buf4_n;
reg [31:0] current_buf_size, next_buf_size;

reg [31:0] stream_out_count;

reg [31:0] stream_in_count;

reg[31:0] data_out_current, data_out_next;
reg[31:0] delay_c, delay_n;
always @(posedge usb_clk  or negedge rst)begin
    if(!rst) begin

        current_fpga_master_mode <= fpga_master_mode_idle;
        fin_cur <= 0;
        led_cur <=8'b00000000;
        current_buf_size <=32'd0;
        data_out_current <=32'hdf;

        buf0_c <=32'hfd;
        buf1_c <=32'hfd;
        buf2_c <=32'hfd;
        buf3_c <=32'hfd;
        buf4_c <=32'hfd;

        delay_c <=32'd0;

        winc<=0;
        rinc<=0;
    end
    else begin
		current_fpga_master_mode <= next_fpga_master_mode;
        fin_cur <= fin_next;
        led_cur <= led_next;
        current_buf_size <= next_buf_size;
        data_out_current <= data_out_next;


        buf0_c <=buf0_n;
        buf1_c <=buf1_n;
        buf2_c <=buf2_n;
        buf3_c <=buf3_n;
        buf4_c <=buf4_n;

        delay_c <=delay_n;

        winc<=winn;
        rinc<=rinn;
        //wdata <=fdata_d[DSIZE-1:0];
        wdata <= wdata_n;
    end

end




always @(*) begin
    fin_next = fin_cur;
    led_next = led_cur;
    data_out_next =data_out_current;
    next_buf_size = current_buf_size;
    buf0_n = buf0_c;
    buf1_n = buf1_c;
    buf2_n = buf2_c;
    buf3_n = buf3_c;
    buf4_n = buf4_c;
    delay_n = delay_c;
    rinn = rinc;
    winn = winc;
    //wdata_n = wdata;
    wdata_n = stream_out_data_from_fx3[DSIZE-1:0];
	case(current_fpga_master_mode)
	fpga_master_mode_idle:begin
            led_next[4] = FLAGA;
            led_next[5] = FLAGB;
            led_next[6] = FLAGC;
            led_next[7] = FLAGD;

        if(fin_cur == 1 )begin
            next_fpga_master_mode = fpga_master_mode_idle;
            fin_next = 1;
            led_next[1] = 1;
            led_next[2] = 1;
        end
        else begin
            delay_n = 0;
            next_fpga_master_mode = fpga_master_mode_stream_out; 
            led_next = 8'b00000001;
            fin_next = 0;
            next_buf_size = 0;
            // //TODO: just for testing
            //next_buf_size = 6'd5;
        end
	end


    fpga_master_mode_stream_out:begin
        led_next = 8'b00000011;

        if(current_buf_size <8 && FLAGC == 1 && FLAGD == 1 && slrd_streamOUT_ == 0)begin
            // try 3 delay? could try to switch address+3 delay before got to sleep
            // case(current_buf_size)  
            //     3: begin buf0_n=stream_out_data_from_fx3; end
            //     4: begin buf1_n=stream_out_data_from_fx3; end
            //     5: begin buf2_n=stream_out_data_from_fx3; end
            //     6: begin buf3_n=stream_out_data_from_fx3; end
            //     7: begin buf4_n=stream_out_data_from_fx3; end
            
            // endcase
            if(current_buf_size == 3)begin
                // write data
                //wdata = stream_out_data_from_fx3[DSIZE-1:0];

                // if(!wfull)begin
                //     wdata_n = stream_out_data_from_fx3[DSIZE-1:0];
                // end
                
            end
            else begin
                next_buf_size = next_buf_size+1;
                if(current_buf_size == 2)begin
                    winn = 1;
                end
            end
        end
        // // else, big error
        // else begin
        //     led_next = 8'hff; // turn on all led
        // end

        //TODO: DATA LOST
        if(stream_out_count > 32'd512+5)begin 
            next_fpga_master_mode = fpga_master_mode_delay;
            winn=0;
    
            delay_n=3;
        end
        else begin
            next_fpga_master_mode = fpga_master_mode_stream_out;
        end
    end

    //TODO: 3 cycle delay
    fpga_master_mode_delay: begin
        led_next = 8'b00001111;
        if(delay_c == 0)begin
            if(next_buf_size ==0) begin next_fpga_master_mode=fpga_master_mode_stream_out; end
            else begin  next_fpga_master_mode=fpga_master_mode_stream_in; next_buf_size = 5;   rinn=1;     winn=0;     end // manual specific buf size
        end
        else begin
            delay_n = delay_n-1;
            // if(delay_c == 1 && delay_c !=0)begin
            //     winn=0;
            // end
            next_fpga_master_mode = fpga_master_mode_delay;
        end
        // if(sloe_streamOUT_ == 1)begin
        //     winn=0;
        //     rinn=1;
        //     next_fpga_master_mode = fpga_master_mode_stream_in;
        // end
        // else begin
        //     next_fpga_master_mode = fpga_master_mode_delay;
        // end
    end
	fpga_master_mode_stream_in:begin
  
                //winn=0;
        //winn = 0; 
        // if(current_buf_size >=0 && current_buf_size <9)begin
        //     if(current_buf_size == 0 )begin
        //         next_buf_size=10; // make sure never enter again

        //     end
        //     case(current_buf_size)
        //         4: begin data_out_next=buf0_c;  next_buf_size =next_buf_size -1;end
        //         3: begin data_out_next=buf1_c;  next_buf_size =next_buf_size -1;end
        //         2: begin data_out_next=buf2_c;  next_buf_size =next_buf_size -1;end
        //         1: begin data_out_next=buf3_c;  next_buf_size =next_buf_size -1;end
        //         0: begin data_out_next=buf4_c;  next_buf_size =next_buf_size -1;end
        //         5: begin  data_out_next =32'hab;   next_buf_size =next_buf_size -1; end
        //         default: begin data_out_next =32'hbb;  next_buf_size =next_buf_size -1;end //something very wrong
        //     endcase

        //     // else begin
        //     //     next_buf_size =next_buf_size -1;
        //     // end
        // end
        // else begin
        //     //TODO: just for testign
        //     data_out_next = 32'hcc;
        // end
        // if(FLAGA == 1 && FLAGB === 1)begin
        

        if(!rempty)begin
            data_out_next = rdata;
        end
        else begin
            data_out_next =  32'hcc;
        end
      
        // end
        // else begin
        //     // big troubel
        //     led_next = 8'h0f;
        // end



        if(stream_in_count >32'd512 )begin
            fin_next = 1;
            rinn =0;
            next_fpga_master_mode = fpga_master_mode_idle;
            led_next[6] = rempty;
            //led_next = 8'b00000111;
        end
        else begin
            next_fpga_master_mode = fpga_master_mode_stream_in;
            fin_next = 0;
        end
    

	end
	default:begin
		next_fpga_master_mode = fpga_master_mode_idle;
        fin_next = 0;
	end	
	endcase
    // case (current_fpga_master_mode)
    //     fpga_master_mode_idle: begin
    //         if(cnt == 0 )begin
    //             next_fpga_master_mode = fpga_master_mode_stream_in;
    //         end
    //         else begin
    //             next_fpga_master_mode = fpga_master_mode_idle;
    //         end

    //     end
    //     fpga_master_mode_stream_in: begin

    //         if(cnt ==4096) begin
    //             next_fpga_master_mode = fpga_master_mode_idle;
    //         end
    //         else begin
    //             next_fpga_master_mode = fpga_master_mode_stream_in;
    //         end 

    //     end
    //     default: begin 
    //         next_fpga_master_mode = fpga_master_mode_idle;
    //         cnt = 0;
    //     end

    // endcase
    // if(cnt > 4096)begin
    //     next_fpga_master_mode = fpga_master_mode_idle;
    // end
    // else begin
    //     // if(alreadyWrite == 0)begin
    //         next_fpga_master_mode = fpga_master_mode_stream_in;
    //     // end
    //     // else begin
    //     // next_fpga_master_mode = fpga_master_mode_idle;
    //     // end

    // end
end

//TODO: restore later
// always @(*) begin
//     fin_next = fin_cur;
//     led_next = led_cur;
//     data_out_next =data_out_current;
//     next_buf_size = current_buf_size;
//     buf0_n = buf0_c;
//     buf1_n = buf1_c;
//     buf2_n = buf2_c;
//     buf3_n = buf3_c;
//     buf4_n = buf4_c;
//     delay_n = delay_c;

// 	case(current_fpga_master_mode)
// 	fpga_master_mode_idle:begin
            
//             // led_next[4] = FLAGA;
//             // led_next[5] = FLAGB;
//             led_next[6] = FLAGC;
//             led_next[7] = FLAGD;
//             // led_next[0] = 1'b1;
//             //led_next = 8'b00000001;
//         // if(fin_cur == 1 )begin
//         //     next_fpga_master_mode = fpga_master_mode_idle;
//         //     fin_next = 1;

//         // end
//         // else begin
//         //     delay_n = 0;
//         //     next_fpga_master_mode = fpga_master_mode_delay; 
//         //     led_next = 8'b00000001;
//         //     fin_next = 0;

//         //     next_buf_size = 0;
//         //     // //TODO: just for testing
//         //     //next_buf_size = 6'd5;
//         // end

//         if(fin_cur == 1 )begin
//             next_fpga_master_mode = fpga_master_mode_idle;
//             fin_next = 1;

//         end
//         else if(enableVideo_c == 1) begin
//             delay_n = 0;
//             next_fpga_master_mode = fpga_master_mode_delay; 
//             fin_next = 0;

//             next_buf_size = 0;
//             // //TODO: just for testing
//             //next_buf_size = 6'd5;
//         end
//         else begin
//             next_fpga_master_mode = fpga_master_mode_idle;
//         end

// 	end


//     fpga_master_mode_stream_out:begin
//         led_next = 8'b00000011;

//         if(current_buf_size <8 && FLAGC == 1 && FLAGD == 1 && slrd_streamOUT_ == 0)begin
//             // try 3 delay? could try to switch address+3 delay before got to sleep
//             // case(current_buf_size)  
//             //     3: begin buf0_n=stream_out_data_from_fx3; end
//             //     4: begin buf1_n=stream_out_data_from_fx3; end
//             //     5: begin buf2_n=stream_out_data_from_fx3; end
//             //     6: begin buf3_n=stream_out_data_from_fx3; end
//             //     7: begin buf4_n=stream_out_data_from_fx3; end
            
//             // endcase
//             // next_buf_size = next_buf_size+1;

//             if(current_buf_size == 3)begin
//                 led_next =8'b00000111;
//             end
//             else begin
//                 next_buf_size = next_buf_size+1;
//             end
//         end
        
        
//         if(current_buf_size == 3 && (FLAGD==0 || FLAGC == 0)  )begin
//             // means have data broken in the flow
//             fin_next = 1;
//             led_next = 8'b00001111;
//             next_fpga_master_mode = fpga_master_mode_idle;
//         end
//         else begin
//             next_fpga_master_mode = fpga_master_mode_stream_out;

//         end

//         // assume never exit stream_out, see if can continously send data from host to fpga
//         // next_fpga_master_mode = fpga_master_mode_stream_out;


//         // if(stream_out_count >32'h1000  && FLAGC == 1)begin
//         //     next_fpga_master_mode = fpga_master_mode_delay;
//         //     delay_n=3;
//         // end
//         // else begin
//         //     next_fpga_master_mode = fpga_master_mode_stream_out;
//         // end
//     end

//     //TODO: 3 cycle delay
//     fpga_master_mode_delay: begin
//         if(delay_c == 0)begin
//             if(current_buf_size ==0) begin next_fpga_master_mode=fpga_master_mode_stream_out; end
//             else begin  next_fpga_master_mode=fpga_master_mode_stream_in; next_buf_size = 5;   end // manual specific buf size
//         end
//         else begin
//             delay_n = delay_n-1;
//             next_fpga_master_mode = fpga_master_mode_delay;
//         end
//     end
// 	// fpga_master_mode_stream_in:begin


//     //     if(current_buf_size >=0 && current_buf_size <9)begin
//     //         if(current_buf_size == 0 )begin
//     //             next_buf_size=10; // make sure never enter again

//     //         end
//     //         case(current_buf_size)
//     //             4: begin data_out_next=buf0_c;  next_buf_size =next_buf_size -1;end
//     //             3: begin data_out_next=buf1_c;  next_buf_size =next_buf_size -1;end
//     //             2: begin data_out_next=buf2_c;  next_buf_size =next_buf_size -1;end
//     //             1: begin data_out_next=buf3_c;  next_buf_size =next_buf_size -1;end
//     //             0: begin data_out_next=buf4_c;  next_buf_size =next_buf_size -1;end
//     //             5: begin  data_out_next =32'hab;   next_buf_size =next_buf_size -1; end
//     //             default: begin data_out_next =32'hbb;  next_buf_size =next_buf_size -1;end //something very wrong
//     //         endcase

//     //         // else begin
//     //         //     next_buf_size =next_buf_size -1;
//     //         // end
//     //     end
//     //     else begin
//     //         //TODO: just for testign
//     //         data_out_next = 32'hcc;
//     //     end



//     //     if(stream_in_count >32'h1000 )begin
//     //         fin_next = 1;
//     //         next_fpga_master_mode = fpga_master_mode_idle;
//     //         led_next = 8'b00000111;
//     //     end
//     //     else begin
//     //         next_fpga_master_mode = fpga_master_mode_stream_in;
//     //         fin_next = 0;
//     //     end
    

// 	// end
// 	default:begin
// 		next_fpga_master_mode = fpga_master_mode_idle;
//         fin_next = 0;
// 	end	
// 	endcase
//     // case (current_fpga_master_mode)
//     //     fpga_master_mode_idle: begin
//     //         if(cnt == 0 )begin
//     //             next_fpga_master_mode = fpga_master_mode_stream_in;
//     //         end
//     //         else begin
//     //             next_fpga_master_mode = fpga_master_mode_idle;
//     //         end

//     //     end
//     //     fpga_master_mode_stream_in: begin

//     //         if(cnt ==4096) begin
//     //             next_fpga_master_mode = fpga_master_mode_idle;
//     //         end
//     //         else begin
//     //             next_fpga_master_mode = fpga_master_mode_stream_in;
//     //         end 

//     //     end
//     //     default: begin 
//     //         next_fpga_master_mode = fpga_master_mode_idle;
//     //         cnt = 0;
//     //     end

//     // endcase
//     // if(cnt > 4096)begin
//     //     next_fpga_master_mode = fpga_master_mode_idle;
//     // end
//     // else begin
//     //     // if(alreadyWrite == 0)begin
//     //         next_fpga_master_mode = fpga_master_mode_stream_in;
//     //     // end
//     //     // else begin
//     //     // next_fpga_master_mode = fpga_master_mode_idle;
//     //     // end

//     // end
// end

reg     [31: 0]                 cnt  ; 
assign  add_cnt     =       slwr_streamIN_ == 1'b0;       
assign  end_cnt     =       add_cnt && cnt == 4096-1;  

//data generator counter for Partial, ZLP, StreamIN modes
always @(posedge usb_clk)begin
	// if(!reset_)begin 
	// 	data_gen_stream_in <= 32'd0;
	// end else 
	// if((slwr_streamIN_ == 1'b0) & (stream_in_mode_selected)) begin
    //     // if(end_cnt) begin
    //     //     cnt <= 0;
	// 	// end
    //     // else begin
    //         cnt <= cnt + 1;
	// 	// end
	// end else if (!stream_in_mode_selected) begin
	// 	cnt<= 0;
	// end	

    if(current_fpga_master_mode == fpga_master_mode_stream_in && slwr_streamIN_ == 1'b0)begin
        cnt <= cnt+1;
        stream_in_count <= stream_in_count+1;
    end
    else if(current_fpga_master_mode == fpga_master_mode_stream_out  )begin
        stream_out_count <=stream_out_count+1;
    end
    else begin
        cnt <= 0;
        stream_in_count <=0;
        stream_out_count <=0;
    end
    // cnt <= cnt+1;
end



// assign next_fpga_master_mode = fpga_master_mode_stream_in;
// //Mode select state machine combo   
// always @(*)   
// begin
//     //TODO:
//     next_fpga_master_mode = fpga_master_mode_stream_in;
// 	// next_fpga_master_mode = current_fpga_master_mode;
// 	// case (current_fpga_master_mode)
// 	// fpga_master_mode_idle:begin
// 	// 	case(mode)
// 	// 	LOOPBACK:begin
// 	// 		next_fpga_master_mode = fpga_master_mode_loopback;
// 	// 	end
// 	// 	PARTIAL:begin
// 	// 		next_fpga_master_mode = fpga_master_mode_partial;
// 	// 	end
// 	// 	ZLP:begin
// 	// 		next_fpga_master_mode = fpga_master_mode_zlp;
// 	// 	end
// 	// 	STREAM_IN:begin
// 	// 		next_fpga_master_mode = fpga_master_mode_stream_in;
// 	// 	end
// 	// 	STREAM_OUT:begin
// 	// 		next_fpga_master_mode = fpga_master_mode_stream_out;
// 	// 	end
// 	// 	default:begin
// 	// 		next_fpga_master_mode = fpga_master_mode_idle;
//     //             end
// 	// 	endcase
// 	// end	
// 	// fpga_master_mode_loopback:begin
// 	// 	if(mode == LOOPBACK)begin
// 	// 		next_fpga_master_mode = fpga_master_mode_loopback;
// 	// 	end else begin
// 	// 	        next_fpga_master_mode = fpga_master_mode_idle;
// 	// 	end	
// 	// end
// 	// fpga_master_mode_partial:begin
// 	// 	if(mode == PARTIAL)begin
// 	// 		next_fpga_master_mode = fpga_master_mode_partial;
// 	// 	end else begin 
// 	// 		next_fpga_master_mode = fpga_master_mode_idle;
// 	// 	end
// 	// end
// 	// fpga_master_mode_zlp:begin
// 	// 	if(mode == ZLP)begin
// 	// 		next_fpga_master_mode = fpga_master_mode_zlp;
// 	// 	end else begin 
// 	// 		next_fpga_master_mode = fpga_master_mode_idle;
// 	// 	end
// 	// end	
// 	// fpga_master_mode_stream_in:begin
// 	// 	if(mode == STREAM_IN)begin
// 	// 		next_fpga_master_mode = fpga_master_mode_stream_in;
// 	// 	end else begin 
// 	// 		next_fpga_master_mode = fpga_master_mode_idle;
// 	// 	end
// 	// end	
// 	// fpga_master_mode_stream_out:begin
// 	// 	if(mode == STREAM_OUT)begin
// 	// 		next_fpga_master_mode = fpga_master_mode_stream_out;
// 	// 	end else begin 
// 	// 		next_fpga_master_mode = fpga_master_mode_idle;
// 	// 	end
// 	// end	
// 	// default:begin
// 	// 	next_fpga_master_mode = fpga_master_mode_idle;
// 	// end
// 	// endcase

// end



//selection of data_out based on current mode
always @(*)begin
	case(current_fpga_master_mode)
	fpga_master_mode_partial:begin
		data_out = data_out_partial;
	end
	fpga_master_mode_zlp:begin
		data_out = data_out_zlp;
	end	
	fpga_master_mode_stream_in:begin
		data_out = data_out_stream_in;
	end
	fpga_master_mode_loopback:begin
		data_out = data_out_loopback;
	end
	default:begin
		data_out = 32'd0;
	end	
	endcase
end	


reg [31:0]data_out_d;
always @(posedge usb_clk)begin
		data_out_d <= data_out;
end




   // //button layout in ulx3s
    // //                      0
    // // 1(reset) 2         
    // ///                     3
    // //                  5   4   6


    // reg [7:0] seg_out;
    // reg [5:0] sel_out;
    
    // reg [2:0] key;
    // assign key[0] = btn[5];
    // assign key[1] = btn[4];
    // assign key[2] = btn[1];
    // wire clk = sram_clock;
    // wire rst_n = rst;

    // // // // assign led[7:4] = 0;
    // // // // reg [3:0] o_led;
    // // // // assign led[0] = o_led[0];
    // // // // assign  led[1] = o_led[1];
    // // // // assign led[2] =  o_led[2] ;
    // // // // assign led[3]= o_led[3] ;
    // // // comprehensive_tb #() SRAMTest(
    // // //     .clk(sram_clock),
    // // //     .rst_n(rst),
    // // //     .key(input_key),
    // // //     .led(led),
    // // //     // .seg_out(seg_out),
    // // //     // .sel_out(sel_out),
    // // //     .sdram_clk(sdram_clk),
    // // //     .sdram_cke(sdram_cke),
    // // //     .sdram_cs_n(sdram_csn),
    // // //     .sdram_ras_n(sdram_rasn),
    // // //     .sdram_cas_n(sdram_casn),
    // // //     .sdram_we_n(sdram_wen),
    // // //     .sdram_addr(sdram_a),
    // // //     .sdram_ba(sdram_ba),
    // // //     .sdram_dqm(sdram_dqm),
    // // //     .sdram_dq(sdram_d)
    // // // );


	// wire CLK_OUT;
	// assign CLK_OUT = clk; // input 165.5MHZ
	 
	//  //FSM state declarationss
	//  localparam idle=0,
	// 				//write test-data to all addresses
	// 				new_write=1,
	// 				write_burst=2,
	// 				//read test-data written to all addresses
	// 				new_read=3,
	// 				read_burst=4;
					
	//  reg[2:0] state_q=idle,state_d;
	//  reg[14:0] f_addr_q=0,f_addr_d; 
	//  reg[9:0] burst_index_q=0,burst_index_d;
	//  reg[7:0] led_q=0,led_d;
	//  reg[19:0] error_q=0,error_d;
	 
	//  reg rw,rw_en;
	//  reg[15:0] f2s_data;
	//  wire ready,s2f_data_valid,f2s_data_valid;
	//  wire[15:0] s2f_data;
	//  wire key0_tick,key1_tick;
	//  wire[5:0] in0,in1,in2,in3,in4,in5; //format: {dp,char[4:0]} , dp is active high
	 
	//  (*KEEP="TRUE"*)reg[36:0] counter_q,index_q,index_d; //counter_q increments until 1 second(165_000_000 clk cycles). Index_q holds the number of words read/written(check the value at chipscope)	
	// 							//Memory Bandwidth: index_q*2 = X bytes/seconds
	// 							// RESULT: 190MB/s (100MHz with t_CL=2)
	// 							// RESULT: 316MB/s (165MHz clk with t_CL=3)

	//  //register operations
	//  always @(posedge CLK_OUT,negedge rst_n) begin
	// 	if(!rst_n) begin
	// 		state_q<=0;
	// 		f_addr_q<=0;
	// 		burst_index_q<=0;
	// 		led_q<=0;
	// 		error_q<=0;
	// 		counter_q<=0;
	// 		index_q<=0;
	// 	end
	// 	else begin
	// 		state_q<=state_d;
	// 		f_addr_q<=f_addr_d;
	// 		burst_index_q<=burst_index_d;
	// 		led_q<=led_d;
	// 		error_q<=error_d;	
	// 		counter_q<=(state_q==idle) ?0:counter_q+1'b1;
	// 		index_q<=index_d;
	// 	end
	//  end
	 
	//  //FSM next-state logic
	//  always @* begin
	//  state_d=state_q;
 	//  f_addr_d=f_addr_q;
	//  burst_index_d=burst_index_q;
	//  led_d=led_q;
	//  error_d=error_q;
	//  rw=0;
	//  rw_en=0;
	//  f2s_data=0;
	//  index_d=index_q;
	 
	//  case(state_q)		
	// 	  		 idle: begin  //wait until either button is toggled
	// 						f_addr_d=0;
	// 						burst_index_d=0;
	// 						if(key[0]) begin
	// 							state_d=new_write; 
	// 							index_d=0;
	// 						end
	// 						if(key[1]) begin
	// 							state_d=new_read;
	// 							error_d=0;
	// 							index_d=0;
	// 						end
	// 					 end
	// 	  new_write: if(ready) begin  //write a deterministic data to all possible addresses of sdram
	// 						led_d[1]=1'b1;
	// 						rw_en=1;
	// 						rw=0;
	// 						state_d=write_burst;
	// 						burst_index_d=0;
	// 					 end
	// 	write_burst: begin 
	// 						f2s_data=f_addr_q+burst_index_q;
	// 						//if(key[2] && (f_addr_q==13000 || f_addr_q==100)) f2s_data=9999; //Inject errors when key[2] is pressed. The output error must be 512*2*10=10240
	// 						if(f2s_data_valid) begin
	// 							burst_index_d=burst_index_q+1; //track the number of already bursted data
	// 							index_d=index_q+1'b1; //holds the total number of words written to sdram
	// 						end
	// 						else if(burst_index_q==512) begin //last data must be 512th(for full page mode), since index starts at zero, the 512th is expected to have deasserted f2s_data_valid
	// 							if(counter_q>=165_000_000) begin //1 second had passed
	// 								led_d[1:0]=2'b11; 
	// 								state_d=idle;
	// 							end
	// 							else begin
	// 								f_addr_d=f_addr_q+1;
	// 								state_d=new_write;
	// 							end
	// 						end
	// 					 end
	// 	   new_read: if(ready) begin //read each data from all addresses and test if it matches the deterministic data we assigned earlier
	// 						led_d[2]=1'b1;
	// 						rw_en=1;
	// 						rw=1;
	// 						state_d=read_burst;
	// 						burst_index_d=0;
	// 					 end
	// 	 read_burst: begin
	// 						if(s2f_data_valid) begin
	// 							if(s2f_data!=f_addr_q+burst_index_q) error_d=error_q+1'b1; //count the errors in which the read output does not match the expected assigned data
	// 							burst_index_d=burst_index_q+1;
	// 							index_d=index_q+1'b1; //holds the total number of words read from sdram
	// 							led_d[7:4]  = error_d;
	// 						end
	// 						else if(burst_index_q==512) begin
	// 							if(counter_q>=165_000_000) begin //1 second had passed
	// 								led_d[3:0]=4'b1111; //all leds on after successfull write
	// 								state_d=idle;
	// 							end
	// 							else begin
	// 								f_addr_d=f_addr_q+1;
	// 								state_d=new_read;
	// 							end
	// 						end
	// 					 end
	// 	    default: state_d=idle;
	//  endcase
	//  end
	 
	//  assign led=led_q;
	
	// //module instantiations
	//  sdram_controller m0
	//  (
	// 	//fpga to controller
	// 	.clk(CLK_OUT), //clk=100MHz
	// 	.rst_n(rst_n),  
	// 	.rw(rw), // 1:read , 0:write
	// 	.rw_en(rw_en), //must be asserted before read/write
	// 	.f_addr(f_addr_q), //23:11=row  , 10:9=bank  , no need for column address since full page mode will always start from zero and end with 511 words
	// 	.f2s_data(f2s_data), //fpga-to-sdram data
	// 	.s2f_data(s2f_data), //sdram to fpga data
	// 	.s2f_data_valid(s2f_data_valid),  //asserts while  burst-reading(data is available at output UNTIL the next rising edge)
	// 	.f2s_data_valid(f2s_data_valid), //asserts while burst-writing(data must be available at input BEFORE the next rising edge)
	// 	.ready(ready), //"1" if sdram is available for nxt read/write operation
	// 	//controller to sdram
	// 	.s_clk(sdram_clk),
	// 	.s_cke(sdram_cke), 
	// 	.s_cs_n(sdram_csn),
	// 	.s_ras_n(sdram_rasn ), 
	// 	.s_cas_n(sdram_casn),
	// 	.s_we_n(sdram_wen), 
	// 	.s_addr(sdram_a), 
	// 	.s_ba(sdram_ba), 
	// 	.LDQM(sdram_dqm[0]),
	// 	.HDQM(sdram_dqm[1]),
	// 	.s_dq(sdram_d) 
    // );
	 


endmodule


