module top_vgatest #(parameter x = 1920,     // pixels
                     parameter y = 1080,     // pixels
                     parameter f = 30,       // Hz 60, 50, 30
                     parameter xadjustf = 0, // or to fine-tune f
                     parameter yadjustf = 0, // or to fine-tune f
                     parameter c_ddr = 1,    // 0:SDR 1:DDR
                     )
                    (input clk_25mhz,
                     input [6:0] btn,
                     output [7:0] led,
                     output [3:0] gpdi_dp,
                     output wifi_gpio0,
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
    wire clk_shift = clocks[1];
    wire clk_pixel = clocks[2];
    wire sram_clk = clock[0]; // it is 164000000 + 1500000, which is 165.5MHZ
    ecp5pll #(
    .in_hz  (25000000),
    .out1_hz(pixel_f * 5 * (c_ddr ? 1 : 2)),
    .out2_hz(pixel_f),  // this is a 183.02MHZ at 1080p 30hz
    .out0_hz(164000000),
    .out0_tol_hz(1500000),
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
    
    
    //  initial r_i = 0;
    //  initial g_i = 250;
    //  initial b_i = 255;
    wire  fetch_next;
    
    vga #(
    .c_resolution_x(x),
    .c_hsync_front_porch(hsync_front_porch),
    .c_hsync_pulse(hsync_pulse_width),
    .c_hsync_back_porch(hsync_back_porch),
    .c_resolution_y(y),
    .c_vsync_front_porch(vsync_front_porch),
    .c_vsync_pulse(vsync_pulse_width),
    .c_vsync_back_porch(vsync_back_porch),
    .c_bits_x(11),
    .c_bits_y(11)
    ) vga_instance (
    .clk_pixel(clk_pixel),
    .clk_pixel_ena(1'b1),
    .test_picture(1'b1),  // enable test picture generation
    .fetch_next(fetch_next),
    .vga_hsync(vga_hsync),
    .vga_vsync(vga_vsync),
    .vga_blank(vga_blank)
    );
    
    reg [1:0] pixelState, pixelNextState;
    reg changed;
    parameter red = 2'b00, green = 2'b01, blue = 2'b10;
    
    
    reg [9:0] x_current = 10'b0;
    reg [9:0] y_current = 10'b0;
    
    reg [20:0] countx = 0;    reg [20:0] county = 0;
    always@(posedge clk_pixel) begin
        
        if (fetch_next) begin
            if (countx == x-1)begin
                
                countx <= 0;
                if (county == (y/2)-1) begin
                    pixelState <= pixelNextState;
                    county     <= 0;
                end
                else begin
                    county <= county+1;
                end
                
            end
            else begin
                countx <= countx + 1;
            end
            
        end
        
    end
    
    // always @(posedge vga_hsync) begin
    //     if (count == 100) begin
    //         pixelState <= pixelNextState;
    //         count      <= 0;
    //     end
    //     else begin
    //         count <= count + 1;
    //     end
    
    
    
    // end
    
    
    always @(posedge fetch_next) begin
        
        case (pixelState)
            red:
            begin
                // pixelNextState <= blue ;
                r_i               <= 255;
                b_i               <= 0;
                g_i               <= 0;
                pixelNextState    <= blue ;
                
            end
            blue:
            begin
                pixelNextState <= red;
                r_i            <= 0;
                b_i            <= 255;
                g_i            <= 0;
            end
            
                // green:
                // begin
                //     pixelNextState <= red;
                //     r_i            <= 0;
                //     b_i            <= 0;
                //     g_i            <= 255;
                // end
            
            
            default: pixelNextState <= red;
        endcase
        
    end
    
    // LED blinky
    assign led[7:6] = 0;
    assign led[0]   = vga_vsync;
    assign led[1]   = vga_hsync;
    assign led[2]   = vga_blank;
    
    
    
    
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
    
    /*
     // external GPDI
     ODDRX1F ddr1_clock (.D0(tmds_clock[0]), .D1(tmds_clock[1]), .Q(gp[12]), .SCLK(clk_shift), .RST(0));
     ODDRX1F ddr1_red   (.D0(tmds_red  [0]), .D1(tmds_red  [1]), .Q(gp[11]), .SCLK(clk_shift), .RST(0));
     ODDRX1F ddr1_green (.D0(tmds_green[0]), .D1(tmds_green[1]), .Q(gp[10]), .SCLK(clk_shift), .RST(0));
     ODDRX1F ddr1_blue  (.D0(tmds_blue [0]), .D1(tmds_blue [1]), .Q(gp[9]), .SCLK(clk_shift), .RST(0));
     */
    
endmodule
