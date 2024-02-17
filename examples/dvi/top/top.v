module top #(

    //  modes tested on lenovo monitor
    //  640x400  @50Hz
    //  640x400  @60Hz
    //  640x480  @50Hz
    //  640x480  @60Hz
    //  720x576  @50Hz
    //  720x576  @60Hz
    //  800x480  @60Hz
    //  800x600  @60Hz
    // 1024x768  @60Hz
    // 1280x768  @60Hz
    // 1366x768  @60Hz
    // 1280x1024 @60Hz
    // 1920x1080 @30Hz
    // 1920x1080 @50Hz overclock 540MHz
    // 1920x1200 @50Hz overclock 600MHz
    parameter x        = 800,  // pixels
    parameter y        = 600,  // pixels
    parameter f        = 60,   // Hz 60, 50, 30
    parameter xadjustf = 0,    // or to fine-tune f
    parameter yadjustf = 0,    // or to fine-tune f
    parameter c_ddr    = 1
) (
    input clk_25mhz,
    input [6:0] btn,
    output [7:0] led,
    output [3:0] gpdi_dp,
    output wifi_gpio0
);

  reg [7:0] r_i, g_i, b_i;
  initial r_i = 8'b11111111;
  initial g_i = 8'b11111111;
  initial b_i = 8'b00000000;


  reg fetch_next_pixel_signal;


  top_vgatest #(
      .x(x),
      .y(y),
      .f(f),
      .xadjustf(xadjustf),
      .yadjustf(yadjustf),
      .c_ddr(c_ddr),
  ) top_vgatest_instance (
      .clk_25mhz(clk_25mhz),
      .btn(btn),
      .led(led),
      .gpdi_dp(gpdi_dp),
      .wifi_gpio0(wifi_gpio0),
      .r_i(r_i),
      .g_i(g_i),
      .b_i(b_i),
      .fetch_next_pixel(fetch_next_pixel_signal),

  );

  reg [1:0] pixelState;
  initial pixelState = 2'b00;
  always @( posedge  fetch_next_pixel_signal) begin

    if (pixelState == 2'b00) begin
      r_i <= 8'b11111111;
      g_i <= 8'b00000000;
      b_i <= 8'b00000000;

      pixelState = 2'b01;
    end else if (pixelState == 2'b01) begin
      r_i <= 8'b00000000;
      g_i <= 8'b11111111;
      b_i <= 8'b00000000;
      pixelState = 2'b10;
    end else begin
      r_i <= 8'b00000000;
      g_i <= 8'b00000000;
      b_i <= 8'b11111111;
      pixelState = 2'b00;
    end
  end


endmodule
