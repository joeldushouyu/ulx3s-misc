module top_hex_demo
(
    input  wire clk_25mhz,
    input  wire [6:0] btn,
    output wire [7:0] led,
    output wire oled_csn,
    output wire oled_clk,
    output wire oled_mosi,
    output wire oled_dc,
    output wire oled_resn,
    output wire wifi_gpio0
);
    parameter C_color_bits = 16; // 8 or 16
    assign wifi_gpio0 = btn[0];

    wire clk = clk_25mhz;

    reg [127:0] R_display; // something to display
    always @(posedge clk)
    begin
      R_display[0] <= btn[0];
      R_display[4] <= btn[1];
      R_display[8] <= btn[2];
      R_display[12] <= btn[3];
      R_display[16] <= btn[4];
      R_display[20] <= btn[5];
      R_display[24] <= btn[6];
      R_display[127:64] <= R_display[127:64] + 1; // shown in next OLED row
    end

    wire [6:0] x;
    wire [5:0] y;
    wire next_pixel;
    wire [C_color_bits-1:0] color;

    hex_decoder_v
    #(
        .c_data_len(128),
        .c_font_file("hex_font.mem"),
        .c_grid_6x8(1),
        .c_color_bits(C_color_bits)
    )
    hex_decoder_v_inst
    (
        .clk(clk),
        .data(R_display),
        .x(x),
        .y(y),
        .color(color)
    );

generate
if(0)
begin
    wire o_csn, o_clk; 
    lcd_video
    #(
        .c_init_file("ssd1331_init_xflip_16bit.mem"),
        .c_init_size(90),
        .c_x_size(96),
        .c_y_size(64),
        .c_color_bits(C_color_bits)
    )
    lcd_video_inst
    (
        .clk(clk),
        .reset(~btn[0]),
        .x(x),
        .y(y),
        //.next_pixel(next_pixel),
        .color(color),
        .oled_csn(o_csn),
        .oled_clk(o_clk),
        .oled_mosi(oled_mosi),
        .oled_dc(oled_dc),
        .oled_resn(oled_resn)
    );
    assign oled_csn = ~o_csn;
    assign oled_clk = ~o_clk;
end
else
begin
    localparam C_init_file = C_color_bits < 12 ? 
                             "oled_init_xflip.mem" :
                             "oled_init_xflip_16bit.mem";
    oled_video
    #(
        .C_init_file(C_init_file),
        .C_init_size(44),
        .C_color_bits(C_color_bits)
    )
    oled_video_inst
    (
        .clk(clk),
        .x(x),
        .y(y),
        .next_pixel(next_pixel),
        .color(color),
        .oled_csn(oled_csn),
        .oled_clk(oled_clk),
        .oled_mosi(oled_mosi),
        .oled_dc(oled_dc),
        .oled_resn(oled_resn)
    );
end
endgenerate

endmodule
