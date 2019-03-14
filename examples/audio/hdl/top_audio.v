module top_audio
(
  input clk_25mhz,
  input [6:0] btn,
  output [7:0] led,
  output [3:0] audio_l, audio_r, audio_v,
  output wifi_gpio0
);
    parameter C_sinewave = 1; // 1-sinewave 0-trianglewave

    // wifi_gpio0=1 keeps board from rebooting
    // hold btn0 to let ESP32 take control over the board
    assign wifi_gpio0 = btn[0];

    wire [11:0] pcm;

    generate
    if(C_sinewave == 0)
    begin
    // triangle wave generator
    trianglewave
    #(
      .C_delay(6) // smaller value -> higher freq
    )
    trianglewave_instance
    (
      .clk(clk_25mhz),
      .pcm(pcm)
    );
    end
    endgenerate

    generate
    if(C_sinewave == 1)
    begin
    // sine wave generator
    sinewave
    #(
      .C_delay(10) // smaller value -> higher freq
    )
    sinewave_instance
    (
      .clk(clk_25mhz),
      .pcm(pcm)
    );
    end
    endgenerate
    
    assign led = pcm[11:4];

    // analog output to classic headphones
    wire [3:0] dac;
    dacpwm
    dacpwm_instance
    (
      .clk(clk_25mhz),
      .pcm(pcm),
      .dac(dac)
    );
    assign audio_l = dac;
    assign audio_r = dac;

    // digital output to SPDIF
    wire spdif;
    wire [23:0] pcm_24s;
    assign pcm_24s[23] = pcm[11];
    assign pcm_24s[22:11] = pcm;
    spdif_tx
    #(
      .C_clk_freq(25000000),
      .C_sample_freq(48000)
    )
    spdif_tx_instance
    (
      .clk(clk_25mhz),
      .data_in(pcm_24s),
      .spdif_out(spdif)
    );
    assign audio_v[1] = spdif; // 0.4V at SPDIF (standard: 0.6V MAX)

endmodule
