module usb_stream_out_test;


parameter water_mark = 4;


reg clk = 0;

reg rst_n;
reg [2:0] master_mode;
wire  [31:0] data_in;
wire [1:0] A;
reg FLAGC,  FLAGD;
wire PKTEND, SLOE, SLRD, SLCS, SLWR;
wire [2:0] current_stream_out_mode;
reg [31:0] DQ;


usb_stream_out #(.read_watermark(4)) testModule(
    .clk(clk),
    .rst_n(rst_n),
    .master_mode(master_mode),
    .data_in(data_in),
    .PKTEND(PKTEND),
    .SLOE(SLOE),
    .SLRD(SLRD),
    .SLCS(SLCS),
    .SLWR(SLWR),
    .A(A),
    .FLAGC(FLAGC),
    .FLAGD(FLAGD),
    .DQ(DQ),
    .current_stream_out_mode(current_stream_out_mode)
);




initial begin 

    master_mode = 3'b101;
    rst_n = 1;
    DQ = 32'h100;
    FLAGC = 0;
    FLAGD = 0;

    $monitor("State: %3b PKTEND %b SLOE %b SLRD %b SLCS %b SLWR %b A %d dataRead %h"
    , current_stream_out_mode, PKTEND, SLOE, SLRD, SLCS, SLWR, A, data_in);
    // $monitor(" PKTEND %b SLOE %b SLRD %b SLCS %b SLWR %b A %d dataRead %h"
    // , PKTEND, SLOE, SLRD, SLCS, SLWR, A, data_in);
end



// // setupt clock
// always #1 clk = !clk;


always begin
    // // // reset should do nothing
    // #2 rst_n = 0;
    // #2 rst_n = 1;
    $display("START time :%d",$time);
    master_mode= 3'b001;
    // //State: 000 PKTEND 1 SLOE 1 SLRD 1 SLCS 0 SLWR 1 A 3 dataRead 100
    // #2 ;// wait
    // #2; //wait

    #1 rst_n = 0; // reset
    #1 rst_n = 1; 
    
    FLAGC = 1;


    // // // now, turn on flagC
    #1 clk = 0; 
    #1 clk = 1;
    // //State: 001 PKTEND 1 SLOE 1 SLRD 1 SLCS 0 SLWR 1 A 3 dataRead 100

    #2 clk = 0; 
    #2 clk = 1;
     // now, auto transition to state  stream_out_wait_flagd
    // //State: 010 PKTEND 1 SLOE 1 SLRD 1 SLCS 0 SLWR 1 A 3 dataRead 100

    FLAGD = 1; // stay in waith FLAGD
    #2 clk = 0; 
    #2 clk = 1;
    // //State: 011 PKTEND 1 SLOE 1 SLRD 1 SLCS 0 SLWR 1 A 3 dataRead 100

    DQ=32'h10; // for next read
    #2 clk = 0; 
    #2 clk = 1;
    // // //State: 011 PKTEND 1 SLOE 1 SLRD 0 SLCS 0 SLWR 1 A 3 dataRead 10

    DQ=32'h101; // for next read
    #2 clk = 0; 
    #2 clk = 1;
    // // //State: 011 PKTEND 1 SLOE 1 SLRD 0 SLCS 0 SLWR 1 A 3 dataRead 101


    DQ=32'h70;FLAGD = 0; // indicate that only 3 more data, since watermark=4;

    #2 clk = 0; 
    #2 clk = 1;
    // //State: 100 PKTEND 1 SLOE 1 SLRD 0 SLCS 0 SLWR 1 A 3 dataRead 70
    DQ=32'h22;    
    #2 clk = 0; 
    #2 clk = 1;    
    // SLRD should be 0
    //State: 100 PKTEND 1 SLOE 1 SLRD 0 SLCS 0 SLWR 1 A 3 dataRead 22
    $display("%b", SLRD);
    //TODO: okay here?
    DQ=32'h7F;
    #2 clk = 0; 

    #2 clk = 1;
    // // // //State: 101 PKTEND 1 SLOE 1 SLRD 1 SLCS 0 SLWR 1 A 3 dataRead 7F

    DQ=32'h80;
    FLAGC = 0;
    #2 clk = 0; 

    #2 clk = 1;
    // // // //State: 000 PKTEND 1 SLOE 1 SLRD 1 SLCS 0 SLWR 1 A 3 dataRead 80


    #2;
    $finish;

end

endmodule