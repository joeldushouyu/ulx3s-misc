



module usb_stream_in_test;

    

parameter water_mark = 4;
reg clk = 0;

reg rst_n; 
reg [2:0] master_mode;
reg [ 31:0] data_out;
wire [1:0] A;
reg FLAGA, FLAGB;
wire PKTEND, SLOE, SLRD, SLCS, SLWR;
wire [1:0] current_stream_in_mode;
wire [31:0] DQ;


usb_stream_in #(.write_watermark(water_mark)) testModule(
    .clk(clk),
    .rst_n(rst_n),
    .master_mode(master_mode),
    .data_out(data_out),
    .PKTEND(PKTEND),
    .SLOE(SLOE),
    .SLRD(SLRD),
    .SLCS(SLCS),
    .SLWR(SLWR),
    .A(A),
    .FLAGA(FLAGA),
    .FLAGB(FLAGB),
    .DQ(DQ),
    .current_stream_in_mode(current_stream_in_mode)


);

initial begin

    master_mode = 3'b101; // the idle state
    rst_n = 1;
    data_out = 32'd100;
    FLAGA = 0;
    FLAGB = 0;

    $monitor("State: %2b PKTEND %b SLOE %b SLRD %b SLCS %b SLWR %b A %2b dataReg %h" 
    , current_stream_in_mode, PKTEND, SLOE, SLRD, SLCS, SLWR, A, DQ);

end


// // setupt clock
// always #1 clk = !clk;

always begin


    #2 master_mode=3'b010; 
    //State: xx PKTEND x SLOE x SLRD x SLCS x SLWR 1 A 00 dataReg xx


    // #2 rst_n = 0; // reset
    // #2 rst_n = 1;
    FLAGA = 1; 
    //xxx State: 00 PKTEND 1 SLOE 1 SLRD 1 SLCS 0 SLWR 1 A 00 dataReg xx

    #2 clk = 0; 
    #2 clk = 1;
    //State: 00 PKTEND 1 SLOE 1 SLRD 1 SLCS 0 SLWR 1 A 00 dataReg x  // keep as above 

    FLAGB = 1; data_out = 31'h30;
    #2 clk = 0; 
    #2 clk = 1;
    // //State: 01 PKTEND 1 SLOE 1 SLRD 1 SLCS 0 SLWR 0 A 00 dataReg xx  // keep as above 
   
    data_out = 31'h100; // try another one
    #2 clk = 0; 
    #2 clk = 1;
    // //State: 10 PKTEND 1 SLOE 1 SLRD 1 SLCS 0 SLWR 0 A 00 dataReg 100  // keep as above 
    data_out = 31'h007F007F; // another one 
    #2 clk = 0; 
    #2 clk = 1;
    // // State: 10 PKTEND 1 SLOE 1 SLRD 1 SLCS 0 SLWR 0 A 00 dataReg 007F007F  // keep as above 
    // // // if( water_mark == 6) begin
    // // //     FLAGB = 0; //should trigger into the stream_in_wr
    // // //     #2 clk = 0; 
    // // //     #2 clk = 1;
    // // //     //State: 10 PKTEND 1 SLOE 1 SLRD 1 SLCS 0 SLWR 0 A 00 dataReg 10  // keep as above

    // // //     #2 clk = 0; 
    // // //     #2 clk = 1;
    // // //      // idealy, SLWR =0 and should match Figure 13.
    // // //             // TODO: but in page 38 tell us to only assert 1 time, so SLWR
    // // //     //State: 11 PKTEND 1 SLOE 1 SLRD 1 SLCS 1 SLWR 0 A 00 dataReg 10  // keep as above
    // // //     #2 clk = 0; 
    // // //     #2 clk = 1;
    // // //     // should stil be in 11 TODO:
    // // //     //State: 11 PKTEND 1 SLOE 1 SLRD 1 SLCS 1 SLWR 1 A 00 dataReg 0 // keep as above

    // // //     #2 clk = 0; 
    // // //     #2 clk = 1;

    // // //     //State: 00 PKTEND 1 SLOE 1 SLRD 1 SLCS 1 SLWR 1 A 00 dataReg 10 // keep as above


    // // // end
    // // // else if (water_mark == 4)begin

    // //     // SEE Figure 11 for more explanation
        FLAGB = 0; //should trigger into the stream_in_wr
        data_out = 31'h800080; // another one 
        #2 clk = 0; 
        #2 clk = 1;
    // //     //State: 10 PKTEND 1 SLOE 1 SLRD 1 SLCS 0 SLWR 1 A 00 dataReg 800080  // keep as above
        
        
        #2 clk = 0; 
        #2 clk = 1;
    //     // since water mark is 4,
    //     //State: 00 PKTEND 1 SLOE 1 SLRD 1 SLCS 0 SLWR 1 A 00 dataReg 800080  // keep as above

    // // end




    $finish;
end
endmodule

