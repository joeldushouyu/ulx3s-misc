module frameBufferTest; 
reg updated = 1;
reg reset = 1; wire writeEnable; wire readEnable; 
wire endOfReadOut; wire endOfWriteOut; wire [1:0]nextReadOut; wire [1:0]nextWriteOut; wire [1:0]currentReadOut; wire [1:0]currentWriteOut;
reg endOfReadIn; reg endOfWriteIn; reg [1:0]nextReadIn; reg [1:0]nextWriteIn; reg[1:0] currentReadIn; reg [1:0]currentWriteIn; 
frameBuffer T1(.reset(reset), .writeEnable(writeEnable), .readEnable(readEnable), .endOfReadIn(endOfReadIn), .endOfWriteIn(endOfWriteIn), .nextReadIn(nextReadIn), .nextWriteIn(nextWriteIn), .currentReadIn(currentReadIn), .currentWriteIn(currentWriteIn),.updated(updated));


initial begin
    //     $monitor("Time %t:  writeEnable %b  readEnable %b endOfReadOut  %b  endOfWriteOut %b currentReadOut %d  nextReadOut %d currentWriteOut %d nextWriteOut  %d updated %b ",
    // $time,writeEnable,readEnable,endOfReadOut,endOfWriteOut, currentReadOut, nextReadOut, currentWriteOut,nextWriteOut, updated);
end

always begin 

    //#1 endOfReadOut = 1; endOfReadOut = 0;

    #1 reset       = 0; updated = ~updated;
    #10 reset       = 1;updated = ~updated;

    // $display("Read faster than write");
    // // // // currentRead should always be equal to nextRead
    // #1  endOfReadIn = 1;  // trigger the logic block, remain the same
    // #1  endOfReadIn = 0; 
    // // should swap 
    // #1  endOfReadIn = 0; endOfWriteIn = 0; currentReadIn = 1; nextReadIn = 2; currentWriteIn = 0; nextWriteIn = 3; 
    // //  ableToWrite 1 ableToRead 1 endOfRead 0  endOfWrite 0 $$$ currentRead 2  nextRead 2 currentWrite 0 nextWrite  1 
    // #1  endOfReadIn = 1; // trigger the logic block, remain the same

    // #1  endOfReadIn = 0;
    // #1  endOfReadIn = 1;  // trigger the logic block, remain the same


    // $display("Write faster than read"); 
    // #1  endOfReadIn = 0; endOfWriteIn = 0; currentReadIn = 0; nextReadIn = 0; currentWriteIn = 1; nextWriteIn = 2;  // the default

    // #1 endOfWriteIn = 1;  
    // // index 0 is read, index 1 is finished writing, index 2 is waiting
    // //  ableToWrite 1 ableToRead 1 endOfRead 0  endOfWrite 0 $$$ currentRead 0 nextRead 1 currentWrite 2 nextWrite  2   

    // //cannot write due to buffer is full
    // #1 endOfWriteIn = 0; 
    // #1 endOfWriteIn = 1; 
    // // index 0 is read, index 1 is finished writing, index 2 is waiting
    // //  ableToWrite 0 ableToRead 1 endOfRead 0  endOfWrite 0 $$$ currentRead 0 nextRead 1 currentWrite 2 nextWrite  2   // disable the write for now, until read new frame

    // //now, imaging that an read come in
    // // this update the write page
    // #1 endOfWriteIn = 0; endOfReadIn = 1;
    // // index 0 is waiting, index 1 is finished writing, index 2 is waiting
    // //  ableToWrite 0 ableToRead 1 endOfRead 0  endOfWrite 0 $$$ currentRead 1 nextRead 1 currentWrite 2 nextWrite  0  

    // #1 endOfReadIn = 0;
    // #1 endOfReadIn = 1; // another read, this will use up that last available image buffer
    //                     //NOTE: the writeEnable is still disabled, it only enable when endOfWrite is trigger to pin the block
    // //  ableToWrite 0 ableToRead 1 endOfRead 0  endOfWrite 0 $$$ currentRead 1 nextRead 1 currentWrite 2 nextWrite  0 



    // //now, lets trigger a endOFWrite( imaging a write always block find that ableToWrite = 0 and end the block by set endOfWrite = 1)
    // // this only set ableToWrite = 1
    // #1 endOfWriteIn = 1;
    // // index 0 is waiting, index 1 is finished writing, index 2 is waiting
    // //  ableToWrite 1 ableToRead 1 endOfRead 0  endOfWrite 0 $$$ currentRead 1 nextRead 1 currentWrite 2 nextWrite  0 

    // // another write will actually update the currentWRite and nextWrite
    // #1 endOfWriteIn = 0; 
    // #1 endOfWriteIn = 1;
    // //  ableToWrite 1 ableToRead 1 endOfRead 0  endOfWrite 0 $$$ currentRead 1 nextRead 2 currentWrite 0 nextWrite  0 

    // // calling one more write, will make the ableToWrite be disabled, since no more frame to write
    // #1 endOfWriteIn = 0; 
    // #1 endOfWriteIn = 1;
    // //  ableToWrite 0 ableToRead 1 endOfRead 0  endOfWrite 0 $$$ currentRead 1 nextRead 2 currentWrite 0 nextWrite  0 


    //$display("both read/write at same time"); 
    //!:: when both read is same
    #1  endOfReadIn = 0; endOfWriteIn = 0; currentReadIn = 0; nextReadIn = 0; currentWriteIn = 1; nextWriteIn = 2;  // the default

    // 0: read  1: write 2: waiting for write
    #10  endOfWriteIn = 1;endOfReadIn = 1;
    //#1 updated=~updated;
    // read should execute before write due to the blocking if statement structure
    // //  ableToWrite 1 ableToRead 1 endOfRead 0  endOfWrite 0 $$$ currentRead 0 nextRead 1 currentWrite 2 nextWrite  0

    //expected 0: wait 1: read 2:

    

    //2:: when both write is same

    // 3: when both write & read is different
    $finish;

    

end





endmodule
