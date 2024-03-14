

// // module frameBuffer(input reset,
// //                    inout reg endOfRead,
// //                    inout reg endOfWrite,
// //                    output reg writeEnable,
// //                    output reg readEnable,
// //                    output reg [23:0] readPixel,
// //                    input reg [23:0] writePixel,
// //                    input reg [20:0] readPixelX,
// //                    input reg [20:0] readPixelY,
// //                    input reg readPixelSignal,
// //                    input reg [20:0] writePixelX,
// //                    input reg [20:0] writePixelY,
// //                    input reg writePixelSignal,
// //                    );
    
    
    
    
// //     parameter x                = 1920; parameter y                = 1080;
// //     localparam totalBufferSize = 3;// 3
// //                                   // for the buffer logic
// //     localparam readable  = 2'b00; // frame is designed for read; AKA with buffer image
// //     localparam writeable = 2'b01; // frame is designed for write new data; AKA the data in the buffer is already displayed
    
// //     // for output to user
// //     assign writeEnable = ableToWrite;
// //     assign readEnable  = ableToRead;
    
    
// //     reg [23:0] buffers [0:totalBufferSize - 1][0: x*y-1];// 3 frame buffer RGB888
    
// //     reg [1:0] frameState [0:totalBufferSize - 1];
    
// //     reg alreadyReset = 0;
// //     reg ableToRead, ableToWrite;
// //     reg nextReadable, nextWriteable, currentRead, currentWrite;
    
    
    
// //     always @(negedge reset) begin
// //         //$display("Trigger reset");
// //         ableToRead  = 0;
// //         ableToWrite = 0;
        
// //         //TODO: some reason did not work
// //         // //TODO: initalize a frame?
// //         // for (int i = 0; i < x*y; i = i+1) begin
            
// //         //     buffers[0][i] = 24'b0;
// //         // end
        
// //         //buffers[0][1] = 24'b11111111;
// //         //   buffers[0][1] = 24'b0;


// //         //TODO: somehow the forloop did not work
// //         frameState[0] = readable;
// //         frameState[1] = writeable;
// //         frameState[2] = writeable;
        
// //                            // This 2 variable keep for fast response
// //                            // assume buffer is atleast > = 3
// //         nextReadable  = 0; // index 0;
// //         nextWriteable = 2;
// //         currentRead   = 0;
// //         currentWrite  = 1;
        
        
// //         endOfRead    = 0;
// //         endOfWrite   = 0;
// //         alreadyReset = 1;
// //         ableToRead   = 1;
// //         ableToWrite  = 1;
// //     end
    
    
    
    
    
// //     // always block for normal reading
// //     // for the reading and writing logic
// //     // caller responsibel for trigger endOfRead
// //     always @(readPixelSignal) begin
// //         if (ableToRead == 1 && alreadyReset == 1) begin
// //             // assume user passed in correct
            
// //             readPixel = buffers[currentRead][readPixelX + readPixelY*x];
// //         end
// //     end
    
    
// //     always @(writePixelSignal) begin

// //         if(ableToWrite == 1 && alreadyReset == 1) begin
// //             buffers[currentWrite][writePixelX + writePixelY*x] = writePixel;
// //         end
// //     end
    
    
// //     // always block for buffer
    
// //     reg readBreakCondition;
// //     always @(posedge endOfRead) begin
// //         if (alreadyReset == 1) begin
            
            
// //             // want a new readFramel
            
// //             ableToRead              = 0;
// //             frameState[currentRead] = writeable;
// //                                         //nextReadable frame should already be readable
// //             currentRead = nextReadable; // need be blocking, to ensure the always neddege endOfRead only triggers after update the currentRead
            
// //             ableToRead = 1;
            
// //             // don't really need to worry about the write block
// //             // the write always block only adds more, will not reduce
// //             // looking for a readable frame

// //             // //TODO: did not work

// //             // readBreakCondition = 1;
// //             // for(integer i = 0; i < totalBufferSize; i++)begin
// //             //     if (frameState[i] == readable && readBreakCondition == 1) begin
// //             //         nextReadable = i;
// //             //         readBreakCondition = 0;
// //             //     end
// //             // end


// //             if(frameState[0] == readable && nextReadable != 0)begin
// //                 nextReadable = 0;
// //             end else if (frameState[1] == readable && nextReadable != 1) begin
// //                 nextReadable  = 1;
// //             end else if (frameState[2] == readable && nextReadable != 2) begin
// //                 nextReadable  = 2;
// //             end else begin
// //                 // means no more frame that is still readable
// //                 // keep the nextReadable value as it is 
// //             end

// //             endOfRead = 0;
// //             // if nothing, nextReadable still keep to be the same thing
// //         end
        
        
        
        
// //         // $strobe("ableToWrite %b  ableToRead %b endOfRead %b  endOfWrite %b $$$ currentRead %d  nextRead %d currentWrite %d nextWrite  %d ",
// //         // ableToWrite,ableToRead,endOfRead,endOfWrite, currentRead, nextReadable, currentWrite,nextWriteable);
// //     end
    
    
    
// //     reg writeBreakCondition;
// //     always @(posedge endOfWrite) begin
        
        
// //         // the write part
// //         if (alreadyReset == 1) begin
// //             // assume that both nextWriteable and currentWrite 's bufferState is all in writeable
            
// //             if (nextWriteable != currentWrite) begin
// //                 // means there still a frame able to write
// //                 if (ableToWrite == 1)begin
// //                     // the writting flow is never interrupted
// //                     frameState[currentWrite] = readable;
// //                     currentWrite             = nextWriteable;
                    
// //                     // find new frame for write
// //                     end else begin
// //                     // means the writting is blocked due to reading not fast enough
// //                     // which means currentWrite is actually empty
// //                     // do not update the currentWrite to readable
// //                     // use currentWrite for writeable
// //                     ableToWrite = 1;
// //                 end
                
// //                 end else begin
// //                                  // means no frame for write
// //                 ableToWrite = 0; // forbid first
// //                 // try find new frame for write
// //             end
            
// //             // //TODO: did not work
// //             // // now, try to look for a new frame
// //             // writeBreakCondition = 1;
// //             // for(integer i = 0; i < totalBufferSize; i++)begin
                
// //             //     if (frameState[i] == writeable && writeBreakCondition == 1)begin
// //             //         nextWriteable = i;
// //             //         writeBreakCondition = 0;
// //             //     end
// //             // end
            
// //             if(frameState[0] == writeable && nextWriteable != 0 )begin
// //                 nextWriteable = 0;
// //             end else if (frameState[1] == writeable  && nextWriteable != 1 ) begin
// //                 nextWriteable  = 1;
// //             end else if (frameState[2] == writeable  && nextWriteable != 2) begin
// //                 nextWriteable  = 2;
// //             end else begin
// //                 // means no more frame that is still readable
// //                 // keep the nextWriteable value as it is 
// //             end

            
            
// //             endOfWrite = 0; // start the write process
            
// //         end
        
// //         // $strobe("ableToWrite %b  ableToRead %b endOfRead %b  endOfWrite %b $$$ currentRead %d  nextRead %d currentWrite %d nextWrite  %d ",
// //         // ableToWrite,ableToRead,endOfRead,endOfWrite, currentRead, nextReadable, currentWrite,nextWriteable);
// //     end
    
// // endmodule





module frameBuffer(input reset,
                   input wire endOfRead,
                   input wire endOfWrite,
                   output reg ableToWrite,
                   output reg ableToRead,
                   output reg [23:0] readPixel,
                   input wire [23:0] writePixel,
                   input wire [20:0] readPixelX,
                   input wire [20:0] readPixelY,
                   input wire readPixelSignal,
                   input wire [20:0] writePixelX,
                   input wire [20:0] writePixelY,
                   input wire writePixelSignal
                   );
    
    
    
    
    parameter x                = 1920; parameter y                = 1080;
    localparam pixelNumber = 2073600;
    localparam totalBufferSize = 3;// 3
                                  // for the buffer logic
    localparam readable  = 2'b00; // frame is designed for read; AKA with buffer image
    localparam writeable = 2'b01; // frame is designed for write new data; AKA the data in the buffer is already displayed
    
    // for output to user
    // assign writeEnable = ableToWrite;
    // assign readEnable  = ableToRead;
    
    
    reg [23:0] buffers [0:totalBufferSize - 1][0: x*y-1];// 3 frame buffer RGB888
    
    reg [1:0] frameState [0:totalBufferSize - 1];
    
    reg alreadyReset = 0;
    // reg ableToRead, ableToWrite;
    reg nextReadable, nextWriteable, currentRead, currentWrite;
    
    
    
    always @(negedge reset) begin
        //$display("Trigger reset");
         ableToRead  = 0;
         ableToWrite = 0;
        
        //TODO: some reason did not work
        // //TODO: initalize a frame?
        for (integer i = 0; i < x*y; i = i+1) begin
            
            buffers[0][i] = 16777215;
        end
        
        y = y+10;
        y= y/x; 
        // buffers[0][1] = 24'b11111111;
        //   buffers[0][1] = 24'b0;
        //$readmemh("whiteScreen-640-400.mem", buffers[0]);
        // initial buffers[0][1] = 16777215;

        //TODO: somehow the forloop did not work
         frameState[0] = readable;
         frameState[1] = writeable;
         frameState[2] = writeable;
        
                           // This 2 variable keep for fast response
                           // assume buffer is atleast > = 3
         nextReadable  = 0; // index 0;
         nextWriteable = 2;
         currentRead   = 0;
         currentWrite  = 1;
        
        
        // endOfRead    = 0;
        // endOfWrite   = 0;
         alreadyReset = 1;
         ableToRead   = 1;
         ableToWrite  = 1;
    end
    
    
    // always block for normal reading
    // for the reading and writing logic
    // caller responsibel for trigger endOfRead
    always @(readPixelSignal) begin
        if (ableToRead == 1 && alreadyReset == 1) begin
            // assume user passed in correct
            
            readPixel = buffers[currentRead][readPixelX + readPixelY*x];
        end
    end
    
    
    // always @(writePixelSignal) begin

    //     if(ableToWrite == 1 && alreadyReset == 1) begin
    //         buffers[currentWrite][writePixelX + writePixelY*x] = writePixel;
    //     end
    // end
    
    
    // always block for buffer
    
    // reg readBreakCondition;
    // always @(posedge endOfRead) begin
    //     if (alreadyReset == 1) begin
            
            
    //         // want a new readFramel
            
    //         ableToRead              = 0;
    //         frameState[currentRead] = writeable;
    //                                     //nextReadable frame should already be readable
    //         currentRead = nextReadable; // need be blocking, to ensure the always neddege endOfRead only triggers after update the currentRead
            
    //         ableToRead = 1;
            
    //         // don't really need to worry about the write block
    //         // the write always block only adds more, will not reduce
    //         // looking for a readable frame

    //         // //TODO: did not work

    //         // readBreakCondition = 1;
    //         // for(integer i = 0; i < totalBufferSize; i++)begin
    //         //     if (frameState[i] == readable && readBreakCondition == 1) begin
    //         //         nextReadable = i;
    //         //         readBreakCondition = 0;
    //         //     end
    //         // end


    //         if(frameState[0] == readable && nextReadable != 0)begin
    //             nextReadable = 0;
    //         end else if (frameState[1] == readable && nextReadable != 1) begin
    //             nextReadable  = 1;
    //         end else if (frameState[2] == readable && nextReadable != 2) begin
    //             nextReadable  = 2;
    //         end else begin
    //             // means no more frame that is still readable
    //             // keep the nextReadable value as it is 
    //         end

    //         //endOfRead = 0;
    //         // if nothing, nextReadable still keep to be the same thing
    //     end
        
        
        
        
    //     // $strobe("ableToWrite %b  ableToRead %b endOfRead %b  endOfWrite %b $$$ currentRead %d  nextRead %d currentWrite %d nextWrite  %d ",
    //     // ableToWrite,ableToRead,endOfRead,endOfWrite, currentRead, nextReadable, currentWrite,nextWriteable);
    // end
    
    
    
    // reg writeBreakCondition;
    // always @(posedge endOfWrite) begin
        
        
    //     // the write part
    //     if (alreadyReset == 1) begin
    //         // assume that both nextWriteable and currentWrite 's bufferState is all in writeable
            
    //         if (nextWriteable != currentWrite) begin
    //             // means there still a frame able to write
    //             if (ableToWrite == 1)begin
    //                 // the writting flow is never interrupted
    //                 frameState[currentWrite] = readable;
    //                 currentWrite             = nextWriteable;
                    
    //                 // find new frame for write
    //                 end else begin
    //                 // means the writting is blocked due to reading not fast enough
    //                 // which means currentWrite is actually empty
    //                 // do not update the currentWrite to readable
    //                 // use currentWrite for writeable
    //                 ableToWrite = 1;
    //             end
                
    //             end else begin
    //                              // means no frame for write
    //             ableToWrite = 0; // forbid first
    //             // try find new frame for write
    //         end
            
    //         // //TODO: did not work
    //         // // now, try to look for a new frame
    //         // writeBreakCondition = 1;
    //         // for(integer i = 0; i < totalBufferSize; i++)begin
                
    //         //     if (frameState[i] == writeable && writeBreakCondition == 1)begin
    //         //         nextWriteable = i;
    //         //         writeBreakCondition = 0;
    //         //     end
    //         // end
            
    //         if(frameState[0] == writeable && nextWriteable != 0 )begin
    //             nextWriteable = 0;
    //         end else if (frameState[1] == writeable  && nextWriteable != 1 ) begin
    //             nextWriteable  = 1;
    //         end else if (frameState[2] == writeable  && nextWriteable != 2) begin
    //             nextWriteable  = 2;
    //         end else begin
    //             // means no more frame that is still readable
    //             // keep the nextWriteable value as it is 
    //         end

            
            
    //         //endOfWrite = 0; // start the write process
            
    //     end
        
    //     // $strobe("ableToWrite %b  ableToRead %b endOfRead %b  endOfWrite %b $$$ currentRead %d  nextRead %d currentWrite %d nextWrite  %d ",
    //     // ableToWrite,ableToRead,endOfRead,endOfWrite, currentRead, nextReadable, currentWrite,nextWriteable);
    // end
    
endmodule



// module frameBuffer(input reset,
//                    input wire endOfRead,
//                    input wire endOfWrite,
//                    output reg ableToWrite,
//                    output reg ableToRead,
//                    output reg [23:0] readPixel,
//                    input wire [23:0] writePixel,
//                    input wire [20:0] readPixelX,
//                    input wire [20:0] readPixelY,
//                    input wire readPixelSignal,
//                    input wire [20:0] writePixelX,
//                    input wire [20:0] writePixelY,
//                    input wire writePixelSignal
//                    );
    
//     parameter x                = 1920; parameter y                = 1080;
//     localparam totalBufferSize = 3;
//     localparam readable  = 2'b00; // frame is designed for read; AKA with buffer image
//     localparam writeable = 2'b01; // frame is designed for write new data; AKA the data in the buffer is already displayed
    
//     reg [23:0] buffers [0:totalBufferSize - 1][0: x*y-1];// 3 frame buffer RGB888
//     reg [1:0] frameState [0:totalBufferSize - 1];
    
//     reg alreadyReset = 0;
//     reg nextReadable, nextWriteable, currentRead, currentWrite;
    
    
    
//     always @(negedge reset) begin
//         //$display("Trigger reset");
//          ableToRead  = 0;
//          ableToWrite = 0;
        
//         //TODO: some reason did not work
//         // //TODO: initalize a frame?
//         // for (integer i = 0; i < x*y; i = i+1) begin
            
//         //     buffers[0][i] = 16777215;
//         // end

//         // $readmemh("whiteScreen-1266-768.mem", buffers[0]);
    
//          frameState[0] = readable;
//          frameState[1] = writeable;
//          frameState[2] = writeable;
        

//          nextReadable  = 0; // index 0;
//          nextWriteable = 2;
//          currentRead   = 0;
//          currentWrite  = 1;
        
//          alreadyReset = 1;
//          ableToRead   = 1;
//          ableToWrite  = 1;
//     end
    
    
//     // always block for normal reading
//     // for the reading and writing logic
//     // caller responsibel for trigger endOfRead
//     always @(readPixelSignal) begin
//         if (ableToRead == 1 && alreadyReset == 1) begin
//             // assume user passed in correct
            
//             readPixel = buffers[currentRead][readPixelX + readPixelY*x];
//         end
//     end
    
// endmodule