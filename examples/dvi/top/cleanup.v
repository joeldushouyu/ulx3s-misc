
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
    localparam totalBufferSize = 3;
    localparam readable  = 2'b00; // frame is designed for read; AKA with buffer image
    localparam writeable = 2'b01; // frame is designed for write new data; AKA the data in the buffer is already displayed
    
    reg [23:0] buffers [0:totalBufferSize - 1][0: x*y-1];// 3 frame buffer RGB888
    reg [1:0] frameState [0:totalBufferSize - 1];
    
    reg alreadyReset = 0;
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
    
         frameState[0] = readable;
         frameState[1] = writeable;
         frameState[2] = writeable;
        

         nextReadable  = 0; // index 0;
         nextWriteable = 2;
         currentRead   = 0;
         currentWrite  = 1;
        
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
    
endmodule
