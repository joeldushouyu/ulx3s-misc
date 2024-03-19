
// 32 bit output
// almost_full
// full
module usb_stream_in(input clk,
                     input rst_n,                 // clk max at 100 MHZ
                     input [2:0]master_mode,
                     input [31:0] data_out,
                     output reg PKTEND,
                     output reg SLOE,
                     output reg SLRD,
                     output reg SLCS,
                     output reg SLWR,
                     output reg  [1:0] A,
                     input FLAGA, 
                     input FLAGB,
                     output  [31:0] DQ,
                     // expose the current mode, for debug
                     output [1:0] current_stream_in_mode
                     );
    
    
    //parameter write_watermark = 4;
    //parameter delay_propagation =1'b1; //0 or 1, see end of page 38
    parameter master_mode_loopback = 3'b000;
    parameter master_mode_stream_out = 3'b001;
    parameter master_mode_stream_in = 3'b010;
    parameter master_mode_ZLP = 3'b011;
    parameter master_mode_partial = 3'b100;
    parameter master_mode_idle = 3'b101;
    

    // // $display("Water mark is %h", write_watermark);
    // // the number of actual available write  32 bit data still can write after FLAGB->0
    // // equation from from Section 9.3 (1)
    // localparam write_delay = write_watermark > 4 ? write_watermark   - 4  : 0;
    // reg [3:0] delayCounter;
    //reg delayCounterChangeState; // when delay progation, dely change state from 
        // stream_in_write_wr to idle by 1 clock
    
    // state variable
    parameter stream_in_idle           = 2'b00;
    parameter stream_in_wait_flagb     = 2'b01;
    parameter stream_in_write          = 2'b10;
    parameter stream_in_write_wr_delay = 2'b11;
    
    
    reg [1:0] current_state, next_state;
    // // lets write the data
    // for(integer i = 0; i < 32; i = i+1)begin
    //     assign DQ[i] = data_out[i];
    // end  
    assign DQ = data_out;
    
    always @(posedge clk )begin
        
        // if (~rst_n )begin
        //     // reset happens
        //     //SLWR          <= 1'b1;
        //     current_state <= stream_in_idle;
        //     //$strobe("Current state is %2b ", current_stream_in_mode);
        //     end 
        // // else if(master_mode != master_mode_stream_in)begin
        //     current_state <= stream_in_idle;
        // end
        // else begin
            // normal, update variable if needed
            current_state <= next_state;

            // // sequential logic
            // if(delayCounter >0  ) begin
            //     delayCounter <= delayCounter-1;
            // end 
        // end

    end
    
    
    // the combinational  output logic part
    // expose current mode for testing and calling
    assign current_stream_in_mode = current_state;
    always @(*) begin

        case(current_state)
            stream_in_idle:begin
                PKTEND =1'b1;
                SLOE = 1'b1;
                SLRD = 1'b1;
                SLCS = 1'b0;
                SLWR = 1'b1;
                A = 2'b00;
            end

            stream_in_wait_flagb: begin
                // do nothing
            end

            stream_in_write: begin
                PKTEND =1'b1;
                SLOE = 1'b1;
                SLRD = 1'b1;
                SLCS = 1'b0;
                SLWR = 1'b0;
                A = 2'b00;
            end

            stream_in_write_wr_delay: begin
                PKTEND =1'b1;
                SLOE = 1'b1;
                SLRD = 1'b1;
                SLCS = 1'b0;
                SLWR = 1'b1;
                A = 2'b00;
            end

        endcase

    end
    
    
    // now, the change state things happens
    always @(*)begin
        next_state = current_state;
        case(current_state)
            stream_in_idle: begin

                if(FLAGA == 1 && master_mode == master_mode_stream_in)begin
                    next_state = stream_in_wait_flagb;
                end
            end
            stream_in_wait_flagb: begin

                if(FLAGB == 1) begin
                    next_state = stream_in_write;
                end
            end
            /// since the flag is set to 4, leave no room for delay
            stream_in_write: begin
                if(FLAGB == 0)begin
                    next_state = stream_in_idle;
                end
            end
            default: begin
                next_state = stream_in_idle;
            end
        endcase
    end
    
endmodule
