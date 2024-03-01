// 32 bit input to the board

module usb_stream_out(input clk,
                      input rst_n,
                      input [2:0] master_mode,
                      output reg [31:0] data_in,
                      output reg PKTEND,
                      output reg SLOE,
                      output reg SLRD,
                      output reg SLCS,
                      output reg SLWR,
                      output reg [1:0] A,
                      input FLAGC,
                      input FLAGD,
                      input  [31:0] DQ,
                      output [2:0] current_stream_out_mode);
    
    parameter read_watermark         = 4;
    parameter master_mode_loopback   = 3'b000;
    parameter master_mode_stream_out = 3'b001;
    parameter master_mode_stream_in  = 3'b010;
    parameter master_mode_ZLP        = 3'b011;
    parameter master_mode_partial    = 3'b100;
    parameter master_mode_idle       = 3'b101;
    
    
                                                      // assume read_watermark need to makesure 3 remaing data for 32 bit
    localparam remain_read_data = read_watermark -1;  // see equation 2.a in section 9.3
    localparam ready_delay      = read_watermark - 3; // number of cycle delay for SLRD line
    //In Page 42, is says consider 1 cycle propagation delay
    
    reg [3:0] rd_oe_dealy_cnt; // delayCounter, delay for the SLRD line
    reg [3:0] oe_delay_cnt;    // remaing data after SLRD Line flip, aka remain_read_data - ready_delay
    
    
    
    localparam stream_out_idle             = 3'b000;
    localparam stream_out_flagc_rcvd       = 3'b001;
    localparam stream_out_wait_flagd       = 3'b010;
    localparam stream_out_read             = 3'b011;
    localparam stream_out_read_rd_oe_delay = 3'b100;
    localparam stream_out_read_oe_delay    = 3'b101;
    
    
    reg [2:0] current_state, next_state;
    
    
    always @(posedge clk or negedge rst_n) begin
        //$display("CLK :%d",$time);
        if (rst_n == 0 || master_mode != master_mode_stream_out)begin
            SLRD          <= 1'b1;
            next_state    <= stream_out_idle;
            current_state <= stream_out_idle;
        end
        else begin
            current_state <= next_state;
            
            //TODO: add the delay counter????
            //$display("rd_oe_dealy_cnt %d",rd_oe_dealy_cnt);
            if (rd_oe_dealy_cnt > 0)begin
                rd_oe_dealy_cnt <= rd_oe_dealy_cnt -1;
            end
                if (oe_delay_cnt > 0)begin
                    oe_delay_cnt <= oe_delay_cnt - 1;
                end
            
        end
    end
    
    //combinational output logic
    assign current_stream_out_mode = current_state;
    
    
    always @(*) begin
        if (current_state == stream_out_idle) begin
            
            PKTEND = 1; SLOE = 1;SLRD = 1; SLCS = 0;SLWR = 1;A = 2'b11;
        end
        else if (current_state == stream_out_flagc_rcvd)begin
            // do nothing, wait for 1 cycle to next
        end
            else if (current_state == stream_out_wait_flagd) begin
            // do nothing until FLAGD = 0;
            end
        else begin
            //$display("HERE");
            // means either in
            //stream_out_read
            //stream_out_read_rd_oe_delay
            //stream_out_read_oe_delay
            // lets get the data sing a for loop

            SLRD   = 1; // prevent random state when not initalized
            for(integer i = 0; i < 32; i = i+1)begin
                data_in[i] = DQ[i];
            end
            
            if (current_state == stream_out_read_oe_delay)begin
                // need to change SLRD = 1 in this case
                PKTEND = 1;
                SLOE   = 0;
                SLRD   = 1;
                SLCS   = 0;
                SLWR   = 1;
                A      = 2'b11;
                
            end
            else begin
                // stream_out_read or stream_out_read_rd_oe_delay
                PKTEND = 1;
                SLOE   = 0;
                SLRD   = 0;
                SLCS   = 0;
                SLWR   = 1;
                A      = 2'b11;
            end
            
        end
        
    end
    
    
    
    // change state logic
    always @(*)begin
        
        
        case (current_state)
            stream_out_idle: begin
                if (FLAGC == 1 && master_mode == master_mode_stream_out)begin
                    next_state = stream_out_flagc_rcvd;
                end
            end
            
            
            stream_out_flagc_rcvd: begin
                
                next_state = stream_out_wait_flagd;
            end
            
            stream_out_wait_flagd: begin
                if (FLAGD == 1)begin
                    next_state = stream_out_read;
                end
            end
            
            stream_out_read: begin
                
                if (FLAGD == 0) begin
                    
                    next_state      = stream_out_read_rd_oe_delay;
                    rd_oe_dealy_cnt = ready_delay; 
                end
            end
            
            stream_out_read_rd_oe_delay: begin
                if (rd_oe_dealy_cnt == 0)begin
                    next_state   = stream_out_read_oe_delay;
                    oe_delay_cnt = remain_read_data  -ready_delay;
                end
            end
            
            stream_out_read_oe_delay: begin
                
                if (oe_delay_cnt == 0)begin
                    next_state = stream_out_idle;
                end
            end

            default: begin
                next_state = stream_out_idle;
            end
        endcase
    end
endmodule
