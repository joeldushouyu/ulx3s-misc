module slaveFIFO2b_streamIN(
	input  reset_,
	input  clk_100,
	input  stream_in_mode_selected,
	input  flaga_d,
	input  flagb_d,
	input [31:0] data_for_output,
	output reg slwr_streamIN_,
	output [31:0] data_out_stream_in
);


reg [2:0]current_stream_in_state;
reg [2:0]next_stream_in_state;
reg [31:0]data_gen_stream_in;

//parameters for StreamIN mode state machine
parameter [2:0] stream_in_idle                    = 3'd0;
parameter [2:0] stream_in_wait_flagb              = 3'd1;
parameter [2:0] stream_in_write                   = 3'd2;
parameter [2:0] stream_in_write_wr_delay          = 3'd3;

assign slwr_streamIN_ = ((current_stream_in_state == stream_in_write) && (flagb_d == 1'b1)) ? 1'b0 : 1'b1;

//streamIN mode state machine
always @(posedge clk_100)begin
	// if(!reset_)begin 
	// 	current_stream_in_state <= stream_in_idle;
	// end else begin
		current_stream_in_state <= next_stream_in_state;
	// end	
end

//StreamIN mode state machine combo
always @(*)begin
	next_stream_in_state = current_stream_in_state;
	case(current_stream_in_state)
	stream_in_idle:begin
		if((stream_in_mode_selected) & (flaga_d == 1'b1))begin
			next_stream_in_state = stream_in_wait_flagb; 
		end else begin
			next_stream_in_state = stream_in_idle;
		end	
	end
	stream_in_wait_flagb :begin
		if (flagb_d == 1'b1)begin
			next_stream_in_state = stream_in_write; 
		end else begin
			next_stream_in_state = stream_in_wait_flagb; 
		end
	end
	stream_in_write:begin
		if(flagb_d == 1'b0)begin
			next_stream_in_state = stream_in_idle;
		end else begin
		 	next_stream_in_state = stream_in_write;
		end
	end
    stream_in_write_wr_delay:begin
			next_stream_in_state = stream_in_idle;
	end
	default:begin
		next_stream_in_state = stream_in_idle;
	end
	endcase
end
// //for coungint
// reg     [31: 0]                 cnt  ; 
// assign  add_cnt     =       slwr_streamIN_ == 1'b0;       
// assign  end_cnt     =       add_cnt && cnt == 4096-1;  

// //data generator counter for Partial, ZLP, StreamIN modes
// always @(posedge clk_100)begin
// 	// if(!reset_)begin 
// 	// 	data_gen_stream_in <= 32'd0;
// 	// end else 
// 	if((slwr_streamIN_ == 1'b0) & (stream_in_mode_selected)) begin
//         if(end_cnt) begin
//             cnt <= 0;
// 		end
//         else begin
//             cnt <= cnt + 1;
// 		end
// 	end else if (!stream_in_mode_selected) begin
// 		data_gen_stream_in <= 32'd0;
// 		cnt<= 0;
// 	end	
// end

assign data_out_stream_in = data_for_output;

endmodule
