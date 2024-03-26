module sdram_fifo_interface 
                             (input sram_clk,
                              rst_n,
                              //input[2:0] key,                         //key[0] for burst writing, key[1] for burst reading, press key[2] along with key[0] to inject 10240 errors to be displayed on the seven-segment LEDs
                              output sdram_clk,
                              output sdram_cke,
                              output sdram_cs_n,
                              sdram_ras_n,
                              sdram_cas_n,
                              sdram_we_n,
                              output[12:0] sdram_addr,
                              output[1:0] sdram_ba,
                              output[1:0] sdram_dqm,
                              inout[15:0] sdram_dq,
                              input reg [2:0] user_interface_command, // command from user
                              input reg [14:0] f_addr,                //14:2 = row(13), 1:0 = bank(2), no need for column address since full page mode will always start from zero and end with 512 words
                              input reg[15:0] f2s_data,
                              output wire[15:0] s2f_data,
                              output r_fifo,
                              output w_fifo,
                              output in_idle,
                              );
    wire CLK_OUT;
    assign CLK_OUT = sram_clk; // input 165.5MHZ
    
    
    parameter idleToFifo = 3'b000;
    parameter readFromFifo = 3'b001;
    parameter writeToFifo = 3'b010;


    assign r_fifo = (state_q == write_burst) ;
    assign w_fifo = (state_q == read_burst);
    assign in_idle = (state_q == idle);
    //FSM state declarationss
    localparam idle = 0,
    //write test-data to all addresses
    new_write = 1,
    write_burst = 2,
    //read test-data written to all addresses
    new_read = 3,
    read_burst = 4;
    
    reg[2:0] state_q       = idle,state_d;
    reg[9:0] burst_index_q = 0,burst_index_d;
    reg[7:0] led_q         = 0,led_d;
    
    reg rw,rw_en;
    
    wire ready,s2f_data_valid,f2s_data_valid;
    
    wire key0_tick,key1_tick;
    wire[5:0] in0,in1,in2,in3,in4,in5; //format: {dp,char[4:0]} , dp is active high
    
    (*KEEP = "TRUE"*)reg[36:0] counter_q,index_q,index_d; //counter_q increments until 1 second(165_000_000 clk cycles). Index_q holds the number of words read/written(check the value at chipscope)
    //Memory Bandwidth: index_q*2 = X bytes/seconds
    // RESULT: 190MB/s (100MHz with t_CL = 2)
    // RESULT: 316MB/s (165MHz clk with t_CL = 3)
    
    //register operations
    always @(posedge CLK_OUT,negedge rst_n) begin
        if (!rst_n) begin
            state_q       <= 0;
            burst_index_q <= 0;
            led_q         <= 0;
            counter_q     <= 0;
            index_q       <= 0;
        end
        else begin
            state_q       <= state_d;
            burst_index_q <= burst_index_d;
            led_q         <= led_d;
            counter_q <= (state_q == idle) ?0:counter_q+1'b1;
            index_q <= index_d;
        end
    end
    
    //FSM next-state logic
    always @* begin
        state_d = state_q;
        
        burst_index_d = burst_index_q;
        led_d         = led_q;
        rw            = 0;
        rw_en         = 0;
        index_d       = index_q;
        
        case(state_q)
            idle: begin  //wait until either button is toggled
                
                burst_index_d = 0;
                if (user_interface_command == readFromFifo)begin
                    state_d = new_read;
                    index_d = 0;
                end
                else if (user_interface_command == writeToFifo)begin
                    state_d = new_write;
                    index_d = 0;
                end
                // if (key[0]) begin
                    //     state_d = new_write;
                    //     index_d = 0;
                // end
                // if (key[1]) begin
                    //     state_d = new_read;
                    //     error_d = 0;
                    //     index_d = 0;
                // end
            end
            new_write: if (ready) begin  //write a deterministic data to all possible addresses of sdram
                led_d[1]      = 1'b1;
                rw_en         = 1;
                rw            = 0;
                state_d       = write_burst;
                burst_index_d = 0;
 
                //TODO: rin from fifo = 1
            end
            write_burst: begin
                
                if (f2s_data_valid) begin
                    burst_index_d = burst_index_q+1; //track the number of already bursted data
                    index_d       = index_q+1'b1; //holds the total number of words written to sdram
                end
                else if (burst_index_q == 512) begin //last data must be 512th(for full page mode), since index starts at zero, the 512th is expected to have deasserted f2s_data_valid
                    
                    led_d[1:0] = 2'b11;
                    state_d    = idle;
             
                    
                end
            end
            new_read: if (ready) begin //read each data from all addresses and test if it matches the deterministic data we assigned earlier
                led_d[2]      = 1'b1;
                rw_en         = 1;
                rw            = 1;
                state_d       = read_burst;
                burst_index_d = 0;
                //TODO: win from fifo
             
            end
            read_burst: begin
                if (s2f_data_valid) begin
                    //if (s2f_data! = f_addr+burst_index_q) error_d = error_q+1'b1; //count the errors in which the read output does not match the expected assigned data
                    burst_index_d   = burst_index_q+1;
                    index_d         = index_q+1'b1; //holds the total number of words read from sdram
                end
                else if (burst_index_q == 512) begin
                    led_d[3:0] = 4'b1111; //all leds on after successfull write
                    state_d    = idle;
              
                end
            end
            default: state_d = idle;
        endcase
    end
    
    // assign led = led_q;
    
    //module instantiations
    sdram_controller m0
    (
    //fpga to controller
    .clk(CLK_OUT), //clk = 100MHz
    .rst_n(rst_n),
    .rw(rw), // 1:read , 0:write
    .rw_en(rw_en), //must be asserted before read/write
    .f_addr(f_addr), //23:11 = row  , 10:9 = bank  , no need for column address since full page mode will always start from zero and end with 511 words
    .f2s_data(f2s_data), //fpga-to-sdram data
    .s2f_data(s2f_data), //sdram to fpga data
    .s2f_data_valid(s2f_data_valid),  //asserts while  burst-reading(data is available at output UNTIL the next rising edge)
    .f2s_data_valid(f2s_data_valid), //asserts while burst-writing(data must be available at input BEFORE the next rising edge)
    .ready(ready), //"1" if sdram is available for nxt read/write operation
    //controller to sdram
    .s_clk(sdram_clk),
    .s_cke(sdram_cke),
    .s_cs_n(sdram_csn),
    .s_ras_n(sdram_rasn),
    .s_cas_n(sdram_casn),
    .s_we_n(sdram_wen),
    .s_addr(sdram_a),
    .s_ba(sdram_ba),
    .LDQM(sdram_dqm[0]),
    .HDQM(sdram_dqm[1]),
    .s_dq(sdram_d)
    );
    
    
endmodule
