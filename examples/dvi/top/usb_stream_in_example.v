/************************************************************************
 * Author        : Wen Chunyang
 * Email         : 1494640955@qq.com
 * Create time   : 2018-12-21 10:57
 * Last modified : 2018-12-21 10:57
 * Filename      : usb.v
 * Description   : 
 * *********************************************************************/
module  usb_stream_in_example(
        input                   clk                             ,
        input                   pclk_in                         ,
        input                   rst_n                           ,
        //usb
        input                   flag_a                          ,
        input                   flag_b                          ,
        input                   flag_c                          ,
        input                   flag_d                          ,
        output  wire            pclk                            ,
        output  wire            slcs                            ,
        output  wire            sloe                            ,
        output  wire            slrd                            ,
        output  reg             slwr                            ,
        output  wire            pktend                          ,
        output  wire  [ 1: 0]   fifo_addr                       ,
        output   wire  [31: 0]   usb_data                        
)/*synthesis noprune*/;
//======================================================================\
//************** Define Parameter and Internal Signals *****************
//======================================================================/
parameter   IDLE            =   4'b0001                         ;
parameter   WRITE           =   4'b0010                         ;
reg     [ 3: 0]                 state_c /*synthesis preserve*/  ;
reg     [ 3: 0]                 state_n /*synthesis preserve*/  ;
//cnt
reg     [31: 0]                 cnt                             ;
wire                            add_cnt                         ;
wire                            end_cnt                         ;

wire    [31: 0]                 data_gen                        ;
//======================================================================\
//**************************** Main Code *******************************
//======================================================================/
assign  slrd        =   1'b1;
assign  fifo_addr   =   2'd0;
assign  sloe        =   1'b1;
assign  slcs        =   1'b0;
assign  pclk        =   pclk_in;
assign  pktend      =   1'b1;
assign  usb_data    =   (slwr) ? 32'dz : data_gen;

//state_c
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        state_c <= IDLE;
    end
    else begin
        state_c <= state_n;
    end
end

//state_n
always@(*)begin
    case(state_c)
        IDLE:begin
            if(flag_a && flag_b)begin
                state_n = WRITE;
            end
            else begin
                state_n = state_c;
            end
        end
        WRITE:begin
            if(flag_b == 1'b0)begin
                state_n = IDLE;
            end
            else begin
                state_n = state_c;
            end
        end
        default:begin
            state_n = IDLE;
        end
    endcase
end

//slwr
always  @(posedge clk or negedge rst_n)begin
    if(rst_n == 1'b0)begin
        slwr    <=  1'b1;
    end
    else if(state_n == WRITE)begin
        slwr    <=  1'b0;
    end
    else begin
        slwr    <=  1'b1;
    end
end

//cnt
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        cnt <= 0;
    end
    else if(add_cnt)begin
        if(end_cnt)
            cnt <= 0;
        else
            cnt <= cnt + 1;
    end
    else begin
        cnt <= 0;
    end
end

assign  add_cnt     =       slwr == 1'b0;       
assign  end_cnt     =       add_cnt && cnt == 4096-1;     

//data_gen
assign  data_gen    =       cnt;

endmodule
