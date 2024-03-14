/************************************************************************
 * Author        : Wen Chunyang
 * Email         : 1494640955@qq.com
 * Create time   : 2018-12-21 10:57
 * Last modified : 2018-12-21 10:57
 * Filename      : usb.v
 * Description   : 
 * *********************************************************************/
module  usb_stream_out_example(
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
        output  wire            slwr                            ,
        output  wire            pktend                          ,
        output  wire  [ 1: 0]   fifo_addr                       ,
        input   wire  [31: 0]   usb_data                        ,
        //additiona
)/*synthesis noprune*/;
//======================================================================\
//************** Define Parameter and Internal Signals *****************
//======================================================================/
parameter   IDLE            =   4'b0001                         ;
parameter   READ            =   4'b0010                         ;
reg     [ 3: 0]                 state_c /*synthesis preserve*/  ;
reg     [ 3: 0]                 state_n /*synthesis preserve*/  ;
//======================================================================\
//**************************** Main Code *******************************
//======================================================================/
assign  fifo_addr   =   2'b11;
assign  slcs        =   1'b0;
assign  pclk        =   pclk_in;
assign  pktend      =   1'b1;
assign  slwr        =   1'b1;
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
            if(flag_c && flag_d)begin
                state_n = READ;
            end
            else begin
                state_n = state_c;
            end
        end
        READ:begin
            if(flag_d == 1'b0)begin
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

assign  slrd    =   (state_c == READ) ? 1'b0 : 1'b1;
assign  sloe    =   slrd;

endmodule
