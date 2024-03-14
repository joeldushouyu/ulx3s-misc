/************************************************************************
 * Author        : Wen Chunyang
 * Email         : 1494640955@qq.com
 * Create time   : 2018-12-21 10:57
 * Last modified : 2018-12-21 10:57
 * Filename      : usb.v
 * Description   : 
 * *********************************************************************/
module  usb_fifo_example(
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
        output  reg             sloe                            ,
        output  reg             slrd                            ,
        output  reg             slwr                            ,
        output  wire            pktend                          ,
        output  reg   [ 1: 0]   fifo_addr                       ,
        inout   wire  [31: 0]   usb_data                        ,
        output  reg             cmd_flag                        ,
        output  wire  [31: 0]   cmd_data 
)/*synthesis noprune*/;
//======================================================================\
//************** Define Parameter and Internal Signals *****************
//======================================================================/
parameter   IDLE            =   4'b0001                         ;
parameter   WRITE           =   4'b0010                         ;
parameter   READ            =   4'b0100                         ;
reg     [ 3: 0]                 state_c /*synthesis preserve*/  ;
reg     [ 3: 0]                 state_n /*synthesis preserve*/  ;
//cnt
reg     [31: 0]                 cnt                             ;
wire                            add_cnt                         ;
wire                            end_cnt                         ;

reg     [15: 0]                 rd_cnt                          ;
reg     [15: 0]                 rd_data_len                     ;
//======================================================================\
//**************************** Main Code *******************************
//======================================================================/
assign  slcs        =   1'b0;
assign  pclk        =   pclk_in;
assign  pktend      =   1'b1;
assign  usb_data    =   (slwr) ? 32'dz : cnt;

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
            else if(flag_c && flag_d)begin
                state_n = READ;
            end
            else begin
                state_n = state_c;
            end
        end
        WRITE:begin
            if(flag_b == 1'b0)begin //写满
                state_n = IDLE;
            end
            else begin
                state_n = state_c;
            end
        end
        READ:begin
            if(flag_d == 1'b0)begin //读空
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

//fifo_addr
always  @(posedge clk or negedge rst_n)begin
    if(rst_n == 1'b0)begin
        fifo_addr   <=  2'b00;
        sloe        <=  1'b1;
    end
    else if(state_n[2])begin
        fifo_addr   <=  2'b11;
        sloe        <=  1'b0;
    end
    else begin
        fifo_addr   <=  2'b00;
        sloe        <=  1'b1;
    end
end

//slwr 输出信号，用时序逻辑好点
always  @(posedge clk or negedge rst_n)begin
    if(rst_n == 1'b0)begin
        slwr    <=  1'b1;
    end
    else if(state_n[1])begin 
        slwr    <=  1'b0; // start writing out to fx3
    end
    else begin
        slwr    <=  1'b1;
    end
end

//slrd
always  @(posedge clk or negedge rst_n)begin
    if(rst_n == 1'b0)begin
        slrd    <=  1'b1;
    end
    else if(state_n[2])begin
        slrd    <=  1'b0; // start reading from fx3
    end
    else begin
        slrd    <=  1'b1;
    end
end

//rd_cnt
always  @(posedge clk or negedge rst_n)begin
    if(rst_n == 1'b0)begin
        rd_cnt  <=  16'd0;
    end
    else if(slrd)begin
        rd_cnt  <=  16'd0;
    end
    else begin
        rd_cnt  <=  rd_cnt + 1'b1;
    end
end

//rd_data_len
always  @(posedge clk or negedge rst_n)begin
    if(rst_n == 1'b0)begin
        rd_data_len <=  16'd0;
    end
    else if(rd_cnt == 16'd3)begin
        rd_data_len <=  usb_data + 16'd3; //加3是因为地址到数据有3拍延迟
    end
end

//cmd_flag
always  @(posedge clk or negedge rst_n)begin
    if(rst_n == 1'b0)begin
        cmd_flag    <=  1'b0;
    end
    else if(rd_cnt == 16'd3)begin
        cmd_flag    <=  1'b1;
    end
    else if(rd_cnt == rd_data_len)begin
        cmd_flag    <=  1'b0;
    end
end

assign  cmd_data    =   usb_data;

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

endmodule
