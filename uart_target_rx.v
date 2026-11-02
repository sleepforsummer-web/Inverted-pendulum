// 115200 8N1接收及目标指令解析。仅移位、计数、异或，不使用DSP。
// 指令11字节：A5 5A 01 序号 目标Q8小端4字节 速度Q8小端2字节 XOR。
// 目标单位：编码器计数；速度单位：每10ms目标增量。XOR覆盖01至速度末字节。
module uart_target_rx #(
 parameter integer CLOCK_HZ=50000000,
 parameter integer BAUD=115200
)(
 input wire clk,rst_n,rx,
 output reg command_valid,
 output reg [7:0] command_seq,
 output reg signed [31:0] command_goal_q8,
 output reg [15:0] command_step_q8
);
 localparam integer BIT_CLOCKS=(CLOCK_HZ+BAUD/2)/BAUD;
 localparam integer CW=$clog2(BIT_CLOCKS+1);
 localparam integer TIMEOUT_CLOCKS=CLOCK_HZ/20;
 localparam integer TW=$clog2(TIMEOUT_CLOCKS+1);
 (* syn_preserve=1 *) reg rx_meta=1,rx_sync=1;
 reg [1:0] state=0;
 reg [CW-1:0] counter=0;
 reg [2:0] bit_no=0;
 reg [7:0] shift=0,byte_value=0;
 reg byte_valid=0;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   rx_meta<=1;rx_sync<=1;state<=0;counter<=0;bit_no<=0;
   shift<=0;byte_value<=0;byte_valid<=0;
  end else begin
   rx_meta<=rx;rx_sync<=rx_meta;byte_valid<=0;
   case(state)
    0: if(!rx_sync) begin state<=1;counter<=BIT_CLOCKS/2-1;end
    1: if(counter!=0) counter<=counter-1'b1;
       else if(!rx_sync) begin state<=2;counter<=BIT_CLOCKS-1;bit_no<=0;end
       else state<=0; // 半位检查：忽略短毛刺。
    2: if(counter!=0) counter<=counter-1'b1;
       else begin
        shift[bit_no]<=rx_sync;counter<=BIT_CLOCKS-1;
        if(bit_no==7) state<=3;else bit_no<=bit_no+1'b1;
       end
    3: if(counter!=0) counter<=counter-1'b1;
       else begin
        state<=0;
        if(rx_sync) begin byte_value<=shift;byte_valid<=1;end
       end
   endcase
  end
 end
 reg [3:0] index=0;
 reg [7:0] check=0,opcode=0,seq_hold=0;
 reg signed [31:0] goal_hold=0;
 reg [15:0] step_hold=0;
 reg [TW-1:0] timeout_count=0;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   index<=0;check<=0;opcode<=0;seq_hold<=0;goal_hold<=0;step_hold<=0;
   timeout_count<=0;command_valid<=0;command_seq<=0;command_goal_q8<=0;command_step_q8<=0;
  end else begin
   command_valid<=0;
   if(index!=0 && !byte_valid) begin
    if(timeout_count==TIMEOUT_CLOCKS-1) begin index<=0;timeout_count<=0;end
    else timeout_count<=timeout_count+1'b1;
   end
   if(byte_valid) begin
    timeout_count<=0;
    if(index>=2 && index<=9) check<=check^byte_value;
    case(index)
     0: if(byte_value==8'hA5) index<=1;
     1: if(byte_value==8'h5A) begin index<=2;check<=0;end
        else if(byte_value!=8'hA5) index<=0;
     2: begin opcode<=byte_value;index<=3;end
     3: begin seq_hold<=byte_value;index<=4;end
     4: begin goal_hold[7:0]<=byte_value;index<=5;end
     5: begin goal_hold[15:8]<=byte_value;index<=6;end
     6: begin goal_hold[23:16]<=byte_value;index<=7;end
     7: begin goal_hold[31:24]<=byte_value;index<=8;end
     8: begin step_hold[7:0]<=byte_value;index<=9;end
     9: begin step_hold[15:8]<=byte_value;index<=10;end
     10: begin
      index<=0;
      if(byte_value==check && opcode==8'd1) begin
       command_goal_q8<=goal_hold;command_step_q8<=step_hold;
       command_seq<=seq_hold;command_valid<=1;
      end
     end
     default:index<=0;
    endcase
   end
  end
 end
endmodule
