// 角度失稳后，仅等待旋臂速度连续低于门限，再请求自动起摆。
// 不读取摆杆角速度，不改变原PID/起摆算法；人工取消优先于失稳和重启。
module auto_recover #(
 parameter ENABLE=1,
 parameter signed [15:0] SPEED_MAX=16'sd1, // 编码器计数/10ms，包含正负边界
 parameter integer LOW_SAMPLES=20        // 连续20次有效速度采样，约200ms
)(
 input wire clk, rst_n,
 input wire sample_tick,                 // 编码器速度更新节拍
 input wire signed [15:0] speed,
 input wire running, fault_event, cancel,
 output reg waiting,
 output wire restart_request
);
 localparam integer COUNT_BITS=(LOW_SAMPLES<2) ? 1 : $clog2(LOW_SAMPLES+1);
 reg [COUNT_BITS-1:0] low_count;
 reg sample_d;
 wire speed_low=(speed>=-SPEED_MAX) && (speed<=SPEED_MAX);
 // 等顶层确认running后才撤销waiting，避免确认前出现人工取消漏窗。
 assign restart_request=ENABLE && rst_n && waiting && !running && !cancel &&
                        speed_low && (low_count>=LOW_SAMPLES);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin waiting<=0; low_count<=0; sample_d<=0; end
  else begin
   sample_d<=sample_tick;
   if(!ENABLE || cancel) begin waiting<=0; low_count<=0; end
   else if(fault_event) begin waiting<=1; low_count<=0; end
   else if(running) begin waiting<=0; low_count<=0; end
   else if(waiting && sample_d) begin
    // 延迟一个50MHz时钟，读取编码器本次更新后的速度，不能反复统计旧值。
    if(!speed_low) low_count<=0;
    else if(low_count<LOW_SAMPLES) low_count<=low_count+1'b1;
   end
  end
 end
endmodule
