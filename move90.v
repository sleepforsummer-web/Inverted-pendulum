// 单次匀速目标：每10ms累加一个Q16步长，最后一拍对齐终点，然后保持，不返回。
// 当前标定1080计数/圈；STEP_COUNTS=1080表示转360°，100个10ms节拍=1秒。
module move90 #(
 parameter signed [16:0] STEP_COUNTS=17'sd1080, // 改为负值可反向
 parameter integer MOVE_TICKS=130,           // 运动时间=本参数乘10ms，必须>=1
 parameter USE_DIRECTION=0,                  // 1: latch reverse with each request
 parameter USE_LOAD_Q16=0                    // 串口目标可含小数；旧实例仍按整数装载
)(
 input wire clk, rst_n, enable, sample_tick, start_event,
 input wire reverse,
 input wire load_target, // 恢复捕获时装载新的绝对位置目标，同时取消旧运动
 input wire signed [16:0] load_position,
 input wire signed [32:0] load_position_q16,
 output reg signed [32:0] target_q16,
 output reg busy, target_valid, range_rejected,
 output wire motion_active // 包括按键请求与等待首个位置节拍，保证第一拍就使用运动限幅
);
 reg pending_start;
 reg reverse_latched;
 wire reverse_now=USE_DIRECTION && ((pending_start || busy) ? reverse_latched : reverse);
 assign motion_active=enable && !load_target && (busy || pending_start || start_event);
 localparam integer COUNT_BITS=(MOVE_TICKS<2) ? 1 : $clog2(MOVE_TICKS+1);
 localparam signed [33:0] OFFSET_Q16=STEP_COUNTS*34'sd65536;
 localparam signed [33:0] INCREMENT_Q16=OFFSET_Q16/MOVE_TICKS;
 reg [COUNT_BITS-1:0] remaining;
 reg signed [32:0] goal_q16;
 wire signed [33:0] old_target=$signed({target_q16[32],target_q16});
 wire signed [33:0] next_goal=old_target+(reverse_now ? -OFFSET_Q16 : OFFSET_Q16);
 wire signed [33:0] incremented=old_target+(reverse_now ? -INCREMENT_Q16 : INCREMENT_Q16);
 localparam signed [33:0] TARGET_MAX=34'sd4294901760; // +65535计数
 localparam signed [33:0] TARGET_MIN=-34'sd4294967296; // -65536计数
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   target_q16<=0; busy<=0; target_valid<=0; range_rejected<=0;
   pending_start<=0; remaining<=0; goal_q16<=0; reverse_latched<=0;
  end else if(load_target) begin
   // 装载优先于enable，允许在PID第一组计算前建立新目标。
   target_q16<=USE_LOAD_Q16 ? load_position_q16 : $signed({load_position,16'b0}); target_valid<=1;
   busy<=0; range_rejected<=0; pending_start<=0; remaining<=0; goal_q16<=0; reverse_latched<=0;
  end else if(!enable) begin
   // 停机/起摆不保留旧运动；自动恢复切入PID时另行装载当前位置目标。
   target_q16<=0; busy<=0; target_valid<=0; range_rejected<=0;
   pending_start<=0; remaining<=0; goal_q16<=0; reverse_latched<=0;
  end else begin
   target_valid<=0;
   if(start_event && !busy && !pending_start) begin
    pending_start<=1; reverse_latched<=USE_DIRECTION && reverse;
   end
   if(sample_tick) begin
    if(!busy && (pending_start || start_event)) begin
     pending_start<=0; range_rejected<=0;
     if(next_goal<=TARGET_MAX && next_goal>=TARGET_MIN) begin
      goal_q16<=next_goal[32:0]; target_valid<=1;
      if(MOVE_TICKS==1) begin target_q16<=next_goal[32:0]; remaining<=0; busy<=0; end
      else begin target_q16<=incremented[32:0]; remaining<=MOVE_TICKS-1; busy<=1; end
     end else range_rejected<=1; // 接近计数边界时拒绝整次运动，不回卷目标。
    end else if(busy) begin
     target_valid<=1;
     if(remaining==1) begin
      // 末拍补齐常数除法截断误差，终点精确对齐STEP_COUNTS设定的位置。
      target_q16<=goal_q16; remaining<=0; busy<=0;
     end else begin target_q16<=incremented[32:0]; remaining<=remaining-1'b1; end
    end
   end
  end
 end
endmodule
