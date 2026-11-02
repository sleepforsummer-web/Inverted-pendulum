// 上位机绝对位置轨迹。10ms更新，目标保持Q16；限加速度与提前减速，停止距离不用DSP。
module serial_position #(
 parameter integer ACCEL_Q16=2048, // 每10ms速度增加1/32计数/节拍，1080计数/圈时约104.17度/秒²
 parameter integer MAX_STEP_Q8=2765 // 360度/秒×1080/360/100×256，四舍五入
)(
 input wire clk,rst_n,enable,sample_tick,allow_start,start_guard,
 input wire command_valid,
 input wire [7:0] command_seq,
 input wire signed [31:0] command_goal_q8,
 input wire [15:0] command_step_q8,
 input wire signed [32:0] anchor_q16,
 output reg signed [32:0] target_q16,
 output reg signed [31:0] goal_q8,
 output reg busy,target_valid,finished,
 output wire accepted,motion_active,
 output reg [7:0] reply_seq,reply_status
);
 // 状态：0尚无命令，1接受/执行，2未平衡，3其它运动忙，4范围错，5启动摆角/速度不满足，7轨迹完成，8被停机取消。
 wire goal_ok=command_goal_q8>=-32'sd16777216 && command_goal_q8<=32'sd16776960;
 wire step_ok=command_step_q8!=0 && command_step_q8<=MAX_STEP_Q8;
 assign accepted=command_valid && enable && allow_start && !busy && start_guard && goal_ok && step_ok;
 assign motion_active=enable && (busy || accepted);
 reg signed [32:0] goal_q16=0;
 reg [23:0] max_step_q16=0;
 reg signed [24:0] velocity_q16=0;
 wire signed [33:0] remaining=$signed({goal_q16[32],goal_q16})-$signed({target_q16[32],target_q16});
 wire signed [33:0] speed_cap=$signed({10'b0,max_step_q16});
 wire signed [33:0] near_velocity=remaining>>>5; // 距离缩小后逐步降速，避免末端突然刹停。
 wire signed [33:0] normal_velocity=near_velocity>speed_cap ? speed_cap :
     (near_velocity< -speed_cap ? -speed_cap : near_velocity);
 // 超过旧速度门限后，按当前速度估算停止距离，避免高速冲到终点才截断。
 // 速度折算为ceil(|v|/a)个加速量，停止距离取a*n*(n+1)/2的保守上界。
 // 小位宽乘法强制逻辑实现，两级主时钟流水；10ms采样前已稳定，不增加DSP。
 localparam integer BRAKE_STEPS_BITS=$clog2((MAX_STEP_Q8*256+ACCEL_Q16-1)/ACCEL_Q16+1);
 wire [24:0] velocity_magnitude=velocity_q16[24] ? -velocity_q16 : velocity_q16;
 wire [BRAKE_STEPS_BITS-1:0] brake_steps=(velocity_magnitude+ACCEL_Q16-1)/ACCEL_Q16;
 reg [BRAKE_STEPS_BITS-1:0] brake_steps_q=0;
 wire [BRAKE_STEPS_BITS:0] brake_steps_plus={1'b0,brake_steps_q}+1'b1;
 wire [2*BRAKE_STEPS_BITS:0] brake_product_logic /* synthesis syn_dspstyle="logic" */ =brake_steps_q*brake_steps_plus;
 reg [2*BRAKE_STEPS_BITS:0] brake_product=0;
 reg [33:0] brake_distance=0;
 wire [33:0] distance_magnitude=remaining[33] ? -remaining : remaining;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin brake_steps_q<=0;brake_product<=0;brake_distance<=0;end
  else if(!enable || !busy) begin brake_steps_q<=0;brake_product<=0;brake_distance<=0;end
  else begin brake_steps_q<=brake_steps;brake_product<=brake_product_logic;brake_distance<=(brake_product*ACCEL_Q16)>>1;end
 end
 // 同时预留下一拍可能继续加速并前进的距离，不能只比较当前速度的停止距离。
 wire [33:0] brake_guard_distance=brake_distance+({9'b0,velocity_magnitude}<<1)+3*ACCEL_Q16;
 wire brake_now_calc=max_step_q16>24'd262144 && velocity_magnitude!=0 && distance_magnitude<=brake_guard_distance;
 reg brake_now=0;
 wire signed [33:0] desired_velocity=brake_now ? 34'sd0 : normal_velocity;
 wire signed [33:0] old_velocity={{9{velocity_q16[24]}},velocity_q16};
 wire signed [33:0] next_velocity_calc=desired_velocity>old_velocity+ACCEL_Q16 ? old_velocity+ACCEL_Q16 :
     (desired_velocity<old_velocity-ACCEL_Q16 ? old_velocity-ACCEL_Q16 : desired_velocity);
 reg signed [33:0] next_velocity=0;
 reg [5:0] calculation_ready_pipe=0;
 wire calculation_ready=calculation_ready_pipe[5];
 // 每组位置更新后留出六个50MHz时钟，让停止距离、刹车判定、速度计算依次完成。
 // 生产采样间隔为500000个时钟；运动目标仍在原10ms节拍更新。
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin brake_now<=0;next_velocity<=0;calculation_ready_pipe<=0;end
  else if(!enable || !busy) begin brake_now<=0;next_velocity<=0;calculation_ready_pipe<=0;end
  else begin
   brake_now<=brake_now_calc;next_velocity<=next_velocity_calc;
   if(sample_tick && calculation_ready) calculation_ready_pipe<=0;
   else calculation_ready_pipe<={calculation_ready_pipe[4:0],1'b1};
  end
 end
 wire signed [33:0] next_position=$signed({target_q16[32],target_q16})+next_velocity;
 wire endpoint=(remaining<=34'sd256 && remaining>=-34'sd256 &&
     old_velocity<=ACCEL_Q16 && old_velocity>=-ACCEL_Q16) ||
     (remaining>0 && next_velocity>=remaining) || (remaining<0 && next_velocity<=remaining);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   target_q16<=0;goal_q8<=0;goal_q16<=0;max_step_q16<=0;velocity_q16<=0;
   busy<=0;target_valid<=0;finished<=0;reply_seq<=0;reply_status<=0;
  end else begin
   target_valid<=0;finished<=0;
   if(!enable) begin
    target_q16<=anchor_q16;busy<=0;velocity_q16<=0;
    if(busy && reply_status==1) reply_status<=8;
   end else if(busy && sample_tick && calculation_ready) begin
    target_valid<=1;
    if(endpoint) begin
     target_q16<=goal_q16;velocity_q16<=0;busy<=0;finished<=1;
     if(reply_status==1) reply_status<=7;
    end else begin target_q16<=next_position[32:0];velocity_q16<=next_velocity[24:0];end
   end
   if(command_valid) begin
    reply_seq<=command_seq;
    if(!enable) reply_status<=2;
    else if(!allow_start || busy) reply_status<=3;
    else if(!goal_ok || !step_ok) reply_status<=4;
    else if(!start_guard) reply_status<=5;
    else begin
     reply_status<=1;goal_q8<=command_goal_q8;goal_q16<={command_goal_q8[24:0],8'b0};
     target_q16<=anchor_q16;velocity_q16<=0;max_step_q16<={command_step_q8,8'b0};busy<=1;
    end
   end
  end
 end
endmodule
