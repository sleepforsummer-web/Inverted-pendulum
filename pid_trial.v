module pid_trial #(
 parameter integer KEY_DEBOUNCE_CYCLES=1000000, // 50MHz主时钟下，对应20ms按键消抖
 parameter integer KEY4_LONG_CYCLES=50000000, // KEY4长按1秒反向一圈
 parameter AUTO_RECOVER_ENABLE=1,              // 失稳自动恢复开关；0保留原来的越界停机不重启
 parameter signed [15:0] RECOVER_SPEED_MAX=16'sd6, // 旋臂速度门限，计数/10ms
 parameter integer RECOVER_LOW_SAMPLES=20      // 连续低速采样次数，20次约200ms
)(
 input wire clk_50m, rst_n, // rst_n为KEY5（AB3），按下立即停机并全局复位
 input wire [9:0] adc_data_in,
 input wire [3:0] key_sw, // KEY1自动/停机，KEY2手动启动，KEY3正弦启停，KEY4单次运动
 input wire uart_rx, // J280板载CH340N的TXD到FPGA L20，115200 8N1

 // 旋臂编码器
 input wire enc_a,
 input wire enc_b,

 output wire adc_clk, adc_oe_n,
 output wire uart_tx,
 output wire PWMA, AN1, AN2, PWMB, BN1, BN2
);
 // 统一的竖直向上标定值，同时用于ADC角度换算和内环目标角。
 localparam [11:0] UPRIGHT_ADC = 12'd1979;
 // MATLAB平滑D候选参数，Q16值=实际系数乘65536后取整。
 // 所有调参统一改这里；Kd必须与下方滤波配置配套使用。
 localparam signed [31:0] INNER_KP_Q = 32'sd406323;  // Kp约6.2
 localparam signed [31:0] INNER_KI_Q = 32'sd0;       // 关闭积分
 localparam signed [31:0] INNER_KD_Q = 32'sd9339312; // Kd约137.7764，乘每1ms实际角度差值
 localparam integer INNER_D_FILTER_SHIFT = 2;      // 新差值权重1/4，Q8滤波状态
 localparam signed [31:0] OUTER_KP_Q = 32'sd19661;  // Kp约0.3，保留用户当前值
 localparam signed [31:0] OUTER_KD_Q = 32'sd1564112; // Kd约23.8665，乘每10ms实际位置差值
 localparam integer OUTER_D_FILTER_SHIFT = 2;      // 新差值权重1/2
 // 外环积分复用pid_i：增量=Ki×位置误差，每10ms执行一次，贡献单位为目标角ADC。
 localparam signed [31:0] OUTER_KI_Q = 32'sd80; // 约0.001/采样；设0关闭外环积分
 localparam signed [48:0] OUTER_I_MAX_Q = 49'sd786432; // 12×65536，积分贡献限幅±12 ADC
 localparam signed [16:0] OUTER_I_SEP_COUNTS = 17'sd50; // 圆整误差绝对值<50才积分，否则清零
 // 外环限幅切换：位置误差按编码器计数，倾角限幅按ADC计数，时间按毫秒。
 localparam signed [16:0] OUTER_LIMIT_NEAR_ADC = 17'sd40;//外环目标角小限幅
 localparam signed [16:0] OUTER_LIMIT_FAR_ADC = 17'sd200;//保留用户当前常规大限幅200，不平滑
 localparam signed [16:0] OUTER_LIMIT_POSITION_COUNTS = 17'sd10;
 localparam integer OUTER_LIMIT_MIN_MS = 500;
 localparam integer OUTER_SAMPLE_MS = 10; // 与下方100Hz位置节拍一致，不单独改此值
 // 每次自动起摆/恢复接管PID时执行一次150，固定设定时长后结束，不检查位置。
 localparam OUTER_STARTUP_LIMIT_ENABLE = 1;
 localparam signed [16:0] OUTER_STARTUP_LIMIT_ADC = 17'sd150;
 localparam integer OUTER_STARTUP_LIMIT_MS = 500;
 // 已恢复原40/110双限幅；下面的200包络关闭，不参与实际控制或综合。
 localparam OUTER_DISTURB_LIMIT_ENABLE = 0;
 localparam signed [16:0] OUTER_DISTURB_LIMIT_ADC = 17'sd200;
 localparam integer OUTER_DISTURB_RAMP_MS = 1000; // 包络关闭时此值不生效
 localparam signed [16:0] MOTION_ARRIVE_COUNTS = 17'sd50; // 轨迹完成后，实际旋臂进入目标±50计数才退出运动小限幅
 // KEY3正弦轨迹：每10ms更新目标；再按KEY3立即切回原目标。
 localparam integer SINE_COUNTS_PER_REV = 1080;
 localparam integer SINE_AMPLITUDE_DEG = 90;
 localparam integer SINE_PERIOD_TICKS = 500;
 localparam integer SINE_START_THETA_ADC = 100;
 localparam integer SINE_START_SPEED = 3;
 // 起摆参数：角度采用ADC计数，时间采用毫秒，幅值采用电机指令单位。
 localparam [11:0] CAPTURE_RANGE = 12'd250;//切换PID
 localparam integer SWING_SAMPLE_MS = 20;
 localparam integer SWING_PULSE_MS = 100;
 localparam signed [16:0] SWING_DUTY = 17'sd550; // 最高点驱动幅值；500指令约对应40%占空比
 localparam LEFT_FIRST_POSITIVE = 0; // 保留用户方向设置：左侧先负后正，右侧相反
 // 自动模式在底部附近启动时，先执行一次初始正反双脉冲。
 localparam signed [16:0] START_KICK_DUTY = 17'sd700; // 初始驱动幅值，约56%占空比；设0关闭
 localparam integer START_KICK_MS = 200; // 每段200ms，合计400ms；中心捕获或停机可提前结束
 localparam START_KICK_POSITIVE = 1;
 localparam [11:0] START_BOTTOM_LOW = 12'd100;
 localparam [11:0] START_BOTTOM_HIGH = 12'd3990;
 wire [2:0] swing_state;
 wire balance_mode, swing_pulse_active, swing_peak;
 wire [1:0] swing_peak_region;
 wire signed [16:0] swing_duty;
 wire adc_valid;
 // 直接用状态判断是否由PID接管，避免与受角度保护影响的使能信号形成组合依赖。
 reg manual_mode=0;
 wire balance_active = manual_mode || (swing_state == 3'd4);
 wire signed [12:0] theta;
 // KEY1启动自动起摆，运行中再按KEY1停机；KEY2仅在停机时手扶启动PID。
 // KEY3启动/立即结束正弦，不停止平衡；KEY5全局停机复位。
 // 角度失稳停机后，自动恢复模块仅在旋臂连续低速时请求重新起摆。
 // 等待期间KEY1/KEY2取消恢复；KEY3不排队。人工停机不会触发自动恢复。
 // 再次接受KEY1按下之前，必须先完成松开消抖。
 reg [2:0] key_meta=3'b111, key_sync=3'b111;
 reg [19:0] press_count=0, release_count=0;
 reg press_ready=0;
 reg [19:0] key2_press_count=0, key2_release_count=0;
 reg key2_press_ready=0;
 reg running=0;
 wire key2_released=key_sync[1]; // 仅供原串口状态字段观察，不再用于禁止或停止运行
 // 停机原因：1为KEY1、3为角度越界；KEY5复位会清除诊断。旧2/4编码保留兼容。
 reg [7:0] stop_reason=0;
 reg signed [12:0] stop_theta=0;
 wire [11:0] raw, filtered_raw;
 wire [11:0] adc_block_min, adc_block_max;
 wire [17:0] adc_block_sum;
 wire [7:0] adc_block_count;
 reg [11:0] stop_min=0, stop_max=0;
 reg [17:0] stop_sum=0;
 reg [7:0] stop_count=0;
 reg [11:0] stop_raw=0, stop_filtered_raw=0;
 reg [2:0] stop_keys=3'b111; // 停机时的同步按键电平：第0/1/2位对应KEY1/2/3，低电平为按下
 localparam signed [12:0] STOP_THRESHOLD=13'sd1000; // PID模式停机阈值，单位为ADC计数，不是度数
 wire in_range=(theta > -STOP_THRESHOLD && theta < STOP_THRESHOLD);
 wire press_event=press_ready && !key_sync[0] && press_count==KEY_DEBOUNCE_CYCLES-1;
 wire key2_press_event=key2_press_ready && !key_sync[1] &&
                       key2_press_count==KEY_DEBOUNCE_CYCLES-1;
 // 停机时KEY1/KEY2同时按下不启动；运行中只有KEY1仍作主动停机。
 wire auto_start=press_event && !key2_press_event;
 wire manual_start=!running && key2_press_event && !press_event && in_range;
 wire angle_stop=balance_active && !in_range;
 wire enable=rst_n && running && !angle_stop;
 wire pid_enable=enable && balance_active;
 wire recovery_waiting, recovery_restart;
 // 失稳时锁存目标，自动恢复捕获后计算原目标的最近等效圈。
 reg recovery_anchor_pending=0;
 reg signed [32:0] recovery_saved_target=0;
 wire signed [32:0] recovery_nearest_target;
 wire recovery_target_ready;
 wire recovery_load_target=recovery_anchor_pending && pid_enable && recovery_target_ready;
 wire control_enable=pid_enable && !recovery_anchor_pending;
 reg control_enable_d=0;
 always @(posedge clk_50m or negedge rst_n)
  if(!rst_n) control_enable_d<=0;
  else control_enable_d<=control_enable;
 wire startup_once_arm=control_enable && !control_enable_d && !manual_mode;
 always @(posedge clk_50m or negedge rst_n) begin
  if(!rst_n) recovery_anchor_pending<=0;
  else if(recovery_restart && !running) recovery_anchor_pending<=1;
  else if(!running || recovery_load_target) recovery_anchor_pending<=0;
 end
 always @(posedge clk_50m or negedge rst_n) begin
  if(!rst_n) begin
   key_meta<=7; key_sync<=7; press_count<=0; release_count<=0;
   press_ready<=0; running<=0; manual_mode<=0;
   key2_press_count<=0; key2_release_count<=0; key2_press_ready<=0;
   stop_reason<=0; stop_theta<=0; stop_keys<=3'b111;
   stop_raw<=0; stop_filtered_raw<=0;
   stop_min<=0; stop_max<=0; stop_sum<=0; stop_count<=0;
  end
  else 
    begin
   key_meta<=key_sw[2:0]; key_sync<=key_meta;
   if(key_sync[0]) 
   begin
    press_count<=0;
    if(release_count<KEY_DEBOUNCE_CYCLES) release_count<=release_count+1'b1;
    if(release_count==KEY_DEBOUNCE_CYCLES-1) press_ready<=1;
   end
    else
       begin
    release_count<=0;
    if(press_count<KEY_DEBOUNCE_CYCLES) press_count<=press_count+1'b1;
   if(press_event) press_ready<=0;
   end
   if(key_sync[1]) begin
    key2_press_count<=0;
    if(key2_release_count<KEY_DEBOUNCE_CYCLES)
     key2_release_count<=key2_release_count+1'b1;
    if(key2_release_count==KEY_DEBOUNCE_CYCLES-1) key2_press_ready<=1;
   end else begin
    key2_release_count<=0;
    if(key2_press_count<KEY_DEBOUNCE_CYCLES)
     key2_press_count<=key2_press_count+1'b1;
    if(key2_press_event) key2_press_ready<=0;
   end
   // 锁存触发停机当时的测量值，避免串口稍后发送的快照掩盖瞬时异常。
   // 角度越界和按键停机同时发生时，优先记录角度越界原因。
   if(running && (angle_stop || press_event)) begin
    running<=0; manual_mode<=0;
    stop_reason<=angle_stop ? 8'd3 : 8'd1;
    stop_theta<=theta; stop_keys<=key_sync;
    // 以下ADC寄存输出与触发停机的theta属于同一组采样。
    stop_raw<=raw; stop_filtered_raw<=filtered_raw;
    stop_min<=adc_block_min; stop_max<=adc_block_max;
    stop_sum<=adc_block_sum; stop_count<=adc_block_count;
   end
   else if(recovery_waiting && (press_event || key2_press_event)) begin
    running<=0; manual_mode<=0;
   end
   else if(auto_start || manual_start || recovery_restart) begin
    // 自动恢复固定进入起摆模式，不沿用手动PID模式或原运动任务。
    running<=1; manual_mode<=recovery_restart ? 1'b0 : manual_start;
    stop_reason<=0; stop_theta<=0; stop_keys<=3'b111;
    stop_raw<=0; stop_filtered_raw<=0;
   stop_min<=0; stop_max<=0; stop_sum<=0; stop_count<=0;
   end
  end
 end
 
 reg [15:0] tick_count=0;
 wire tick=(tick_count==16'd49999);
 always @(posedge clk_50m or negedge rst_n)
  if(!rst_n) tick_count<=0;
  else if(tick) tick_count<=0;
  else tick_count<=tick_count+1'b1;

 swing_up #(
  .CENTER_ADC(UPRIGHT_ADC), .CAPTURE_RANGE(CAPTURE_RANGE),
  .SAMPLE_PERIOD_MS(SWING_SAMPLE_MS), .PULSE_MS(SWING_PULSE_MS),
  .PULSE_DUTY(SWING_DUTY), .LEFT_FIRST_POSITIVE(LEFT_FIRST_POSITIVE),
  .START_KICK_DUTY(START_KICK_DUTY), .START_KICK_MS(START_KICK_MS),
  .START_KICK_POSITIVE(START_KICK_POSITIVE),
  .START_BOTTOM_LOW(START_BOTTOM_LOW), .START_BOTTOM_HIGH(START_BOTTOM_HIGH)
 ) swing_inst(
  .clk(clk_50m), .rst_n(rst_n), .enable(enable && !manual_mode), .tick_1ms(tick),
  .adc_valid(adc_valid), .adc_value(filtered_raw),
  .balance_mode(balance_mode), .pulse_active(swing_pulse_active),
  .duty_command(swing_duty), .state(swing_state),
  .peak_event(swing_peak), .peak_region(swing_peak_region));
// ============================================================
// 外环 100 Hz 采样脉冲
// 50 MHz / 500000 = 100 Hz
// ============================================================
reg [18:0] pos_tick_count = 0;

wire pos_tick = (pos_tick_count == 19'd499999);

always @(posedge clk_50m or negedge rst_n) 
begin
    if(!rst_n)
        pos_tick_count <= 19'd0;
    else if(pos_tick)
        pos_tick_count <= 19'd0;
    else
        pos_tick_count <= pos_tick_count + 1'b1;
end
 
wire signed [16:0] position;
wire signed [15:0] speed;
reg signed [32:0] position_target_q16=0;

always @(posedge clk_50m or negedge rst_n)
 if(!rst_n) recovery_saved_target<=0;
 else if(running && angle_stop) recovery_saved_target<=position_target_q16;
shortest_recovery #(.COUNTS_PER_REV(SINE_COUNTS_PER_REV)) recovery_target_calc(
 .clk(clk_50m),.rst_n(rst_n),.enable(recovery_anchor_pending && pid_enable),
 .saved_target_q16(recovery_saved_target),.position(position),
 .result_q16(recovery_nearest_target),.ready(recovery_target_ready));

auto_recover #(.ENABLE(AUTO_RECOVER_ENABLE),.SPEED_MAX(RECOVER_SPEED_MAX),
              .LOW_SAMPLES(RECOVER_LOW_SAMPLES)) recovery(
 .clk(clk_50m),.rst_n(rst_n),.sample_tick(pos_tick),.speed(speed),.running(running),
 .fault_event(running && angle_stop),
 .cancel(press_event || key2_press_event),
 .waiting(recovery_waiting),.restart_request(recovery_restart));

encoder_decoder encoder_inst (
    .clk      (clk_50m),
    .rst_n    (rst_n),
    .en       (pos_tick),

    .a        (enc_a),
    .b        (enc_b),

    .position (position),
    .speed    (speed)
);

wire signed [16:0] position_error;
wire signed [16:0] theta_ref;
wire theta_ref_valid;
wire signed [31:0] outer_p_mon, outer_d_mon, outer_i_mon;
wire outer_limited;
wire signed [16:0] outer_active_limit;
wire [2:0] outer_limit_phase;
wire outer_boost_active;

// KEY4只改变旋臂位置目标，不能启动电机或绕过起摆/角度保护。
wire move_key_short, move_key_long;
wire move_key_event=move_key_short || move_key_long;
wire sine_key_event;
key_press #(.DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES)) sine_key(
 .clk(clk_50m),.rst_n(rst_n),.key_n(key_sw[2]),.pressed(sine_key_event));
wire motion_busy, motion_target_valid, motion_range_rejected, motion_limit_active;
wire motion_profile_active;
reg motion_arrival_wait=0;
wire signed [32:0] single_target_q16, sine_target_q16;
wire single_busy, single_valid, single_range_rejected, single_profile_active;
wire sine_busy, sine_valid, sine_range_rejected, sine_profile_active, sine_arrived;
wire sine_request_rejected, sine_stop_pending;
wire [2:0] sine_state;
reg single_activity_d=0;
// 上位机绝对目标命令不启动电机，只有已进入PID且其它运动空闲时接受。
wire remote_command_valid;
wire [7:0] remote_command_seq;
wire signed [31:0] remote_command_goal_q8;
wire [15:0] remote_command_step_q8;
wire signed [32:0] remote_target_q16;
wire signed [31:0] remote_goal_q8;
wire remote_busy,remote_valid,remote_finished,remote_accepted,remote_profile_active;
wire [7:0] remote_reply_seq,remote_reply_status;
reg remote_activity_d=0;
key_short_long #(.DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES),.LONG_CYCLES(KEY4_LONG_CYCLES)) move_key(
 .clk(clk_50m),.rst_n(rst_n),.key_n(key_sw[3]),
 .cancel(!control_enable || !key_sw[2] || sine_busy || remote_busy || remote_activity_d),
 .short_pressed(move_key_short),.long_pressed(move_key_long));

always @(posedge clk_50m or negedge rst_n)
 if(!rst_n) remote_activity_d<=0;
 else if(!control_enable) remote_activity_d<=0;
 else remote_activity_d<=remote_busy;
uart_target_rx command_receiver(
 .clk(clk_50m),.rst_n(rst_n),.rx(uart_rx),.command_valid(remote_command_valid),
 .command_seq(remote_command_seq),.command_goal_q8(remote_command_goal_q8),
 .command_step_q8(remote_command_step_q8));
wire remote_start_guard=(theta>=-SINE_START_THETA_ADC && theta<=SINE_START_THETA_ADC &&
                         speed>=-SINE_START_SPEED && speed<=SINE_START_SPEED);
serial_position remote_motion(
 .clk(clk_50m),.rst_n(rst_n),.enable(control_enable),.sample_tick(pos_tick),
 .allow_start(!single_busy && !single_activity_d && !sine_busy && !motion_arrival_wait &&
              !move_key_event && !sine_key_event),.start_guard(remote_start_guard),
 .command_valid(remote_command_valid),.command_seq(remote_command_seq),
 .command_goal_q8(remote_command_goal_q8),.command_step_q8(remote_command_step_q8),
 .anchor_q16(position_target_q16),.target_q16(remote_target_q16),.goal_q8(remote_goal_q8),
 .busy(remote_busy),.target_valid(remote_valid),.finished(remote_finished),
 .accepted(remote_accepted),.motion_active(remote_profile_active),
 .reply_seq(remote_reply_seq),.reply_status(remote_reply_status));
// 用已寄存的单次运动状态作互斥，避免start/load/motion_active之间形成组合环。
always @(posedge clk_50m or negedge rst_n)
 if(!rst_n) single_activity_d<=0;
 else if(!control_enable) single_activity_d<=0;
 else single_activity_d<=single_profile_active;
// KEY3在正弦运行时可立即切回原目标；新启动必须等待先前运动实际到位。
wire sine_event=control_enable && sine_key_event &&
 (sine_busy || (!single_busy && !single_activity_d && !remote_busy && !remote_activity_d && !motion_arrival_wait && !move_key_event && key_sw[3]));
wire sine_new_request=sine_event && !sine_busy;
wire single_start=control_enable && move_key_event && !sine_busy && !sine_key_event && !remote_busy && !remote_activity_d;
wire sine_data_valid=sine_valid && (sine_busy || sine_arrived);
wire sine_finished=sine_valid && sine_arrived;
wire single_load=recovery_load_target || sine_new_request || sine_finished || remote_accepted || remote_finished;
wire signed [32:0] single_load_q16=recovery_load_target ? recovery_nearest_target :
 (remote_finished ? remote_target_q16 : (sine_finished ? sine_target_q16 : position_target_q16));
wire signed [16:0] single_load_position=recovery_load_target ? recovery_nearest_target[32:16] :
 (sine_finished ? sine_target_q16[32:16] : position_target_q16[32:16]);
move90 #(.USE_LOAD_Q16(1),.USE_DIRECTION(1)) motion(
 .clk(clk_50m),.rst_n(rst_n),.enable(control_enable),.sample_tick(pos_tick),
 .load_target(single_load),.load_position(single_load_position),
 .load_position_q16(single_load_q16),
 .start_event(single_start),.reverse(move_key_long),.target_q16(single_target_q16),
 .busy(single_busy),.target_valid(single_valid),.range_rejected(single_range_rejected),
 .motion_active(single_profile_active));
sine_motion #(.COUNTS_PER_REV(SINE_COUNTS_PER_REV),.AMPLITUDE_DEG(SINE_AMPLITUDE_DEG),
 .PERIOD_TICKS(SINE_PERIOD_TICKS),.IMMEDIATE_STOP(1),.START_THETA_ADC(SINE_START_THETA_ADC),
 .START_SPEED(SINE_START_SPEED),.USE_START_ANCHOR(1)) sine_path(
 .clk(clk_50m),.rst_n(rst_n),.enable(control_enable),.sample_tick(pos_tick),
 .start_event(sine_event),.start_anchor_q16(position_target_q16),
 .load_target(recovery_load_target || single_start),
 .load_position(recovery_load_target ? recovery_nearest_target[32:16] : position_target_q16[32:16]),
 .actual_speed(speed),.theta(theta),.target_q16(sine_target_q16),
 .busy(sine_busy),.target_valid(sine_valid),.range_rejected(sine_range_rejected),
 .motion_active(sine_profile_active),.arrived(sine_arrived),
 .request_rejected(sine_request_rejected),.stop_pending_mon(sine_stop_pending),.state_mon(sine_state));
assign motion_busy=single_busy || sine_busy || remote_busy;
assign motion_target_valid=single_valid || sine_data_valid || remote_valid;
assign motion_range_rejected=single_range_rejected || sine_range_rejected;
assign motion_profile_active=single_profile_active || sine_profile_active || remote_profile_active;
always @(posedge clk_50m or negedge rst_n) begin
 if(!rst_n) position_target_q16<=0;
 else if(recovery_load_target) position_target_q16<=recovery_nearest_target;
 else if(!control_enable) position_target_q16<=0;
 else if(remote_valid) position_target_q16<=remote_target_q16;
 else if(sine_data_valid) position_target_q16<=sine_target_q16;
 else if(single_valid && !sine_busy && !remote_busy) position_target_q16<=single_target_q16;
end
// 保留原极性：误差=实际旋臂位置-目标位置；Q16小数直接参与外环P运算。
wire signed [33:0] position_error_full_q16 =
 $signed({position[16],position,16'b0}) - $signed({position_target_q16[32],position_target_q16});
wire signed [32:0] position_error_q16 =
 position_error_full_q16>34'sd4294901760 ? 33'sd4294901760 :
 position_error_full_q16< -34'sd4294967296 ? -33'sd4294967296 : position_error_full_q16[32:0];
assign position_error=position_error_q16>>>16; // 原串口位置误差字段跟随新运动目标

// 轨迹走完仍可能实际未到位：锁住小限幅，直到一次位置采样确认实际到达。
// 仅在本次运动中等待首次到位；到位后恢复原规则，不因后续扰动反复进入等待。
localparam signed [33:0] MOTION_ARRIVE_Q16 = MOTION_ARRIVE_COUNTS*34'sd65536;
wire motion_position_arrived = (position_error_full_q16 >= -MOTION_ARRIVE_Q16) &&
                               (position_error_full_q16 <= MOTION_ARRIVE_Q16);
wire motion_arrival_sample = pos_tick && !motion_profile_active && motion_position_arrived;
always @(posedge clk_50m or negedge rst_n) begin
 if(!rst_n) motion_arrival_wait<=0;
 else if(!control_enable || recovery_load_target) motion_arrival_wait<=0;
 else if(motion_profile_active) motion_arrival_wait<=1;
 else if(motion_range_rejected || motion_arrival_sample) motion_arrival_wait<=0;
end
// 到位确认当拍即可恢复正常外环采样；不等待下一个采样，也不带回旧的大限幅计时。
assign motion_limit_active = motion_profile_active || (motion_arrival_wait && !motion_arrival_sample);
// 只对KEY4单次运动加中心偏移，持续到实际到位退出运动模式。
reg key4_center_active=0, key4_center_reverse=0;
always @(posedge clk_50m or negedge rst_n) begin
 if(!rst_n) begin key4_center_active<=0; key4_center_reverse<=0; end
 else if(!control_enable || recovery_load_target || sine_new_request || remote_accepted) begin
  key4_center_active<=0; key4_center_reverse<=0;
 end else if(single_start && !single_busy && !single_activity_d) begin
  key4_center_active<=1; key4_center_reverse<=move_key_long;
 end else if((single_range_rejected && !single_profile_active) || motion_arrival_sample) begin
  key4_center_active<=0; key4_center_reverse<=0;
 end
end
wire signed [16:0] key4_center_offset =
 (control_enable && key4_center_active && motion_limit_active) ?
 (key4_center_reverse ? 17'sd12 : -17'sd12) : 17'sd0;



position_controller #(
    .Kp_pos(OUTER_KP_Q), .Kd_pos(OUTER_KD_Q),
    .Ki_pos(OUTER_KI_Q), .I_POS_MAX_Q(OUTER_I_MAX_Q), .I_POS_SEP_THRESHOLD(OUTER_I_SEP_COUNTS),
    .D_FILTER_SHIFT(OUTER_D_FILTER_SHIFT), .USE_Q16_POSITION(1), .D_ON_MEASUREMENT(1),
    .THETA_REF_MAX(OUTER_LIMIT_NEAR_ADC), .LIMIT_SWITCH_ENABLE(1),
    .THETA_REF_BOOST_MAX(OUTER_LIMIT_FAR_ADC),
    .LIMIT_POSITION_THRESHOLD(OUTER_LIMIT_POSITION_COUNTS),
    .LIMIT_BOOST_MIN_MS(OUTER_LIMIT_MIN_MS), .POSITION_SAMPLE_MS(OUTER_SAMPLE_MS),
    .STARTUP_BOOST_ENABLE(OUTER_STARTUP_LIMIT_ENABLE), .STARTUP_BOOST_MAX(OUTER_STARTUP_LIMIT_ADC),
    .STARTUP_BOOST_MS(OUTER_STARTUP_LIMIT_MS),
    .DISTURB_LIMIT_ENABLE(OUTER_DISTURB_LIMIT_ENABLE),
    .DISTURB_LIMIT_MAX(OUTER_DISTURB_LIMIT_ADC), .DISTURB_LIMIT_RAMP_MS(OUTER_DISTURB_RAMP_MS)
) position_control (
    .clk            (clk_50m),
    .rst            (~rst_n),
    .enable         (control_enable),
    .sample_tick    (pos_tick),

    .position_error (position_error),
    .position_error_q16(position_error_q16), .position_measurement(position),
    .integral_clear(motion_profile_active), // 保留原清积分阶段；轨迹结束后允许I帮助消除到位静差
    .motion_active(motion_limit_active), // 轨迹完成且实际到位后才恢复双条件限幅
    .startup_boost_arm(startup_once_arm),

    .theta_ref      (theta_ref),
    .theta_ref_valid(theta_ref_valid),
    .p_pos_mon(outer_p_mon),
    .d_pos_mon(outer_d_mon),
    .i_pos_mon(outer_i_mon),
    .limited_mon(outer_limited), .active_limit_mon(outer_active_limit),
    .boost_active_mon(outer_boost_active), .limit_phase_mon(outer_limit_phase)
);

wire signed [16:0] error;
wire signed [16:0] outer_adc_offset;
wire signed [16:0] target_adc;
wire signed [16:0] actual_adc;

// INVERT=1时，外环正向目标角偏移对应更小的绝对ADC目标值。
// 外环提供中心角附近的偏移；内环接收合成后的目标ADC和实际ADC。
assign outer_adc_offset = -theta_ref;
assign target_adc = $signed({5'b0, UPRIGHT_ADC}) + key4_center_offset + outer_adc_offset;
assign actual_adc = $signed({5'b0, filtered_raw});

 wire signed [50:0] pid_sum;
 wire pid_valid;
 wire signed [15:0] p_mon;
 wire signed [15:0] i_mon;
 wire signed [15:0] d_mon;
 wire signed [16:0] duty;
 // Q16输出限幅由下方OUT_MAX_Q设置：实际电机指令上限为该值除以65536。
 // P/I输入误差限幅为±100 ADC；D使用实际测量值，串口保留未限幅误差。
 // 积分分离按实际误差判断：绝对值小于40才允许积分，否则清除积分。



 pid_adc_controller #(
  .ADC_INVERT(1),
  .KP_Q(INNER_KP_Q), .KD_Q(INNER_KD_Q),
  .D_FILTER_SHIFT(INNER_D_FILTER_SHIFT),
  .OUT_MAX_Q(51'sd65536000),
  .KI_Q(INNER_KI_Q), // 当前关闭积分，内环采样周期为1ms
  .I_SEP_THRESHOLD(17'sd40), // 实际内环误差小于40才积分
  .ERROR_INPUT_LIMIT(17'sd150) // P/I输入误差限幅；设0关闭
) control(
  .clk(clk_50m), .rst(~rst_n), .enable(control_enable), .sample_tick(tick),
  .target_adc(target_adc), .actual_adc(actual_adc), .error_out(error),
  .pid_sum(pid_sum), .pid_valid(pid_valid),
  .p_mon(p_mon), .i_mon(i_mon), .d_mon(d_mon));
 // 起摆与PID共用一个PWM驱动，根据当前状态二选一，避免同时驱动电机。
 // 切入PID时，在第一组PID结果就绪前先清除旧的起摆驱动指令。
 reg balance_mode_d=0;
 always @(posedge clk_50m or negedge rst_n)
  if(!rst_n) balance_mode_d<=0;
  else balance_mode_d<=balance_active;
 wire signed [50:0] swing_sum=$signed({{34{swing_duty[16]}},swing_duty}) <<< 16;
 wire signed [50:0] motor_sum=balance_active ? pid_sum : swing_sum;
 wire motor_valid=balance_active ? (pid_valid || !balance_mode_d) : 1'b1;
 // PID模式保留角度停机保护；起摆需要经过大角度，因此起摆阶段绕过该保护。
 wire signed [12:0] motor_theta=balance_active ? theta : 13'sd0;
 motor_pwm #(.MOTOR_SIGN(1), .FALL_THRESH(STOP_THRESHOLD)) motor(
  .clk(clk_50m), .rst_n(rst_n), .enable(enable), .theta(motor_theta),
  .pid_sum(motor_sum), .pid_valid(motor_valid), .pwm_out(PWMA),
  .in1(AN1), .in2(AN2), .duty_out(duty));
 wire [15:0] monitor_duty=duty[15:0];
 assign PWMB=1'b0; assign BN1=1'b0; assign BN2=1'b0;
 // 传感器已重新安装：竖直向上ADC为1958，右侧区域小于1658，左侧区域大于2258。



 adc_v2 #(.OFFSET(UPRIGHT_ADC), .INVERT(1)) adc_inst(.clk_50m(clk_50m), .rst_n(rst_n),
  .adc_data_in(adc_data_in), .adc_clk(adc_clk), .adc_oe_n(adc_oe_n),
  .adc_stby_n(), .theta_raw(raw), .theta_filtered(filtered_raw), .theta(theta), .data_valid(adc_valid),
  .block_min(adc_block_min), .block_max(adc_block_max),
  .block_sum(adc_block_sum), .block_count(adc_block_count));
 reg [11:0] frame_raw=0;
 reg [15:0] frame_theta=0, frame_duty=0;
 reg [15:0] frame_err=0, frame_pos=0;
 reg [15:0] frame_p=0, frame_i=0, frame_d=0;
 reg [31:0] frame_err32=0, frame_pos32=0, frame_ref32=0, frame_pos_error32=0;
 reg [15:0] frame_speed=0;
 reg [31:0] frame_outer_p=0, frame_outer_d=0, frame_outer_i=0;
 reg [7:0] frame_flags=0;
 reg [7:0] frame_stop_reason=0, frame_stop_keys=8'd7;
 reg [15:0] frame_stop_theta=0;
 reg [15:0] frame_stop_raw=0, frame_stop_filtered_raw=0;
 reg [15:0] frame_stop_min=0, frame_stop_max=0;
 reg [31:0] frame_stop_sum=0;
 reg [7:0] frame_stop_count=0;
 reg [7:0] frame_limit_status=0; // bit4:2限幅阶段；bit1:0保留为0
 reg [15:0] frame_active_limit=0;
 reg signed [31:0] frame_remote_goal=0,frame_position_target=0;
 reg [7:0] frame_command_seq=0,frame_command_status=0;
 reg [8:0] baud_count=0;
 reg [3:0] bit_index=0;
 reg [6:0] byte_index=0;
 // 每帧开始时统一锁存数据，保证本帧各字段在发送期间保持不变。
 always @(posedge clk_50m or negedge rst_n)
  if(!rst_n) begin
   frame_theta<=0; frame_duty<=0; frame_err<=0; frame_pos<=0;
   frame_p<=0; frame_i<=0; frame_d<=0;
   frame_err32<=0; frame_pos32<=0; frame_ref32<=0; frame_pos_error32<=0;
   frame_speed<=0; frame_outer_p<=0; frame_outer_d<=0; frame_outer_i<=0; frame_flags<=0;
   frame_stop_reason<=0; frame_stop_theta<=0; frame_stop_keys<=7;
   frame_stop_raw<=0; frame_stop_filtered_raw<=0;
   frame_stop_min<=0; frame_stop_max<=0; frame_stop_sum<=0; frame_stop_count<=0;
   frame_limit_status<=0;frame_active_limit<=0;
   frame_remote_goal<=0;frame_position_target<=0;frame_command_seq<=0;frame_command_status<=0;
  end
  else if(byte_index==0 && bit_index==0 && baud_count==0) begin
   frame_duty<=monitor_duty; frame_theta<={{3{theta[12]}},theta};
   frame_err<=error[15:0]; frame_pos<=position[15:0];
   frame_p<=p_mon; frame_i<=i_mon; frame_d<=d_mon;
   frame_err32<={{15{error[16]}},error};
   frame_pos32<={{15{position[16]}},position};
   frame_ref32<={{15{theta_ref[16]}},theta_ref};
   frame_pos_error32<={{15{position_error[16]}},position_error};
   frame_speed<=speed;
   frame_outer_p<=outer_p_mon; frame_outer_d<=outer_d_mon; frame_outer_i<=outer_i_mon;
   // 第6位仍表示自动起摆；第7位在起摆时表示脉冲，在PID时表示大限幅有效。
   // AA5D在AA5C数据上追加实时限幅，并在状态字节中加入限幅阶段。
   frame_flags<={(balance_active ? (pid_enable && outer_boost_active) : swing_pulse_active),
                 (enable && !balance_active),key2_released,key_sync[0],in_range,running,outer_limited,enable};
   frame_stop_reason<=stop_reason;
   frame_stop_theta<={{3{stop_theta[12]}},stop_theta};
   frame_stop_keys<={5'b0,stop_keys};
   frame_stop_raw<={4'b0,stop_raw};
   frame_stop_filtered_raw<={4'b0,stop_filtered_raw};
   frame_stop_min<={4'b0,stop_min}; frame_stop_max<={4'b0,stop_max};
   frame_stop_sum<={14'b0,stop_sum}; frame_stop_count<=stop_count;
   frame_limit_status<={3'b0,(control_enable ? outer_limit_phase : 3'd0),2'b0};
   frame_active_limit<=control_enable ? outer_active_limit[15:0] : 16'd0;
   frame_remote_goal<=remote_goal_q8;frame_position_target<=position_target_q16>>>8;
   frame_command_seq<=remote_reply_seq;frame_command_status<=remote_reply_status;
  end
 wire [7:0] checksum=frame_theta[7:0] ^ frame_theta[15:8] ^
                      frame_err32[7:0] ^ frame_err32[15:8] ^ frame_err32[23:16] ^ frame_err32[31:24] ^
                      frame_pos32[7:0] ^ frame_pos32[15:8] ^ frame_pos32[23:16] ^ frame_pos32[31:24] ^
                      frame_duty[7:0] ^ frame_duty[15:8] ^
                      frame_p[7:0] ^ frame_p[15:8] ^ frame_i[7:0] ^ frame_i[15:8] ^ frame_d[7:0] ^ frame_d[15:8] ^
                      frame_ref32[7:0] ^ frame_ref32[15:8] ^ frame_ref32[23:16] ^ frame_ref32[31:24] ^
                      frame_pos_error32[7:0] ^ frame_pos_error32[15:8] ^ frame_pos_error32[23:16] ^ frame_pos_error32[31:24] ^
                      frame_speed[7:0] ^ frame_speed[15:8] ^
                      frame_outer_p[7:0] ^ frame_outer_p[15:8] ^ frame_outer_p[23:16] ^ frame_outer_p[31:24] ^
                      frame_outer_d[7:0] ^ frame_outer_d[15:8] ^ frame_outer_d[23:16] ^ frame_outer_d[31:24] ^ frame_flags ^
                      frame_stop_reason ^ frame_stop_theta[7:0] ^ frame_stop_theta[15:8] ^ frame_stop_keys ^
                      frame_stop_raw[7:0] ^ frame_stop_raw[15:8] ^
                      frame_stop_filtered_raw[7:0] ^ frame_stop_filtered_raw[15:8] ^
                      frame_stop_min[7:0] ^ frame_stop_min[15:8] ^
                      frame_stop_max[7:0] ^ frame_stop_max[15:8] ^
                      frame_stop_sum[7:0] ^ frame_stop_sum[15:8] ^
                      frame_stop_sum[23:16] ^ frame_stop_sum[31:24] ^ frame_stop_count ^
                      frame_outer_i[7:0] ^ frame_outer_i[15:8] ^ frame_outer_i[23:16] ^ frame_outer_i[31:24] ^
                      8'd0 ^ 8'd16 ^ frame_limit_status ^
                      frame_active_limit[7:0] ^ frame_active_limit[15:8] ^
                      frame_remote_goal[7:0] ^ frame_remote_goal[15:8] ^ frame_remote_goal[23:16] ^ frame_remote_goal[31:24] ^
                      frame_position_target[7:0] ^ frame_position_target[15:8] ^ frame_position_target[23:16] ^ frame_position_target[31:24] ^
                      frame_command_seq ^ frame_command_status;
 // 帧头AA 5E，共76字节：65~68指令终点Q8，69~72轨迹目标Q8，73序号，74指令状态，75校验。
 // 串口连续发送，115200波特率、8位数据、无校验、1位停止位；每帧使用固定的数据快照。
 reg [7:0] data_byte;
 always @* begin
  case(byte_index)
   0: data_byte=8'hAA;
   1: data_byte=8'h5E;
   2: data_byte=frame_theta[7:0];
   3: data_byte=frame_theta[15:8];
   4: data_byte=frame_err32[7:0];
   5: data_byte=frame_err32[15:8];
   6: data_byte=frame_err32[23:16];
   7: data_byte=frame_err32[31:24];
   8: data_byte=frame_pos32[7:0];
   9: data_byte=frame_pos32[15:8];
   10: data_byte=frame_pos32[23:16];
   11: data_byte=frame_pos32[31:24];
   12: data_byte=frame_duty[7:0];
   13: data_byte=frame_duty[15:8];
   14: data_byte=frame_p[7:0];
   15: data_byte=frame_p[15:8];
   16: data_byte=frame_i[7:0];
   17: data_byte=frame_i[15:8];
   18: data_byte=frame_d[7:0];
   19: data_byte=frame_d[15:8];
   20: data_byte=frame_ref32[7:0];
   21: data_byte=frame_ref32[15:8];
   22: data_byte=frame_ref32[23:16];
   23: data_byte=frame_ref32[31:24];
   24: data_byte=frame_pos_error32[7:0];
   25: data_byte=frame_pos_error32[15:8];
   26: data_byte=frame_pos_error32[23:16];
   27: data_byte=frame_pos_error32[31:24];
   28: data_byte=frame_speed[7:0];
   29: data_byte=frame_speed[15:8];
   30: data_byte=frame_outer_p[7:0];
   31: data_byte=frame_outer_p[15:8];
   32: data_byte=frame_outer_p[23:16];
   33: data_byte=frame_outer_p[31:24];
   34: data_byte=frame_outer_d[7:0];
   35: data_byte=frame_outer_d[15:8];
   36: data_byte=frame_outer_d[23:16];
   37: data_byte=frame_outer_d[31:24];
   38: data_byte=frame_flags;
   39: data_byte=frame_stop_reason;
   40: data_byte=frame_stop_theta[7:0];
   41: data_byte=frame_stop_theta[15:8];
   42: data_byte=frame_stop_keys;
   43: data_byte=frame_stop_raw[7:0];
   44: data_byte=frame_stop_raw[15:8];
   45: data_byte=frame_stop_filtered_raw[7:0];
   46: data_byte=frame_stop_filtered_raw[15:8];
   47: data_byte=frame_stop_min[7:0];
   48: data_byte=frame_stop_min[15:8];
   49: data_byte=frame_stop_max[7:0];
   50: data_byte=frame_stop_max[15:8];
   51: data_byte=frame_stop_sum[7:0];
   52: data_byte=frame_stop_sum[15:8];
   53: data_byte=frame_stop_sum[23:16];
   54: data_byte=frame_stop_sum[31:24];
   55: data_byte=frame_stop_count;
   56: data_byte=frame_outer_i[7:0];
   57: data_byte=frame_outer_i[15:8];
   58: data_byte=frame_outer_i[23:16];
   59: data_byte=frame_outer_i[31:24];
   // 保留原AA5D帧的字节位置，旧倍率字段固定为1.0（4096），不参与控制。
   60: data_byte=8'd0;
   61: data_byte=8'd16;
   62: data_byte=frame_limit_status;
   63: data_byte=frame_active_limit[7:0];
   64: data_byte=frame_active_limit[15:8];
   65: data_byte=frame_remote_goal[7:0];
   66: data_byte=frame_remote_goal[15:8];
   67: data_byte=frame_remote_goal[23:16];
   68: data_byte=frame_remote_goal[31:24];
   69: data_byte=frame_position_target[7:0];
   70: data_byte=frame_position_target[15:8];
   71: data_byte=frame_position_target[23:16];
   72: data_byte=frame_position_target[31:24];
   73: data_byte=frame_command_seq;
   74: data_byte=frame_command_status;
   75: data_byte=checksum;
   default: data_byte=8'h00;
  endcase
 end
 always @(posedge clk_50m or negedge rst_n) begin
  if(!rst_n) begin baud_count<=0; bit_index<=0; byte_index<=0; end
  else if(baud_count==9'd433) begin
   baud_count<=0;
   if(bit_index==9) begin
    bit_index<=0;
    if(byte_index==75) byte_index<=0;
    else byte_index<=byte_index+1'b1;
   end else bit_index<=bit_index+1'b1;
  end else baud_count<=baud_count+1'b1;
 end
 assign uart_tx= !rst_n ? 1'b1 : bit_index==0 ? 1'b0 :
                 bit_index==9 ? 1'b1 : data_byte[bit_index-1'b1];
endmodule

