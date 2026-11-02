//========================================================================
// motor_pwm.v —— PID 输出 -> TB6612 电机驱动 (单电机, 以电机A为例)
//
// pid_sum 是 Q16 定点数(已限幅±1000*2^16), 右移16位得到 ±1000 的占空比指令,
// PWM 频率 20kHz (50MHz/2500), |duty|<=1000 -> 最大约 80% 占空比(保护电机).
//
// 安全保护:
//   1. enable=0 时输出全关（滑行），由顶层的按键和运行保护控制
//   2. |theta| > FALL_THRESH 判定角度越界，强制关断
//   3. DEAD_BAND 死区, 防止平衡点附近电机嗡嗡作响
//   4. MOTOR_SIGN: 若实测电机方向使系统发散(越摆越倒), 把它改成 0
//      (等价于交换 AN1/AN2, 但不用重新布线)
//========================================================================
module motor_pwm #(
    parameter signed [12:0] FALL_THRESH = 13'sd900,  // 默认角度越界阈值，单位为ADC计数，顶层可覆盖
    parameter        [15:0] DEAD_BAND   = 16'd15,    // 死区
    parameter               MOTOR_SIGN  = 1         // 电机方向符号
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,     // 电机总使能，由顶层控制起摆或PID是否运行
    input  wire signed [12:0] theta,      // 用于倒地保护
    input  wire signed [50:0] pid_sum,    // Q16 定点 PID 输出
    input  wire               pid_valid,
    output reg                pwm_out,    // 接 TB6612 PWMA
    output reg                in1,        // 接 TB6612 AIN1
    output reg                in2,        // 接 TB6612 AIN2
    output reg  signed [16:0] duty_out    // 供串口监视
);

//---------------- 20kHz PWM 载波 ----------------
reg [11:0] cnt;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)            cnt <= 12'd0;
    else if(cnt == 2499)  cnt <= 12'd0;
    else                  cnt <= cnt + 1'b1;
end

//---------------- Q16 -> 占空比, 限幅 ±1000 ----------------
wire signed [50:0] duty_s = pid_sum >>> 16;
wire signed [16:0] duty_clamped =
      (duty_s >  51'sd1000) ?  17'sd1000 :
      (duty_s < -51'sd1000) ? -17'sd1000 : duty_s[16:0];
wire signed [16:0] duty_signed = MOTOR_SIGN ? duty_clamped : -duty_clamped;
wire fail = (theta > FALL_THRESH) || (theta < -FALL_THRESH);

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        duty_out <= 17'sd0;

    else if(!enable || fail)
        duty_out <= 17'sd0;

    else if(pid_valid)
        duty_out <= duty_signed;
end

//---------------- 绝对值 / 死区 / 比较电平 ----------------
wire [15:0] duty_abs =
    duty_out[16] ? (~duty_out[15:0] + 16'd1)
                 : duty_out[15:0];
wire        dead     = (duty_abs < DEAD_BAND);
wire [11:0] cmp      = dead ? 12'd0 : {duty_abs[10:0], 1'b0};  // <=2000

//---------------- 倒地保护与输出 ----------------
wire run  = enable && !fail;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        pwm_out <= 1'b0; in1 <= 1'b0; in2 <= 1'b0;
    end else if(!run) begin
        pwm_out <= 1'b0; in1 <= 1'b0; in2 <= 1'b0;   // 关断/滑行
    end else begin
        in1     <= ~duty_out[16];   // duty>=0 正转, <0 反转
        in2     <=  duty_out[16];
        pwm_out <= (cnt < cmp);
    end
end

endmodule
