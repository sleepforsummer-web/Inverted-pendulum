module pid_p #
(
    parameter signed [31:0] Kp=32'sd524231// Q16比例系数：实际Kp为本参数除以65536
)
(
    input wire clk,//主时钟
    input wire rst,//复位
    input wire enable,//PID正常工作
    input wire sample_tick,// 采样使能脉冲，持续一个主时钟周期
    input wire signed[16:0] error,
    output reg signed[48:0] p,
    output reg p_valid
);
always@(posedge clk)begin
    if(rst||!enable)begin
        p<=49'sd0;
        p_valid<=1'b0;
    end
    else begin
        p_valid<=1'b0;
        if(sample_tick)begin
            p<=error*Kp;
            p_valid<=1'b1;
        end
    end
end
endmodule

module pid_d#
(
    parameter signed [31:0] Kd = 32'sd867527,
    // 0关闭滤波；1~8表示新差值权重为1/(2^FILTER_SHIFT)。
    parameter integer FILTER_SHIFT = 0
)
(
    input wire clk,
    input wire rst,
    input wire enable,
    input wire sample_tick,
    input wire signed[16:0] error,
    output reg signed[49:0] d,
    output reg d_valid,
    output wire signed [25:0] velocity_q8_mon
);
    reg signed[16:0] error_prev;
    reg have_prev;
    // 差分前显式扩展，覆盖17位有符号输入两端之间的变化。
    wire signed [17:0] delta = $signed({error[16],error}) -
                               $signed({error_prev[16],error_prev});
    reg signed [17:0] delta_sample;
    reg [1:0] filter_pending;
    // Q8保留不足一个ADC/编码器计数的差值，避免小速度被整数截断。
    // 18位差分乘256恰好需要26位；滤波为凸组合，不会超出差分输入范围。
    reg signed [25:0] filter_q8;
    // 复用D模块现有差分/滤波状态，避免模糊调度器另建速度滤波器。
    assign velocity_q8_mon=(FILTER_SHIFT==0) ? {delta_sample,8'b0} : filter_q8;
    wire signed [25:0] filter_next;
    wire signed [49:0] d_next;
    generate
        if (FILTER_SHIFT == 0) begin : unfiltered
            assign filter_next = 26'sd0;
            assign d_next = delta * Kd;
        end else begin : filtered
            wire signed [25:0] delta_q8 = {delta_sample,8'b0};
            wire signed [26:0] innovation =
                $signed({delta_q8[25],delta_q8}) - $signed({filter_q8[25],filter_q8});
            // 对称四舍五入：正负半数均远离0，避免直接算术右移造成负向偏置。
            wire signed [27:0] magnitude = innovation[26] ?
                -$signed({innovation[26],innovation}) : $signed({innovation[26],innovation});
            wire signed [27:0] rounded =
                (magnitude + (28'sd1 <<< (FILTER_SHIFT-1))) >>> FILTER_SHIFT;
            wire signed [27:0] correction = innovation[26] ? -rounded : rounded;
            wire signed [27:0] candidate =
                $signed({{2{filter_q8[25]}},filter_q8}) + correction;
            // 零速时清除量化残留，使正负方向都能回到真正的0。
            wire clear_residue = (delta_sample == 18'sd0) &&
                (filter_q8 > -(26'sd1 <<< FILTER_SHIFT)) &&
                (filter_q8 <  (26'sd1 <<< FILTER_SHIFT));
            assign filter_next = clear_residue ? 26'sd0 : candidate[25:0];
            // 差分、滤波、乘法分三个主时钟执行，避免单周期组合路径过长。
            // 相比原版只增加两个50MHz时钟（40ns），不是增加两个1ms采样周期。
            wire signed [57:0] product_q24 = filter_q8 * Kd;
            wire signed [57:0] product_q16 = product_q24 >>> 8;
            assign d_next = product_q16[49:0];
        end
    endgenerate
always@(posedge clk)begin
    if(rst||!enable)begin
        error_prev<=17'sd0;
        have_prev<=1'b0;
        filter_q8<=26'sd0;
        delta_sample<=18'sd0;
        filter_pending<=2'b00;
        d<=50'sd0;
        d_valid<=1'b0;
    end
    else begin
        d_valid<=1'b0;
        if(FILTER_SHIFT != 0) begin
            filter_pending <= {filter_pending[0],sample_tick};
            if(filter_pending[0]) filter_q8<=filter_next;
            if(filter_pending[1]) begin
                d<=d_next;
                d_valid<=1'b1;
            end
        end
        if(sample_tick)begin
            delta_sample <= have_prev ? delta : 18'sd0;
            if(FILTER_SHIFT == 0) begin
                d <= have_prev ? d_next : 50'sd0;
                d_valid<=1'b1;
            end
            error_prev<=error;
            have_prev<=1'b1;
        end
    end
end
endmodule

module pid_i #(
    parameter signed[31:0] Ki=32'sd33,
    parameter signed[48:0] I_MAX=49'sd6553600,//积分限幅
    parameter signed[16:0] I_SEP_THRESHOLD=17'sd50
)
(
    input wire clk,
    input wire rst,
    input wire enable,
    input wire sample_tick,
    input wire signed[16:0] error,
    output reg signed[48:0] i,
    output reg i_valid
);
    // 依据常数系数和积分上限推导乘加位宽，仍保留完整精度和原来的49位接口。
    function integer gain_bits;
        input signed [31:0] value;
        integer bit_n;
        begin
            gain_bits=32;
            for(bit_n=31;bit_n>=1;bit_n=bit_n-1)
                if((value >>> (bit_n-1))==0 || (value >>> (bit_n-1))==-1) gain_bits=bit_n;
        end
    endfunction
    function integer integral_bits;
        input signed [48:0] limit_value;
        integer bit_n;
        begin
            integral_bits=49;
            if(limit_value>=0)
                for(bit_n=49;bit_n>=1;bit_n=bit_n-1)
                    if(limit_value<=( (64'sd1 << (bit_n-1))-1)) integral_bits=bit_n;
        end
    endfunction
    localparam integer I_BITS=integral_bits(I_MAX);
    localparam integer INCREMENT_BITS=17+gain_bits(Ki);
    localparam integer CANDIDATE_BITS=(I_BITS>INCREMENT_BITS ? I_BITS : INCREMENT_BITS)+1;
    wire signed [INCREMENT_BITS-1:0] increment=Ki*error;
    wire signed [CANDIDATE_BITS-1:0] bounded_candidate /* synthesis syn_dspstyle="logic" */;
    assign bounded_candidate=$signed(i[I_BITS-1:0]) + increment;
    // 小常数积分乘加使用普通逻辑，避免整段49位加法占用专用DSP。
    wire signed[49:0] candidate /* synthesis syn_dspstyle="logic" */;//候选积分
    wire integrate_allowed=(error > -I_SEP_THRESHOLD) && (error < I_SEP_THRESHOLD);
    assign candidate=bounded_candidate;
always@(posedge clk)begin
    if(rst||!enable)begin
        i<=49'sd0;
        i_valid<=1'b0;
    end
    else begin
        i_valid<=1'b0;
        if(sample_tick)begin
           // 积分分离
           if(!integrate_allowed)begin
            i<=49'sd0;
           end
           else if(candidate>I_MAX)begin
            i<=I_MAX;
           end
           else if(candidate<-I_MAX)begin
            i<=-I_MAX;
           end
            else begin
                i <= candidate[48:0];
            end
            i_valid<=1'b1;
        end
    end
end
endmodule


//角度内环
module pid_controller #
(
    parameter signed [31:0] KP_Q = 32'sd524231,
    parameter signed [31:0] KD_Q = 32'sd867527,
    parameter integer D_FILTER_SHIFT = 0,
    // 实际输出限幅 ±1000，保存值放大 65536 倍
    parameter signed [50:0] OUT_MAX_Q = 51'sd65536000,
    parameter signed [31:0] KI_Q = 32'sd65,
    parameter signed [16:0] I_SEP_THRESHOLD = 17'sd50,
    // 内环P/I误差限幅，单位为ADC计数；0或负值关闭，D使用独立测量输入。
    parameter signed [16:0] ERROR_INPUT_LIMIT = 17'sd0
)
(
    input wire clk,
    input wire rst,
    input wire enable,
    input wire sample_tick,
    input wire signed [16:0] error,
    input wire signed [16:0] d_input,

    // 定义pid输出以及输出有效
    output reg signed [50:0] pid_sum,
    output reg pid_valid,

    // 监视口: P/I/D 各项 (Q16右移12位, 饱和16bit)
    output reg signed [15:0] p_mon,
    output reg signed [15:0] i_mon,
    output reg signed [15:0] d_mon
);

    // 连接三个子模块的输出
    wire signed [48:0] p;
    wire signed [48:0] i;
    wire signed [49:0] d;

    wire p_valid;
    wire i_valid;
    wire d_valid;

    // 滤波D多用两个主时钟；同步延迟P/I的数据与有效位，保持同一次采样求和。
    wire signed [48:0] p_aligned, i_aligned;
    wire p_aligned_valid, i_aligned_valid;
    generate if(D_FILTER_SHIFT == 0) begin : direct_pi
        assign p_aligned=p; assign i_aligned=i;
        assign p_aligned_valid=p_valid; assign i_aligned_valid=i_valid;
    end else begin : align_pi
        reg signed [48:0] p_d0, p_d1, i_d0, i_d1;
        reg [1:0] pv, iv;
        always @(posedge clk) begin
            if(rst || !enable) begin
                p_d0<=0; p_d1<=0; i_d0<=0; i_d1<=0; pv<=0; iv<=0;
            end else begin
                p_d0<=p; p_d1<=p_d0; i_d0<=i; i_d1<=i_d0;
                pv<={pv[0],p_valid}; iv<={iv[0],i_valid};
            end
        end
        assign p_aligned=p_d1; assign i_aligned=i_d1;
        assign p_aligned_valid=pv[1]; assign i_aligned_valid=iv[1];
    end endgenerate

    wire signed [16:0] error_pi;
    assign error_pi = (ERROR_INPUT_LIMIT <= 17'sd0) ? error :
                     (error > ERROR_INPUT_LIMIT) ? ERROR_INPUT_LIMIT :
                     (error < -ERROR_INPUT_LIMIT) ? -ERROR_INPUT_LIMIT : error;
    // 积分分离按实际误差判定；即使限幅设得比积分阈值小，也不能误开启积分。
    wire signed [16:0] error_i;
    assign error_i = (error > -I_SEP_THRESHOLD && error < I_SEP_THRESHOLD) ? error_pi : error;

    // P模块
    pid_p #(.Kp(KP_Q)) u_p (
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .sample_tick(sample_tick),
        .error(error_pi),
        .p(p),
        .p_valid(p_valid)
    );

    // I模块
    pid_i #(.Ki(KI_Q), .I_SEP_THRESHOLD(I_SEP_THRESHOLD)) u_i (
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .sample_tick(sample_tick),
        .error(error_i),
        .i(i),
        .i_valid(i_valid)
    );

    // D对独立测量输入d_input做差分，不受P/I误差限幅和外环目标角变化的直接影响。
    pid_d #(.Kd(KD_Q), .FILTER_SHIFT(D_FILTER_SHIFT)) u_d (
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .sample_tick(sample_tick),
        .error(d_input),
        .d(d),
        .d_valid(d_valid),.velocity_q8_mon()
    );

    // 先扩展到51位，防止求和溢出
    // 因为右边都是signed，这里自动符号扩展
    wire signed [50:0] p_ext;
    wire signed [50:0] i_ext;
    wire signed [50:0] d_ext;
    // 保持位宽与组合表达式不变，仅指定宽位求和使用逻辑进位链。
    wire signed [50:0] sum_next /* synthesis syn_dspstyle="logic" */;

    assign p_ext = p_aligned;
    assign i_ext = i_aligned;
    assign d_ext = d;

    assign sum_next = p_ext + i_ext + d_ext;

    // 等三个结果同时有效，再保存总和
always @(posedge clk) begin
    if (rst || !enable) begin
        pid_sum   <= 51'sd0;
        pid_valid <= 1'b0;
        p_mon <= 16'sd0;
        i_mon <= 16'sd0;
        d_mon <= 16'sd0;
    end
    else begin
        pid_valid <= 1'b0;

        if (p_aligned_valid && i_aligned_valid && d_valid) begin

            // 超过正向上限
            if (sum_next > OUT_MAX_Q)
                pid_sum <= OUT_MAX_Q;

            // 超过负向下限
            else if (sum_next < -OUT_MAX_Q)
                pid_sum <= -OUT_MAX_Q;

            // 没有超出范围，直接保存求和结果
            else
                pid_sum <= sum_next;

            pid_valid <= 1'b1;

            // 监视输出
            p_mon <= (p_ext >  51'sd134217728) ? 16'sd32767 :
                     (p_ext < -51'sd134217728) ? -16'sd32768 : p_ext[27:12];
            i_mon <= (i_ext >  51'sd134217728) ? 16'sd32767 :
                     (i_ext < -51'sd134217728) ? -16'sd32768 : i_ext[27:12];
            d_mon <= (d_ext >  51'sd134217728) ? 16'sd32767 :
                     (d_ext < -51'sd134217728) ? -16'sd32768 : d_ext[27:12];

        end
    end
end
endmodule

// 角度内环接口：接收绝对ADC目标值和实际测量值。
// 保持既有标定方向：INVERT=1时，正向摆角增大对应ADC减小，
// 因此PID误差为实际ADC减去目标ADC；D项只对带方向的实际测量值做差分。
module pid_adc_controller #(
    parameter ADC_INVERT = 1,
    parameter signed [31:0] KP_Q = 32'sd524231,
    parameter signed [31:0] KD_Q = 32'sd867527,
    parameter integer D_FILTER_SHIFT = 0,
    parameter signed [50:0] OUT_MAX_Q = 51'sd65536000,
    parameter signed [31:0] KI_Q = 32'sd65,
    parameter signed [16:0] I_SEP_THRESHOLD = 17'sd50,
    parameter signed [16:0] ERROR_INPUT_LIMIT = 17'sd0
)(
    input wire clk, rst, enable, sample_tick,
    input wire signed [16:0] target_adc,
    input wire signed [16:0] actual_adc,
    output wire signed [16:0] error_out,
    output wire signed [50:0] pid_sum,
    output wire pid_valid,
    output wire signed [15:0] p_mon, i_mon, d_mon
);
    wire signed [16:0] adc_error = target_adc - actual_adc;
    assign error_out = ADC_INVERT ? -adc_error : adc_error;
    // D使用测量值；方向与原来的误差微分保持一致
    wire signed [16:0] d_input;
    assign d_input = ADC_INVERT ? actual_adc : -actual_adc;

    pid_controller #(
        .KP_Q(KP_Q), .KD_Q(KD_Q), .D_FILTER_SHIFT(D_FILTER_SHIFT),
        .OUT_MAX_Q(OUT_MAX_Q), .KI_Q(KI_Q),
        .I_SEP_THRESHOLD(I_SEP_THRESHOLD), .ERROR_INPUT_LIMIT(ERROR_INPUT_LIMIT)
    ) inner_pid (
        .clk(clk), .rst(rst), .enable(enable), .sample_tick(sample_tick),
        .error(error_out), .pid_sum(pid_sum), .pid_valid(pid_valid),
        .p_mon(p_mon), .i_mon(i_mon), .d_mon(d_mon),.d_input(d_input)
    );
endmodule

//位置外环
module position_controller #(
    // 外环参数，Q16
    parameter signed [31:0] Kp_pos = 32'sd0,  // Q16比例系数：实际Kp为本参数除以65536
    parameter signed [31:0] Kd_pos = 32'sd0, // Q16差分系数：乘相邻外环采样的位置误差变化量
    // theta_ref 限
    parameter signed [16:0] THETA_REF_MAX = 17'sd40,
    parameter integer D_FILTER_SHIFT = 0,
    // 开关关闭时保持固定限幅，便于与原PD对照；顶层显式开启动态限幅。
    parameter LIMIT_SWITCH_ENABLE = 0,
    parameter signed [16:0] THETA_REF_BOOST_MAX = 17'sd80,
    parameter signed [16:0] LIMIT_POSITION_THRESHOLD = 17'sd50,
    parameter integer LIMIT_BOOST_MIN_MS = 1000,
    parameter integer POSITION_SAMPLE_MS = 10,
    // 自动起摆接管时触发一次定时150；到时无条件结束，再按常规双限幅调节。
    parameter STARTUP_BOOST_ENABLE = 0,
    parameter signed [16:0] STARTUP_BOOST_MAX = 17'sd150,
    parameter integer STARTUP_BOOST_MS = 1000,
    // 0保留旧双限幅；1在起摆收敛后启用独立的受扰限幅包络。
    parameter DISTURB_LIMIT_ENABLE = 0,
    parameter signed [16:0] DISTURB_LIMIT_MAX = 17'sd200,
    parameter integer DISTURB_LIMIT_RAMP_MS = 1000,
    // 运动目标保留Q16小数；D只对实际位置差分，防止目标运动产生微分冲击。
    parameter USE_Q16_POSITION = 0,
    parameter D_ON_MEASUREMENT = 0,
    parameter signed [31:0] Ki_pos = 32'sd0,
    parameter signed [48:0] I_POS_MAX_Q = 49'sd524288, // 外环I独立限幅±8 ADC，Q16
    parameter signed [16:0] I_POS_SEP_THRESHOLD = 17'sd50 // 圆整位置误差小于50计数才积分
)
(
    input  wire clk,
    input  wire rst,
    input  wire enable,
    input  wire sample_tick,

    // 位置误差
    input  wire signed [16:0] position_error,
    input  wire signed [32:0] position_error_q16,
    input  wire signed [16:0] position_measurement,
    input  wire integral_clear, // 运动时清除积分；仍按正常节拍产生有效零I
    input  wire motion_active, // 运动请求至目标轨迹完成：强制小限幅并清除大限幅计时
    input  wire startup_boost_arm, // 仅自动起摆进入PID时一拍；默认关闭，不影响旧接口实例

    // 外环输出，给内环作为目标角度
    output reg  signed [16:0] theta_ref,
    output reg                theta_ref_valid,
    // 仅供上位机诊断：P/D项为目标角限幅前的贡献值，采用Q8格式，真实值需除以256。
    output reg signed [31:0] p_pos_mon,
    output reg signed [31:0] d_pos_mon,
    output reg signed [31:0] i_pos_mon,
    output reg              limited_mon,
    // 与theta_ref同一帧的限幅状态；起摆固定时长，常规大限幅需同时满足时间及回区。
    output reg signed [16:0] active_limit_mon,
    output reg              boost_active_mon,
    output reg [2:0]        limit_phase_mon // 仅诊断，不参与控制计算
);

    // 进入条件：|位置误差|>50。退出条件：已满1秒 AND |位置误差|<=50。
    // 进入后的计时连续进行，不因提前进入小误差区、再次出区或误差变号而重置。
    localparam integer BOOST_HOLD_TICKS =
        (LIMIT_BOOST_MIN_MS+POSITION_SAMPLE_MS-1)/POSITION_SAMPLE_MS;
    localparam integer STARTUP_HOLD_TICKS =
        (STARTUP_BOOST_MS+POSITION_SAMPLE_MS-1)/POSITION_SAMPLE_MS;
    localparam integer MAX_HOLD_TICKS = BOOST_HOLD_TICKS>STARTUP_HOLD_TICKS ? BOOST_HOLD_TICKS : STARTUP_HOLD_TICKS;
    // 起摆与常规阶段复用计数器，阶段切换时重新计时。
    localparam integer BOOST_COUNT_BITS = MAX_HOLD_TICKS > 0 ? $clog2(MAX_HOLD_TICKS+1) : 32;
    wire signed [32:0] limit_position_q16=$signed({LIMIT_POSITION_THRESHOLD,16'b0});
    wire position_outside = USE_Q16_POSITION ?
                            ((position_error_q16>limit_position_q16) ||
                             (position_error_q16< -limit_position_q16)) :
                            ((position_error > LIMIT_POSITION_THRESHOLD) ||
                             (position_error < -LIMIT_POSITION_THRESHOLD));
    reg boost_mode;
    reg startup_boost_active;
    wire startup_requested=STARTUP_BOOST_ENABLE && (startup_boost_active || startup_boost_arm);
    reg [BOOST_COUNT_BITS-1:0] boost_elapsed_ticks;
    wire startup_timeout = startup_requested && !startup_boost_arm && boost_mode &&
        (STARTUP_HOLD_TICKS==0 || boost_elapsed_ticks>=STARTUP_HOLD_TICKS-1);
    wire startup_sample_active = startup_requested && !startup_timeout;
    reg boost_next;
    reg [BOOST_COUNT_BITS-1:0] elapsed_next;
    always @* begin
        boost_next=boost_mode;
        elapsed_next=boost_elapsed_ticks;
        if(!LIMIT_SWITCH_ENABLE || motion_active) begin boost_next=0; elapsed_next=0; end
        else if(STARTUP_BOOST_ENABLE && startup_boost_arm) begin boost_next=1; elapsed_next=0; end
        else if(!boost_mode) begin
            elapsed_next=0;
            if(position_outside || startup_requested) boost_next=1;
        end else if(startup_requested) begin
            if(startup_timeout) begin
                // 满1秒就退出150：区外直接转常规大限幅，区内转小限幅。
                boost_next=position_outside;
                elapsed_next=0;
            end else if(boost_elapsed_ticks<STARTUP_HOLD_TICKS)
                elapsed_next=boost_elapsed_ticks+1'b1;
        end else begin
            if(boost_elapsed_ticks<BOOST_HOLD_TICKS)
                elapsed_next=boost_elapsed_ticks+1'b1;
            // 第一次大限幅样本为t=0；第100个后续10ms节拍恰好满1秒。
            if((BOOST_HOLD_TICKS==0 || boost_elapsed_ticks>=BOOST_HOLD_TICKS-1) &&
               !position_outside) begin boost_next=0; elapsed_next=0; end
        end
    end
    wire signed [16:0] startup_sample_limit = boost_next ?
        (startup_sample_active ? STARTUP_BOOST_MAX : THETA_REF_BOOST_MAX) : THETA_REF_MAX;
    wire signed [16:0] sample_limit;
    wire sample_boost;
    wire [2:0] sample_phase;
    generate if(DISTURB_LIMIT_ENABLE && LIMIT_SWITCH_ENABLE) begin : disturbance_envelope
        disturbance_limit #(.NEAR_ADC(THETA_REF_MAX),.DISTURB_ADC(DISTURB_LIMIT_MAX),
            .RAMP_MS(DISTURB_LIMIT_RAMP_MS),.SAMPLE_MS(POSITION_SAMPLE_MS),
            .STARTUP_MIN_MS(LIMIT_BOOST_MIN_MS)) envelope(
            .clk(clk),.rst(rst),.enable(enable),.sample_tick(sample_tick),.motion_active(motion_active),
            .position_outside(position_outside),.startup_limit(startup_sample_limit),
            .startup_boost(boost_next),.sample_limit(sample_limit),.sample_boost(sample_boost),
            .sample_phase(sample_phase));
    end else begin : legacy_limit
        assign sample_limit=startup_sample_limit;
        assign sample_boost=boost_next;
        // 旧双限幅只有常规小/大两种状态，不存在等待回区或受扰回落阶段。
        assign sample_phase=LIMIT_SWITCH_ENABLE ?
            ((boost_next && startup_sample_active) ? 3'd1 : (boost_next ? 3'd7 : 3'd6)) : 3'd0;
    end endgenerate
    reg signed [16:0] limit_d0, limit_d1, limit_d2, limit_d3;
    reg boost_d0, boost_d1, boost_d2, boost_d3;
    reg [2:0] phase_d0, phase_d1, phase_d2, phase_d3;
    wire signed [16:0] aligned_limit = (D_FILTER_SHIFT==0) ? limit_d1 : limit_d3;
    // 运动开始时也覆盖流水线中尚未输出的旧大限幅样本。
    wire signed [16:0] result_limit = motion_active ? THETA_REF_MAX : aligned_limit;
    wire result_boost = !motion_active && ((D_FILTER_SHIFT==0) ? boost_d1 : boost_d3);
    wire [2:0] result_phase=motion_active ? 3'd5 : ((D_FILTER_SHIFT==0) ? phase_d1 : phase_d3);
    always @(posedge clk) begin
        if(rst || !enable) begin
            boost_mode<=0; boost_elapsed_ticks<=0;
            startup_boost_active<=0;
            limit_d0<=THETA_REF_MAX; limit_d1<=THETA_REF_MAX; limit_d2<=THETA_REF_MAX; limit_d3<=THETA_REF_MAX;
            boost_d0<=0; boost_d1<=0; boost_d2<=0; boost_d3<=0;
            phase_d0<=0; phase_d1<=0; phase_d2<=0; phase_d3<=0;
        end else begin
            if(motion_active) startup_boost_active<=0;
            else if(STARTUP_BOOST_ENABLE && startup_boost_arm) startup_boost_active<=1;
            else if(sample_tick && (startup_timeout || (boost_mode && !boost_next))) startup_boost_active<=0;
            // 非采样时刻收到新起摆事件，也必须清除上一轮常规限幅计时。
            if(STARTUP_BOOST_ENABLE && startup_boost_arm) begin boost_mode<=0; boost_elapsed_ticks<=0; end
            // 运动期间每个主时钟都清除，不能把运动前/期间的计时带到运动完成之后。
            if(motion_active) begin boost_mode<=0; boost_elapsed_ticks<=0; end
            if(sample_tick) begin
                boost_mode<=boost_next; boost_elapsed_ticks<=elapsed_next;
                limit_d0<=sample_limit;
                boost_d0<=sample_boost;
                phase_d0<=sample_phase;
            end
            // 随同P/D流水线对齐，避免用新样本的限幅裁剪上一组PID结果。
            limit_d1<=limit_d0; limit_d2<=limit_d1; limit_d3<=limit_d2;
            boost_d1<=boost_d0; boost_d2<=boost_d1; boost_d3<=boost_d2;
            phase_d1<=phase_d0; phase_d2<=phase_d1; phase_d3<=phase_d2;
        end
    end

    // -----------------------------
    // 外环 P、D 输出
    // -----------------------------
    reg sample_d;
    reg signed [16:0] position_error_d, derivative_input_d, integral_error_d;
    reg signed [32:0] position_error_q16_d;
    wire signed [48:0] p_pos;
    wire signed [49:0] d_pos;
    wire signed [48:0] i_pos;
    wire signed [25:0] outer_velocity_q8;

    wire p_pos_valid;
    wire d_pos_valid;
    wire i_pos_valid;
    wire signed [48:0] i_pos_aligned;
    wire i_pos_aligned_valid;
    wire signed [48:0] p_pos_aligned;
    wire p_pos_aligned_valid;
    generate if(D_FILTER_SHIFT == 0) begin : direct_outer_p
        assign p_pos_aligned=p_pos;
        assign p_pos_aligned_valid=p_pos_valid;
        assign i_pos_aligned=i_pos;
        assign i_pos_aligned_valid=i_pos_valid;
    end else begin : align_outer_p
        reg signed [48:0] p_d0, p_d1;
        reg [1:0] pv;
        reg signed [48:0] i_d0, i_d1;
        reg [1:0] iv;
        always @(posedge clk) begin
            if(rst || !enable) begin p_d0<=0; p_d1<=0; pv<=0; i_d0<=0; i_d1<=0; iv<=0; end
            else begin
                p_d0<=p_pos; p_d1<=p_d0; pv<={pv[0],p_pos_valid};
                i_d0<=i_pos; i_d1<=i_d0; iv<={iv[0],i_pos_valid};
            end
        end
        assign p_pos_aligned=p_d1;
        assign p_pos_aligned_valid=pv[1];
        assign i_pos_aligned=i_d1;
        assign i_pos_aligned_valid=iv[1];
    end endgenerate

    // -----------------------------
    // 调用原来的 P 模块
    // -----------------------------
    generate if(USE_Q16_POSITION) begin : fractional_position_p
        reg signed [48:0] p_value;
        reg p_valid;
        wire signed [64:0] product = position_error_q16_d*Kp_pos;
        always @(posedge clk) begin
            if(rst || !enable) begin p_value<=0; p_valid<=0; end
            else begin
                p_valid<=sample_d;
                if(sample_d) p_value<=product>>>16;
            end
        end
        assign p_pos=p_value;
        assign p_pos_valid=p_valid;
    end else begin : integer_position_p
    pid_p #(
        .Kp(Kp_pos)
    ) u_pos_p (
        .clk        (clk),
        .rst        (rst),
        .enable     (enable),
        .sample_tick(sample_d),
        .error      (position_error_d),

        .p          (p_pos),
        .p_valid    (p_pos_valid)
    );
    end endgenerate

    // -----------------------------
    // 调用原来的 D 模块
    // -----------------------------
    pid_d #(
        .Kd(Kd_pos), .FILTER_SHIFT(D_FILTER_SHIFT)
    ) u_pos_d (
        .clk        (clk),
        .rst        (rst),
        .enable     (enable),
        .sample_tick(sample_d),
        .error      (derivative_input_d),

        .d          (d_pos),
        .d_valid    (d_pos_valid),
        .velocity_q8_mon(outer_velocity_q8)
    );

    // -----------------------------
    // 扩展到统一位宽后求和
    // -----------------------------
    // 复用原pid_i，不修改内环。Q16误差正负对称四舍五入，避免负数截断偏置。
    // 小于半个编码器计数不积分；积分分离及I_MAX由原模块执行。
    wire round_integral_up=position_error_q16[15] &&
        (!position_error_q16[32] || (|position_error_q16[14:0]));
    wire signed [17:0] integral_rounded=
        $signed({position_error_q16[32],position_error_q16[32:16]})+
        (round_integral_up ? 18'sd1 : 18'sd0);
    wire signed [16:0] integral_error_q16=
        integral_rounded>18'sd65535 ? 17'sd65535 :
        integral_rounded< -18'sd65536 ? -17'sd65536 : integral_rounded[16:0];
    wire signed [16:0] integral_error=integral_clear ? I_POS_SEP_THRESHOLD :
        (USE_Q16_POSITION ? integral_error_q16 : position_error);
    // 先锁存同一次采样的全部输入，拆开Q16误差换算与积分乘加路径；仅增加20ns。
    always @(posedge clk) begin
        if(rst || !enable) begin
            sample_d<=0; position_error_d<=0; derivative_input_d<=0;
            integral_error_d<=0; position_error_q16_d<=0;
        end else begin
            sample_d<=sample_tick;
            if(sample_tick) begin
                position_error_d<=position_error; position_error_q16_d<=position_error_q16;
                derivative_input_d<=D_ON_MEASUREMENT ? position_measurement : position_error;
                integral_error_d<=integral_error;
            end
        end
    end
    pid_i #(.Ki(Ki_pos),.I_MAX(I_POS_MAX_Q),.I_SEP_THRESHOLD(I_POS_SEP_THRESHOLD)) u_pos_i(
        .clk(clk),.rst(rst),.enable(enable),.sample_tick(sample_d),.error(integral_error_d),
        .i(i_pos),.i_valid(i_pos_valid));

    wire signed [50:0] p_pos_ext ={{2{p_pos_aligned[48]}}, p_pos_aligned};

    wire signed [50:0] d_pos_ext ={{1{d_pos[49]}}, d_pos};
    wire signed [50:0] i_pos_ext ={{2{i_pos_aligned[48]}}, i_pos_aligned};

    wire original_valid=p_pos_aligned_valid && i_pos_aligned_valid && d_pos_valid;
    // 固定系数外环：P/I/D按同一采样对齐后直接相加。
    wire signed [50:0] pos_sum_q /* synthesis syn_dspstyle="logic" */;
    assign pos_sum_q =p_pos_ext + i_pos_ext + d_pos_ext;

    // Q16还原
    wire signed [34:0] theta_ref_calc =pos_sum_q >>> 16;
    wire signed [50:0] p_pos_q8 = p_pos_ext >>> 8;
    wire signed [50:0] d_pos_q8 = d_pos_ext >>> 8;
    wire signed [50:0] i_pos_q8 = i_pos_ext >>> 8;

    // -----------------------------
    // 输出 theta_ref + 限幅
    // -----------------------------
    always @(posedge clk) begin
        if(rst || !enable) begin
            theta_ref       <= 17'sd0;
            theta_ref_valid <= 1'b0;
            p_pos_mon <= 32'sd0;
            d_pos_mon <= 32'sd0;
            i_pos_mon <= 32'sd0;
            limited_mon <= 1'b0;
            active_limit_mon <= THETA_REF_MAX;
            boost_active_mon <= 1'b0;

            limit_phase_mon <= 3'd0;
        end

        else begin
            theta_ref_valid <= 1'b0;

            if(original_valid) begin

                p_pos_mon <= (p_pos_q8 > 51'sd2147483647) ? 32'sd2147483647 :
                             (p_pos_q8 < -51'sd2147483648) ? -32'sd2147483648 : p_pos_q8[31:0];
                d_pos_mon <= (d_pos_q8 > 51'sd2147483647) ? 32'sd2147483647 :
                             (d_pos_q8 < -51'sd2147483648) ? -32'sd2147483648 : d_pos_q8[31:0];
                i_pos_mon <= (i_pos_q8 > 51'sd2147483647) ? 32'sd2147483647 :
                             (i_pos_q8 < -51'sd2147483648) ? -32'sd2147483648 : i_pos_q8[31:0];
                active_limit_mon <= result_limit;
                limit_phase_mon <= result_phase;
                boost_active_mon <= LIMIT_SWITCH_ENABLE && result_boost;
                limited_mon <= (theta_ref_calc > result_limit) || (theta_ref_calc < -result_limit);

                if(theta_ref_calc > result_limit)
                    theta_ref <= result_limit;

                else if(theta_ref_calc < -result_limit)
                    theta_ref <= -result_limit;

                else
                    theta_ref <= theta_ref_calc[16:0];

                theta_ref_valid <= 1'b1;
            end
        end
    end

endmodule
