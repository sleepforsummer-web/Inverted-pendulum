// 反馈起摆：连续采样，使用最近三个相邻采样点组成滑动窗口判断最高点。
// 底部启动时可先执行一次初始双脉冲，随后由最高点检测触发起摆双脉冲。
// 下面是模块默认参数；实际工程以顶层 pid_trial.v 实例化时传入的参数为准。
module swing_up #(
    parameter [11:0] CENTER_ADC = 12'd1958, // 摆杆竖直向上的中心ADC读数
    parameter [11:0] CAPTURE_RANGE = 12'd300, // 进入PID的中心区间半宽，单位为ADC计数
    parameter integer SAMPLE_PERIOD_MS = 40, // 最高点与PID捕获判断共用采样间隔；顶层当前传入20ms
    parameter integer PULSE_MS = 100, // 最高点双脉冲每一段的持续时间，单位为毫秒
    parameter signed [16:0] PULSE_DUTY = 17'sd500,// 最高点驱动指令幅值，不是占空比百分数
    // 左区域为 ADC > 中心值+捕获半宽；第一段方向由下面的参数指定。
    // 当前电机方向约定：正指令对应向右／顺时针，负指令对应相反方向。
    parameter LEFT_FIRST_POSITIVE = 1, // 1：左区域先正后负；0：先负后正；右区域顺序相反
    parameter signed [16:0] START_KICK_DUTY = 17'sd300,// 初始双脉冲幅值；顶层可覆盖，设0关闭
    parameter integer START_KICK_MS = 100, // 初始双脉冲每一段的持续时间，单位为毫秒
    parameter START_KICK_POSITIVE = 1, // 初始第一段的方向：1为正向，0为负向
    parameter [11:0] START_BOTTOM_LOW = 12'd200, // 底部低端判据：ADC不大于此值
    parameter [11:0] START_BOTTOM_HIGH = 12'd3890 // 底部高端判据：ADC不小于此值
)(
    input wire clk, rst_n,
    input wire enable, // 自动起摆总使能；关闭时立即撤销起摆指令
    input wire tick_1ms, // 每1ms有效一个主时钟周期
    input wire adc_valid, // ADC测量有效脉冲；首次有效后允许状态机工作
    input wire [11:0] adc_value, // 当前滤波后的绝对ADC读数
    output wire balance_mode, // 已进入PID平衡状态
    output wire pulse_active, // 初始或最高点驱动脉冲正在输出
    output reg signed [16:0] duty_command, // 带方向的电机驱动指令
    output reg [2:0] state, // 当前状态编号，定义见下方
    output reg peak_event, // 最高点触发脉冲，持续一个主时钟周期
    output reg [1:0] peak_region // 最近触发最高点的区域：1左侧，2右侧
);
    // 状态：停止、等待最高点、起摆第一段、起摆反向段、PID平衡。
    localparam [2:0] STOP=0, JUDGE=1, FIRST=2, REVERSE=3, BALANCE=4;
    // 初始动作状态：第一段、等待1ms边界、反向段。
    localparam [2:0] START_KICK=5, START_ARM=6, START_REVERSE=7;
    localparam [1:0] INVALID=0, LEFT=1, RIGHT=2;
    localparam [12:0] LOWER = {1'b0,CENTER_ADC}-{1'b0,CAPTURE_RANGE};
    localparam [12:0] UPPER = {1'b0,CENTER_ADC}+{1'b0,CAPTURE_RANGE};
    wire [12:0] adc_ext = {1'b0,adc_value};
    wire in_capture = adc_ext >= LOWER && adc_ext <= UPPER;
    wire in_bottom = adc_value <= START_BOTTOM_LOW || adc_value >= START_BOTTOM_HIGH;
    wire kick_enabled = START_KICK_DUTY > 0 && START_KICK_MS > 0;
    wire [1:0] region = (adc_ext > UPPER && adc_value < 12'd4090) ? LEFT :
                        (adc_value > 0 && adc_ext < LOWER) ? RIGHT : INVALID;
    reg adc_seen;
    // 计时器只需覆盖配置的毫秒范围；保留非正参数原有的32位回卷语义。
    localparam integer SAMPLE_COUNT_BITS = SAMPLE_PERIOD_MS > 1 ? $clog2(SAMPLE_PERIOD_MS) :
                                           SAMPLE_PERIOD_MS == 1 ? 1 : 32;
    localparam integer MAX_PULSE_MS = PULSE_MS > START_KICK_MS ? PULSE_MS : START_KICK_MS;
    localparam integer PULSE_COUNT_BITS = (PULSE_MS <= 0 || START_KICK_MS <= 0) ? 32 :
                                          MAX_PULSE_MS > 1 ? $clog2(MAX_PULSE_MS) : 1;
    reg [SAMPLE_COUNT_BITS-1:0] sample_ms;
    reg [PULSE_COUNT_BITS-1:0] pulse_ms;
    reg [1:0] sample_count;
    reg [1:0] capture_count; // 连续处于中心区的采样次数；第三次采样到来时切换PID
    reg [11:0] sample1, sample2;
    reg [1:0] region1, region2;
    reg first_positive;
    wire sample_due = tick_1ms && adc_seen && sample_ms == SAMPLE_PERIOD_MS-1;
    wire capture_confirmed = sample_due && in_capture && capture_count == 2;
    // 三个相邻点必须同侧；左侧取中间最小值，右侧取中间最大值。
    // 使用严格不等式，相等平台不触发；第三点到来后才能确认第二点为最高点。
    wire same_region = region != INVALID && region == region1 && region == region2;
    wire left_peak = same_region && region == LEFT &&
                     sample2 < sample1 && sample2 < adc_value;
    wire right_peak = same_region && region == RIGHT &&
                      sample2 > sample1 && sample2 > adc_value;
    wire accept_peak = state == JUDGE && sample_due && sample_count == 2 &&
                       (left_peak || right_peak);
    assign balance_mode = enable && state == BALANCE;
    assign pulse_active = enable && (state == FIRST || state == REVERSE ||
                                    state == START_KICK || state == START_REVERSE);

    always @* begin
        duty_command = 17'sd0;
        if(enable && state == FIRST)
            duty_command = first_positive ? PULSE_DUTY : -PULSE_DUTY;
        else if(enable && state == REVERSE)
            duty_command = first_positive ? -PULSE_DUTY : PULSE_DUTY;
        else if(enable && state == START_KICK)
            duty_command = START_KICK_POSITIVE ? START_KICK_DUTY : -START_KICK_DUTY;
        else if(enable && state == START_REVERSE)
            duty_command = START_KICK_POSITIVE ? -START_KICK_DUTY : START_KICK_DUTY;
    end

    // 两段驱动脉冲期间仍连续采样并更新窗口，不排队保存过时的最高点。
    // 仅在等待最高点状态使用当前窗口触发，避免正在执行的双脉冲被重复覆盖。
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            sample_ms<=0; sample_count<=0; capture_count<=0;
            sample1<=0; sample2<=0; region1<=INVALID; region2<=INVALID;
        end else if(!enable || state==STOP || state==BALANCE) begin
            sample_ms<=0; sample_count<=0; capture_count<=0;
            region1<=INVALID; region2<=INVALID;
        end else if(tick_1ms && adc_seen) begin
            if(sample_due) begin
                sample_ms<=0;
                // 仅在周期采样点累计；任一采样点出区后，连续次数重新从0开始。
                if(!in_capture) capture_count<=0;
                else if(capture_count<3) capture_count<=capture_count+1'b1;
                sample1<=sample2; region1<=region2;
                sample2<=adc_value; region2<=region;
                if(sample_count<2) sample_count<=sample_count+1'b1;
            end else sample_ms<=sample_ms+1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=STOP; adc_seen<=0;
            pulse_ms<=0;
            first_positive<=0; peak_event<=0; peak_region<=INVALID;
        end else begin
            if(adc_valid) adc_seen<=1;
            peak_event<=0;
            if(!enable) begin
                state<=STOP; pulse_ms<=0; peak_region<=INVALID;
            end else if(state==STOP && adc_seen) begin
                // 仅本次启动入口允许初始双脉冲；等待状态不会反复执行初始动作。
                state <= kick_enabled && in_bottom ? START_ARM : JUDGE;
                pulse_ms<=0;
            end else if(tick_1ms && adc_seen) begin
                // 任意起摆状态下，连续三个采样点在中心区才交给PID，可提前结束驱动脉冲。
                // 当前只按角度判断捕获，没有角速度门限或底部盲区屏蔽。
                if(state!=BALANCE && capture_confirmed) begin
                    state<=BALANCE; pulse_ms<=0;
                end else case(state)
                    START_ARM: begin
                        // 对齐1ms节拍边界后开始初始动作，保证每段计时完整。
                        state <= in_bottom ? START_KICK : JUDGE;
                        pulse_ms<=0;
                    end
                    START_KICK: begin
                        if(pulse_ms == START_KICK_MS-1) begin
                            state<=START_REVERSE; pulse_ms<=0;
                        end else pulse_ms<=pulse_ms+1'b1;
                    end
                    START_REVERSE: begin
                        if(pulse_ms == START_KICK_MS-1) begin
                            state<=JUDGE; pulse_ms<=0;
                        end else pulse_ms<=pulse_ms+1'b1;
                    end
                    JUDGE: begin
                        if(accept_peak) begin
                            state<=FIRST; pulse_ms<=0;
                            peak_event<=1; peak_region<=region;
                            first_positive <= left_peak ? LEFT_FIRST_POSITIVE :
                                                          !LEFT_FIRST_POSITIVE;
                        end
                    end
                    FIRST: begin
                        if(pulse_ms == PULSE_MS-1) begin
                            state<=REVERSE; pulse_ms<=0;
                        end else pulse_ms<=pulse_ms+1'b1;
                    end
                    REVERSE: begin
                        if(pulse_ms == PULSE_MS-1) begin
                            state<=JUDGE; pulse_ms<=0;
                        end else pulse_ms<=pulse_ms+1'b1;
                    end
                    BALANCE: begin
                        // 进入平衡后保持PID接管，直到顶层停机逻辑撤销使能。
                        pulse_ms<=0;
                    end
                    default: state<=STOP;
                endcase
            end
        end
    end
endmodule
