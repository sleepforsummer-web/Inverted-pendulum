// ============================================================
// sine_motion.v
//
// 每10ms更新一次位置目标。
// KEY3：启动 / 停止；由顶层与KEY4单次运动互斥。
//
// 正常运行：
// target = anchor + amplitude * sin(2*pi*t/T)
//
// 启动：一个周期内使用余弦包络逐渐增加幅值。
// IMMEDIATE_STOP=1：再次按键立即切回原目标；0：周期边界渐出。
// 最终保持anchor，不关闭平衡控制。
//
// load_target：自动恢复后重设anchor，取消旧运动。
// !enable：取消运动，target和anchor恢复为0，
//          与原顶层首次启动保留复位零点的逻辑一致。
//
// 参数要求：
// COUNTS_PER_REV > 0
// AMPLITUDE_DEG > 0
// PERIOD_TICKS >= 100
// PERIOD_TICKS不应大到超出32位计数范围。
//
// 当前推荐：1073计数/圈，±30°，8秒周期。
// ============================================================

module sine_motion #(
    parameter integer COUNTS_PER_REV = 1073,
    parameter integer AMPLITUDE_DEG = 30,

    // 每个节拍10ms：800对应8秒。
    parameter integer PERIOD_TICKS = 800,

    // 启动请求允许的实际状态范围。
    parameter integer START_THETA_ADC = 100,
    parameter integer START_SPEED = 3,
    parameter USE_START_ANCHOR = 0, // 顶层置1，从现有旋臂目标中心开始
    parameter IMMEDIATE_STOP = 0 // 置1时再次按键立即切回原目标
)(
    input wire clk,
    input wire rst_n,
    input wire enable,
    input wire sample_tick,
    input wire start_event,
    input wire signed [32:0] start_anchor_q16,

    input wire load_target,
    input wire signed [16:0] load_position,

    input wire signed [15:0] actual_speed,
    input wire signed [12:0] theta,

    output reg signed [32:0] target_q16,
    output wire busy,
    output reg target_valid,

    output reg range_rejected,
    output wire motion_active,

    output reg arrived,
    output reg request_rejected,
    output wire stop_pending_mon,
    output wire [2:0] state_mon
);

    localparam [2:0] IDLE    = 3'd0;
    localparam [2:0] FADE_IN = 3'd1;
    localparam [2:0] RUN     = 3'd2;
    localparam [2:0] FADE_OUT= 3'd3;

    localparam integer SAFE_PERIOD =
        (PERIOD_TICKS >= 100) ? PERIOD_TICKS : 100;
    localparam integer COUNT_BITS=$clog2(SAFE_PERIOD+1);

    // 编码器计数，保留16位小数。
    localparam signed [33:0] AMPLITUDE_Q16 =
        (64'sd65536 * COUNTS_PER_REV * AMPLITUDE_DEG) / 360;
    function integer amplitude_bits;
        input [33:0] value;
        integer bit_n;
        begin
            amplitude_bits=2;
            for(bit_n=0;bit_n<33;bit_n=bit_n+1)
                if(value>>bit_n) amplitude_bits=bit_n+2;
        end
    endfunction
    localparam integer DISTANCE_BITS=amplitude_bits(AMPLITUDE_Q16);

    localparam signed [33:0] TARGET_MAX =
        34'sd4294901760;

    localparam signed [33:0] TARGET_MIN =
        -34'sd4294967296;

    reg [2:0] state;
    reg sample_d;

    reg signed [32:0] anchor_q16;

    reg start_pending;
    reg stop_pending;

    reg [COUNT_BITS-1:0] cycle_tick;
    reg [COUNT_BITS-1:0] envelope_tick;

    assign busy =
        enable && (start_pending || state != IDLE);

    assign motion_active =
        enable && !load_target &&
        (start_pending || state != IDLE);

    assign stop_pending_mon = stop_pending;
    assign state_mon = state;

    wire start_angle_ok =
        ($signed(theta) >= -START_THETA_ADC) &&
        ($signed(theta) <=  START_THETA_ADC);

    wire start_speed_ok =
        ($signed(actual_speed) >= -START_SPEED) &&
        ($signed(actual_speed) <=  START_SPEED);

    wire signed [33:0] anchor_ext = {anchor_q16[32],anchor_q16};

    // 在开始动作前检查整个左右摆幅，避免运动中目标溢出。
    wire trajectory_in_range =
        (anchor_ext + AMPLITUDE_Q16 <= TARGET_MAX) &&
        (anchor_ext - AMPLITUDE_Q16 >= TARGET_MIN);

    // --------------------------------------------------------
    // 四分之一正弦查找表，数值范围0~32768。
    // 对应0、pi/32、2*pi/32……pi/2。
    // --------------------------------------------------------
    function [15:0] quarter_lut;
        input [4:0] index;
        begin
            case(index)
                5'd0:  quarter_lut = 16'd0;
                5'd1:  quarter_lut = 16'd3212;
                5'd2:  quarter_lut = 16'd6393;
                5'd3:  quarter_lut = 16'd9512;
                5'd4:  quarter_lut = 16'd12540;
                5'd5:  quarter_lut = 16'd15447;
                5'd6:  quarter_lut = 16'd18205;
                5'd7:  quarter_lut = 16'd20788;
                5'd8:  quarter_lut = 16'd23170;
                5'd9:  quarter_lut = 16'd25330;
                5'd10: quarter_lut = 16'd27246;
                5'd11: quarter_lut = 16'd28899;
                5'd12: quarter_lut = 16'd30274;
                5'd13: quarter_lut = 16'd31357;
                5'd14: quarter_lut = 16'd32138;
                5'd15: quarter_lut = 16'd32610;
                default: quarter_lut = 16'd32768;
            endcase
        end
    endfunction

    // --------------------------------------------------------
    // 1024相位点的完整正弦。
    //
    // phase=0    -> 0
    // phase=256  -> +32768
    // phase=512  -> 0
    // phase=768  -> -32768
    //
    // 输出17位有符号，实际值=输出/32768。
    // --------------------------------------------------------
    // 第一级：查出相邻表值，锁存符号及4位插值系数。
    function [36:0] sine_lookup;
        input [9:0] phase;
        reg [8:0] coordinate;
        reg [4:0] index;
        begin
            coordinate=phase[8] ? 9'd256-{1'b0,phase[7:0]} : {1'b0,phase[7:0]};
            index=coordinate[8:4];
            sine_lookup={phase[9],coordinate[3:0],quarter_lut(index),quarter_lut(index+5'd1)};
        end
    endfunction

    // 第二级：移位相加实现线性插值，保持与原表完全相同的取整。
    function signed [16:0] sine_interpolate;
        input [36:0] lookup;
        reg [3:0] fraction;
        reg [15:0] value0,difference;
        reg [19:0] interpolation;
        reg [16:0] magnitude;
        begin
            fraction=lookup[35:32];
            value0=lookup[31:16];
            difference=lookup[15:0]-value0;
            interpolation=(fraction[0] ? {4'b0,difference} : 20'd0)+
                          (fraction[1] ? {3'b0,difference,1'b0} : 20'd0)+
                          (fraction[2] ? {2'b0,difference,2'b0} : 20'd0)+
                          (fraction[3] ? {1'b0,difference,3'b0} : 20'd0);
            magnitude={1'b0,value0}+((interpolation+20'd8)>>4);
            sine_interpolate=lookup[36] ? -$signed(magnitude) : $signed(magnitude);
        end
    endfunction

    // --------------------------------------------------------
    // 计算流水线：
    // 1. 锁存相位；
    // 2. 查表并锁存相邻表值；
    // 3. 插值并计算幅值包络；
    // 4. 幅值乘正弦；
    // 5. 乘包络；
    // 6. 输出位置目标。
    //
    // 全部由50MHz主时钟驱动。
    // --------------------------------------------------------
    reg [5:0] valid_pipe;

    reg [9:0] phase_reg;
    reg [9:0] envelope_phase_reg;

    reg signed [16:0] sine_reg;
    reg [36:0] sine_lookup_reg,envelope_lookup_reg;
    wire signed [16:0] cosine_interpolated=sine_interpolate(envelope_lookup_reg);

    reg [16:0] envelope_q16_reg;
    reg [16:0] envelope_aligned_reg;

    reg signed [DISTANCE_BITS-1:0] sine_distance_reg;
    reg signed [DISTANCE_BITS+17:0] enveloped_distance_reg;

    wire [COUNT_BITS+9:0] cycle_numerator={cycle_tick,10'b0};
    wire [COUNT_BITS+9:0] cycle_quotient=cycle_numerator / SAFE_PERIOD;
    wire [9:0] cycle_phase_calc=cycle_quotient[9:0];

    // 渐入/渐出进度映射到0~pi，对应相位0~512。
    wire [COUNT_BITS+8:0] envelope_numerator={envelope_tick,9'b0};
    wire [COUNT_BITS+8:0] envelope_quotient=envelope_numerator / SAFE_PERIOD;
    wire [9:0] envelope_phase_calc=envelope_quotient[9:0];

    wire [9:0] envelope_cos_phase =
        envelope_phase_reg + 10'd256;

    // 1-cos(pi*u)：从0平滑增加到2。
    wire signed [18:0] fade_in_q16 =
        19'sd32768 -
        $signed({{2{cosine_interpolated[16]}},
                 cosine_interpolated});

    // 1+cos(pi*u)：从2平滑减小到0。
    wire signed [18:0] fade_out_q16 =
        19'sd32768 +
        $signed({{2{cosine_interpolated[16]}},
                 cosine_interpolated});

    reg [2:0] state_d0;
    reg [2:0] state_d1;

    wire signed [16:0] sine_extended =
        $signed(sine_reg);
    // 先以完整乘积位宽计算，再右移；不能直接用窄寄存器作为乘法上下文。
    wire signed [DISTANCE_BITS+16:0] amplitude_product=AMPLITUDE_Q16*sine_extended;
    wire signed [DISTANCE_BITS+16:0] amplitude_scaled=amplitude_product >>> 15;

    wire signed [17:0] envelope_extended =
        $signed({1'b0, envelope_aligned_reg});

    wire signed [33:0] final_offset_q16 =
        enveloped_distance_reg >>> 16;

    wire signed [33:0] final_target_q16 /* synthesis syn_dspstyle="logic" */;
    assign final_target_q16 = anchor_ext + final_offset_q16;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state <= IDLE;
            sample_d <= 0;
            anchor_q16 <= 0;

            start_pending <= 0;
            stop_pending <= 0;

            cycle_tick <= 0;
            envelope_tick <= 0;

            target_q16 <= 0;
            target_valid <= 0;

            arrived <= 0;
            range_rejected <= 0;
            request_rejected <= 0;

            valid_pipe <= 0;
            phase_reg <= 0;
            envelope_phase_reg <= 0;

            sine_reg <= 0;
            sine_lookup_reg <= 0; envelope_lookup_reg <= 0;
            envelope_q16_reg <= 0;
            envelope_aligned_reg <= 0;

            sine_distance_reg <= 0;
            enveloped_distance_reg <= 0;

            state_d0 <= IDLE;
            state_d1 <= IDLE;
        end
        else if(load_target) begin
            state <= IDLE;
            sample_d <= 0;

            anchor_q16 <=
                $signed({load_position, 16'b0});
            target_q16 <=
                $signed({load_position, 16'b0});

            start_pending <= 0;
            stop_pending <= 0;
            cycle_tick <= 0;
            envelope_tick <= 0;

            target_valid <= 1;
            arrived <= 0;
            range_rejected <= 0;
            request_rejected <= 0;

            valid_pipe <= 0;
            phase_reg <= 0;
            envelope_phase_reg <= 0;
            sine_reg <= 0;
            sine_lookup_reg <= 0; envelope_lookup_reg <= 0;
            envelope_q16_reg <= 0;
            envelope_aligned_reg <= 0;
            sine_distance_reg <= 0;
            enveloped_distance_reg <= 0;
            state_d0 <= IDLE;
            state_d1 <= IDLE;
        end
        else if(!enable) begin
            state <= IDLE;
            sample_d <= 0;
            anchor_q16 <= 0;
            target_q16 <= 0;

            start_pending <= 0;
            stop_pending <= 0;

            cycle_tick <= 0;
            envelope_tick <= 0;

            target_valid <= 0;
            arrived <= 0;
            range_rejected <= 0;
            request_rejected <= 0;

            valid_pipe <= 0;
            phase_reg <= 0;
            envelope_phase_reg <= 0;
            sine_reg <= 0;
            sine_lookup_reg <= 0; envelope_lookup_reg <= 0;
            envelope_q16_reg <= 0;
            envelope_aligned_reg <= 0;
            sine_distance_reg <= 0;
            enveloped_distance_reg <= 0;
            state_d0 <= IDLE;
            state_d1 <= IDLE;
        end
        else if(IMMEDIATE_STOP && start_event && (state != IDLE || start_pending)) begin
            // 取消所有尚未输出的正弦流水线，防止旧轨迹覆盖原目标。
            state <= IDLE;
            sample_d <= 0;
            start_pending <= 0;
            stop_pending <= 0;
            cycle_tick <= 0;
            envelope_tick <= 0;
            valid_pipe <= 0;
            target_q16 <= anchor_q16;
            target_valid <= 1;
            arrived <= 1;
        end
        else begin
            // 延迟一拍读取编码器更新后的speed。
            sample_d <= sample_tick;

            target_valid <= 0;
            valid_pipe <= {valid_pipe[4:0], 1'b0};

            // KEY3：空闲时启动，运行时请求平滑停止。
            if(start_event) begin
                if(state == IDLE && !start_pending) begin
                    start_pending <= 1;
                    request_rejected <= 0;
                    arrived <= 0;
                    if(USE_START_ANCHOR) begin
                        anchor_q16 <= start_anchor_q16;
                        target_q16 <= start_anchor_q16;
                        target_valid <= 1;
                    end
                end
                else if(state == FADE_IN || state == RUN) begin
                    stop_pending <= 1;
                end
                // 渐出期间重复按键忽略。
            end

            // 查表阶段。
            if(valid_pipe[0]) begin
                sine_lookup_reg <= sine_lookup(phase_reg);
                envelope_lookup_reg <= sine_lookup(envelope_cos_phase);
                state_d1 <= state_d0;
            end

            // 包络计算阶段。
            if(valid_pipe[1]) begin
                sine_reg <= sine_interpolate(sine_lookup_reg);
                if(state_d1 == FADE_IN)
                    envelope_q16_reg <= fade_in_q16[16:0];
                else if(state_d1 == FADE_OUT)
                    envelope_q16_reg <= fade_out_q16[16:0];
                else
                    envelope_q16_reg <= 17'd65536;
            end

            // 正弦乘幅值。
            if(valid_pipe[2]) begin
                sine_distance_reg <= amplitude_scaled[DISTANCE_BITS-1:0];

                envelope_aligned_reg <= envelope_q16_reg;
            end

            // 乘启停幅值包络。
            if(valid_pipe[3]) begin
                enveloped_distance_reg <=
                    sine_distance_reg * envelope_extended;
            end

            // 输出位置目标。
            if(valid_pipe[4]) begin
                if(final_target_q16 > TARGET_MAX)
                    target_q16 <= TARGET_MAX[32:0];
                else if(final_target_q16 < TARGET_MIN)
                    target_q16 <= TARGET_MIN[32:0];
                else
                    target_q16 <= final_target_q16[32:0];

                target_valid <= 1;
            end

            if(sample_d) begin
                if(state == IDLE) begin
                    if(start_pending) begin
                        start_pending <= 0;

                        if(!start_angle_ok ||
                           !start_speed_ok) begin
                            request_rejected <= 1;
                        end
                        else if(!trajectory_in_range) begin
                            range_rejected <= 1;
                            request_rejected <= 1;
                        end
                        else begin
                            state <= FADE_IN;
                            cycle_tick <= 0;
                            envelope_tick <= 0;

                            stop_pending <= 0;
                            arrived <= 0;
                            range_rejected <= 0;
                            request_rejected <= 0;
                        end
                    end
                end
                else begin
                    // 本次10ms采样的相位和状态。
                    phase_reg <= cycle_phase_calc[9:0];
                    envelope_phase_reg <=
                        envelope_phase_calc[9:0];
                    state_d0 <= state;

                    valid_pipe[0] <= 1;

                    if(cycle_tick >= SAFE_PERIOD - 1)
                        cycle_tick <= 0;
                    else
                        cycle_tick <= cycle_tick + 1'b1;

                    case(state)
                        FADE_IN: begin
                            if(envelope_tick >= SAFE_PERIOD) begin
                                state <= RUN;
                                envelope_tick <= 0;
                            end
                            else
                                envelope_tick <=
                                    envelope_tick + 1'b1;
                        end

                        RUN: begin
                            envelope_tick <= 0;

                            // 从周期边界开始渐出，
                            // 使停止过程与完整正弦周期对齐。
                            if(stop_pending &&
                               cycle_tick == 0) begin
                                state <= FADE_OUT;
                                envelope_tick <= 1;
                            end
                        end

                        FADE_OUT: begin
                            if(envelope_tick >= SAFE_PERIOD) begin
                                state <= IDLE;
                                stop_pending <= 0;
                                envelope_tick <= 0;
                                cycle_tick <= 0;
                                arrived <= 1;

                                // 丢弃后续流水线结果，
                                // 最后精确保持运动中心。
                                valid_pipe <= 0;
                                target_q16 <= anchor_q16;
                                target_valid <= 1;
                            end
                            else
                                envelope_tick <=
                                    envelope_tick + 1'b1;
                        end

                        default: begin
                            state <= IDLE;
                            valid_pipe <= 0;
                        end
                    endcase
                end
            end
        end
    end

endmodule
