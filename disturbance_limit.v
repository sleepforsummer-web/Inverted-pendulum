// 外环限幅包络：起摆/手动接管沿用旧双条件；后续偏离触发一次平滑回落。
// 这里只设置允许的目标角上限，不直接产生制动力，也不改起摆PWM。
module disturbance_limit #(
    parameter signed [16:0] NEAR_ADC = 17'sd40,
    parameter signed [16:0] DISTURB_ADC = 17'sd200,
    parameter integer RAMP_MS = 1000,
    parameter integer SAMPLE_MS = 10,
    parameter integer STARTUP_MIN_MS = 1000
)(
    input wire clk, rst, enable, sample_tick, motion_active,
    input wire position_outside,
    input wire signed [16:0] startup_limit,
    input wire startup_boost,
    output reg signed [16:0] sample_limit,
    output reg sample_boost,
    // 仅诊断：1起摆收敛、2等待受扰、3平滑回落、4等待回区、5运动覆盖。
    output wire [2:0] sample_phase
);
    localparam [1:0] STARTUP=2'd0, ARMED=2'd1, DECAY=2'd2, WAIT_INSIDE=2'd3;
    localparam integer RAMP_TICKS_RAW=(RAMP_MS+SAMPLE_MS-1)/SAMPLE_MS;
    localparam integer RAMP_TICKS=(RAMP_TICKS_RAW>0) ? RAMP_TICKS_RAW : 1;
    localparam integer START_TICKS_RAW=(STARTUP_MIN_MS+SAMPLE_MS-1)/SAMPLE_MS;
    localparam integer START_TICKS=(START_TICKS_RAW>0) ? START_TICKS_RAW : 1;
    localparam integer RAMP_BITS=(RAMP_TICKS>1) ? $clog2(RAMP_TICKS+1) : 1;
    localparam integer START_BITS=(START_TICKS>1) ? $clog2(START_TICKS+1) : 1;
    localparam integer SPAN=(DISTURB_ADC>NEAR_ADC) ? DISTURB_ADC-NEAR_ADC : 0;
    localparam signed [16:0] PEAK=(DISTURB_ADC>NEAR_ADC) ? DISTURB_ADC : NEAR_ADC;
    // 常数除法只在编译时求值；运行时只用计数、加减和比较，无乘除法器。
    localparam integer BASE_STEP=SPAN/RAMP_TICKS;
    localparam integer STEP_REMAINDER=SPAN%RAMP_TICKS;
    localparam signed [17:0] BASE_STEP_SIZED=BASE_STEP[17:0];
    localparam [RAMP_BITS:0] REMAINDER_SIZED=STEP_REMAINDER[RAMP_BITS:0];
    localparam [RAMP_BITS:0] TICKS_SIZED=RAMP_TICKS[RAMP_BITS:0];
    reg [1:0] phase, phase_next;
    reg [START_BITS-1:0] startup_ticks, startup_next;
    reg [RAMP_BITS-1:0] elapsed, elapsed_next, residue, residue_next;
    reg signed [16:0] cap, cap_next;
    assign sample_phase=!enable ? 3'd0 : motion_active ? 3'd5 : ({1'b0,phase_next}+3'd1);
    wire [RAMP_BITS:0] remainder_sum={1'b0,residue}+REMAINDER_SIZED;
    wire extra_step=(remainder_sum>=TICKS_SIZED);
    wire [RAMP_BITS:0] residue_full=extra_step ? remainder_sum-TICKS_SIZED : remainder_sum;
    wire signed [17:0] next_cap=$signed({cap[16],cap})-BASE_STEP_SIZED-(extra_step ? 18'sd1 : 18'sd0);
    always @* begin
        phase_next=phase; startup_next=startup_ticks;
        elapsed_next=elapsed; residue_next=residue; cap_next=cap;
        sample_limit=NEAR_ADC; sample_boost=1'b0;
        if(!enable || motion_active) begin
            phase_next=STARTUP; startup_next=0;
            elapsed_next=0; residue_next=0; cap_next=NEAR_ADC;
        end else begin
            case(phase)
                STARTUP: begin
                    // 限幅值和旧双条件完全一致，不对110做平滑处理。
                    sample_limit=startup_limit; sample_boost=startup_boost;
                    if(startup_ticks<START_TICKS) startup_next=startup_ticks+1'b1;
                    if(startup_ticks>=START_TICKS-1 && !startup_boost && !position_outside)
                        phase_next=ARMED;
                end
                ARMED: begin
                    if(position_outside) begin
                        // 第一个触发采样为t=0，从200开始，不保持200一秒再下降。
                        phase_next=DECAY; elapsed_next=0; residue_next=0; cap_next=PEAK;
                        sample_limit=PEAK; sample_boost=(PEAK>NEAR_ADC);
                    end
                end
                DECAY: begin
                    if(elapsed>=RAMP_TICKS-1) begin
                        cap_next=NEAR_ADC; elapsed_next=0; residue_next=0;
                        phase_next=position_outside ? WAIT_INSIDE : ARMED;
                    end else begin
                        elapsed_next=elapsed+1'b1;
                        residue_next=residue_full[RAMP_BITS-1:0];
                        cap_next=(next_cap<NEAR_ADC) ? NEAR_ADC : next_cap[16:0];
                    end
                    sample_limit=cap_next; sample_boost=(cap_next>NEAR_ADC);
                    // 持续偏离/误差变号/提前回区内均不重置本次回落。
                end
                WAIT_INSIDE: begin
                    // 超时仍偏离时维持40，回到阈值内才允许下一次触发。
                    if(!position_outside) phase_next=ARMED;
                end
                default: begin
                    phase_next=STARTUP; startup_next=0;
                    elapsed_next=0; residue_next=0; cap_next=NEAR_ADC;
                end
            endcase
        end
    end
    always @(posedge clk) begin
        if(rst || !enable || motion_active) begin
            phase<=STARTUP; startup_ticks<=0; elapsed<=0; residue<=0; cap<=NEAR_ADC;
        end else if(sample_tick) begin
            phase<=phase_next; startup_ticks<=startup_next;
            elapsed<=elapsed_next; residue<=residue_next; cap<=cap_next;
        end
    end
endmodule
