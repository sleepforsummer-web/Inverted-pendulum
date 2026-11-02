//========================================================================
// adc_v2.v  —— 3PA1030 角度读取模块（改进版，替代官方 adc.v）
//
// 改进点（相对官方 adc.v）：
//   1. 采样率大幅提升：官方每 3ms 才输出一次（187.5us x 16 块平均），
//      本模块每 1us 采样一次、32 点块平均后约每 32us 输出一次（31.25kHz），
//      控制环不再背 3ms 延迟。
//   2. 块平均后再加一级 IIR 低通（y += (x-y)>>IIR_SHIFT），
//      平滑度/延迟可通过 IIR_SHIFT 在 1~5 之间折中。
//   3. 内置标定：OFFSET（竖直向上的原始读数）、INVERT（方向取反），
//      输出 theta 以竖直向上为 0 的有符号角度量。
//   4. 输出 data_valid 脉冲，方便与控制环同步。
//   5. ADC时钟12.5MHz，上升沿后约60ns锁存整组10bit总线。
//      平均器只读取锁存数据；10bit仍左移2位作为12bit ADC计数。
//
// 标定方法见 angle_logger.py 的 calib 模式。
//========================================================================
module adc_v2 #(
    parameter AVG_LOG2 = 5,          // 块平均点数 = 2^AVG_LOG2 = 32
    parameter IIR_SHIFT = 2,         // IIR 强度: 1最弱(最快) ~ 5最强(最平滑)
    parameter OFFSET  = 12'd2048,    // ★标定: 摆杆竖直向上时的 theta_raw，实例化后被覆盖
    parameter INVERT  = 0            // ★标定: 摆杆向右摆 theta 应增大, 否则置1
)(
    input  wire               clk_50m,
    input  wire               rst_n,
    // 3PA1030 硬件接口（与官方一致）
    output wire               adc_clk,      // 12.5MHz转换时钟
    output wire               adc_oe_n,     // 输出使能，低有效
    output wire               adc_stby_n,   // 3PA1030 STBY高电平待机（保留端口名）
    input  wire [9:0]         adc_data_in,  // D9~D0 并行数据
    // 输出
    output reg  [11:0]        theta_raw,    // 32点块平均值，尚未经过IIR
    output wire [11:0]        theta_filtered, // 与theta对应的IIR输出
    output reg  signed [12:0] theta,        // 标定后的有符号ADC计数：竖直向上为0，不是度数
    output reg                data_valid,   // 每输出一个新数据打一拍
    output reg [11:0]         block_min, block_max,
    output reg [17:0]         block_sum,
    output reg [7:0]          block_count   // 与theta_raw/theta同一组平均采样
);

localparam AVG_NUM = (1 << AVG_LOG2);

//---------------- ADC时钟与并行数据锁存 ----------------
// 相位0->1->2->3->0，每相位20ns；0->1时读取外部总线。
// 此时距上一次1->2的ADC上升沿60ns，距下一上升沿20ns。
// 必须按输出数据有效时间锁存整组总线，而非逐位异步同步。
reg [1:0] adc_phase;
reg [9:0] adc_sample_reg;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin adc_phase <= 2'd0; adc_sample_reg <= 10'd0; end
    else begin
        adc_phase <= adc_phase + 1'b1;
        if(adc_phase==2'd0) adc_sample_reg <= adc_data_in;
    end
end
assign adc_clk     = adc_phase[1];
assign adc_oe_n    = 1'b0;
assign adc_stby_n  = 1'b0;

wire [11:0] theta_12bit = {adc_sample_reg, 2'b00}; // 仅使用已经锁存的整组数据

//---------------- 1us 采样节拍 (50 x 20ns) ----------------
reg [5:0] div_cnt;
wire sample_tick = (div_cnt == 6'd49);
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n)            div_cnt <= 6'd0;
    else if(sample_tick)  div_cnt <= 6'd0;
    else                  div_cnt <= div_cnt + 1'b1;
end

//---------------- 块平均 (滑动窗口平均的简化版, 无除法器) ----------------
reg [17:0] acc;                 // 最大 32*4092 = 130944 < 2^17
reg [AVG_LOG2-1:0] scnt;
wire [17:0] acc_next = acc + {6'b0, theta_12bit};
wire avg_done = sample_tick && (scnt == AVG_NUM-1);
reg [11:0] min_work, max_work;
wire [11:0] min_next = (scnt==0 || theta_12bit<min_work) ? theta_12bit : min_work;
wire [11:0] max_next = (scnt==0 || theta_12bit>max_work) ? theta_12bit : max_work;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        acc <= 18'd0; scnt <= {AVG_LOG2{1'b0}};
        min_work <= 12'd0; max_work <= 12'd0;
    end else if(sample_tick) begin
        min_work <= min_next; max_work <= max_next;
        if(avg_done) begin acc <= 18'd0; scnt <= {AVG_LOG2{1'b0}}; end
        else         begin acc <= acc_next; scnt <= scnt + 1'b1; end
    end
end

wire [11:0] raw_next = acc_next[AVG_LOG2 +: 12]; // 显式选择平均结果12位

//---------------- IIR 低通: Q8状态保留小数，输出四舍五入 ----------------
// 高12位为ADC整数，低8位为小数；递推不再丢弃整个ADC计数的小变化。
// 系数仍为1/(2^IIR_SHIFT)，更新节拍仍为avg_done。
reg  [19:0] iir_q8;
reg  [11:0] iir_reg;
assign theta_filtered = iir_reg;
wire signed [20:0] raw_q8 = $signed({1'b0, raw_next, 8'b0});
wire signed [20:0] state_q8 = $signed({1'b0, iir_q8});
wire signed [20:0] delta_q8 = raw_q8 - state_q8;
wire signed [20:0] next_q8 = state_q8 + (delta_q8 >>> IIR_SHIFT);
wire [20:0] rounded_q8 = $unsigned(next_q8) + 21'd128;
wire [11:0] filtered_next = rounded_q8[19:8];

//---------------- 标定: theta = (±)(滤波值 - OFFSET) ----------------
wire signed [12:0] filt_ext = $signed({1'b0, filtered_next});
wire signed [12:0] theta_next = INVERT ?
      ($signed({1'b0, OFFSET}) - filt_ext) : (filt_ext - $signed({1'b0, OFFSET}));

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        theta_raw <= 12'd0;
        iir_q8    <= {OFFSET[11:0], 8'b0};
        iir_reg   <= OFFSET;
        theta     <= 13'sd0;
        data_valid<= 1'b0;
        block_min <= 0; block_max <= 0; block_sum <= 0; block_count <= 0;
    end else if(avg_done) begin
        theta_raw <= raw_next;
        iir_q8    <= next_q8[19:0];
        iir_reg   <= filtered_next;
        theta     <= theta_next;
        data_valid<= 1'b1;
        block_min <= min_next; block_max <= max_next;
        block_sum <= acc_next; block_count <= AVG_NUM;
    end else begin
        data_valid<= 1'b0;
    end
end

endmodule
