`timescale 1ns / 1ps

module encoder_decoder (
    input wire                   clk,
    input wire                   rst_n,
    input wire                   en, // 速度采样节拍，顶层每10ms产生一个主时钟周期的脉冲

    input wire                   a,
    input wire                   b,

    output reg signed [16:0]     position, // 从复位起累计的位置计数，不按每圈自动归零
    output reg signed [15:0]     speed // 相邻速度采样节拍之间的位置增量，单位为计数/10ms
);

    // 两级同步
    reg a_sync1;
    reg a_sync2;
    reg b_sync1;
    reg b_sync2;

    // 上一个编码器状态
    reg a_prev;
    reg b_prev;

    // 上一个采样周期的位置
    reg signed [16:0] position_last;

    // 当前周期的位置变化量
    wire signed [17:0] speed_diff;

    assign speed_diff =
        $signed(position) - $signed(position_last);


    // =========================================================
    // 编码器输入同步
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync1 <= 1'b0;
            a_sync2 <= 1'b0;
            b_sync1 <= 1'b0;
            b_sync2 <= 1'b0;
        end
        else begin
            a_sync1 <= a;
            a_sync2 <= a_sync1;

            b_sync1 <= b;
            b_sync2 <= b_sync1;
        end
    end


    // =========================================================
    // 正交编码器解码 + 累计位置
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_prev   <= 1'b0;
            b_prev   <= 1'b0;
            position <= 17'sd0;
        end
        else begin
            a_prev <= a_sync2;
            b_prev <= b_sync2;

            case ({a_prev, b_prev, a_sync2, b_sync2})

                // 正方向
                4'b00_10,
                4'b10_11,
                4'b11_01,
                4'b01_00:
                    position <= position + 17'sd1;

                // 反方向
                4'b00_01,
                4'b01_11,
                4'b11_10,
                4'b10_00:
                    position <= position - 17'sd1;

                // 非法跳变或无变化
                default:
                    position <= position;
            endcase
        end
    end


    // =========================================================
    // 速度计算
    // en 应该每10ms产生一个单周期脉冲
    // speed = 当前累计位置 - 上一次累计位置
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            position_last <= 17'sd0;
            speed         <= 16'sd0;
        end
        else if (en) begin

            if (speed_diff > 18'sd32767)
                speed <= 16'sd32767;

            else if (speed_diff < -18'sd32768)
                speed <= 16'b1000000000000000;

            else
                speed <= speed_diff[15:0];

            position_last <= position;
        end
    end

endmodule
