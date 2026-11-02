// 计算与失稳前目标物理角度相同、且距离当前位置最近的累计编码器目标。
// Q16精度；逐位取余，避免在组合逻辑中放置大位宽除法器。
module shortest_recovery #(
 parameter integer COUNTS_PER_REV=1080
)(
 input wire clk, rst_n, enable,
 input wire signed [32:0] saved_target_q16,
 input wire signed [16:0] position,
 output reg signed [32:0] result_q16,
 output reg ready
);
 localparam [33:0] TURN_Q16=COUNTS_PER_REV*34'd65536;
 localparam [33:0] HALF_Q16=TURN_Q16 >> 1;
 localparam signed [34:0] TARGET_MAX=35'sd4294967295;
 localparam signed [34:0] TARGET_MIN=-35'sd4294967296;
 reg [1:0] state;
 reg [5:0] remaining;
 reg [33:0] dividend;
 reg [33:0] remainder;
 reg negative;
 reg signed [32:0] capture_q16;
 wire signed [33:0] delta=$signed({saved_target_q16[32],saved_target_q16})-
                          $signed({position[16],position,16'b0});
 wire [34:0] shifted={remainder,dividend[33]};
 wire [33:0] next_remainder=(shifted >= {1'b0,TURN_Q16}) ? shifted-{1'b0,TURN_Q16} : shifted;
 wire signed [34:0] signed_remainder=negative ? -$signed({1'b0,remainder}) : $signed({1'b0,remainder});
 // ±180°等距时保留原差值方向。
 wire signed [34:0] shortest=(signed_remainder > $signed({1'b0,HALF_Q16})) ? signed_remainder-$signed({1'b0,TURN_Q16}) :
                               (signed_remainder < -$signed({1'b0,HALF_Q16})) ? signed_remainder+$signed({1'b0,TURN_Q16}) : signed_remainder;
 wire signed [34:0] candidate=$signed({{2{capture_q16[32]}},capture_q16})+shortest;
 // 累计计数边缘若最近的等效目标超出17位范围，选择范围内的等效圈。
 wire signed [34:0] bounded=(candidate > TARGET_MAX) ? candidate-$signed({1'b0,TURN_Q16}) :
                            (candidate < TARGET_MIN) ? candidate+$signed({1'b0,TURN_Q16}) : candidate;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin state<=0; remaining<=0; dividend<=0; remainder<=0; negative<=0; capture_q16<=0; result_q16<=0; ready<=0; end
  else if(!enable) begin state<=0; ready<=0; end
  else case(state)
   0: begin
    capture_q16<={position,16'b0}; negative<=delta[33];
    dividend<=delta[33] ? -delta : delta; remainder<=0; remaining<=34; state<=1; ready<=0;
   end
   1: begin
    remainder<=next_remainder; dividend<=dividend<<1;
    if(remaining==1) begin remaining<=0; state<=2; end
    else remaining<=remaining-1'b1;
   end
   2: begin result_q16<=bounded[32:0]; ready<=1; state<=3; end
   default: begin end
  endcase
 end
endmodule
