// 低电平按键：双触发器同步；按下、松开都连续稳定后才接受。
// 上电必须先确认松开；长按只产生一次事件。
module key_press #(
 parameter integer DEBOUNCE_CYCLES=1000000
)(input wire clk, rst_n, key_n, output reg pressed);
 reg meta=1'b1, sync=1'b1;
 reg stable=1'b1, armed=1'b0;
 reg [31:0] count=0, release_count=0;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   meta<=1; sync<=1; stable<=1; armed<=0; count<=0; release_count<=0; pressed<=0;
  end else begin
   meta<=key_n; sync<=meta; pressed<=0;
   if(sync==stable) count<=0;
   else if(count==DEBOUNCE_CYCLES-1) begin
    stable<=sync; count<=0;
    if(!sync && armed) begin pressed<=1; armed<=0; end
   end else count<=count+1'b1;
   if(sync) begin
    if(release_count<DEBOUNCE_CYCLES) release_count<=release_count+1'b1;
    if(release_count==DEBOUNCE_CYCLES-1) armed<=1;
   end else release_count<=0;
  end
 end
endmodule

// KEY4: short event on debounced release, long event once after 1 second.
// Cancellation consumes the whole press; release is required to rearm.
module key_short_long #(
 parameter integer DEBOUNCE_CYCLES=1000000,
 parameter integer LONG_CYCLES=50000000
)(input wire clk,rst_n,key_n,cancel,
 output reg short_pressed,long_pressed);
 reg meta=1,sync=1,stable=1,armed=0,held=0,fired=0;
 reg [31:0] debounce_count=0,release_count=0,hold_count=0;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   meta<=1;sync<=1;stable<=1;armed<=0;held<=0;fired<=0;
   debounce_count<=0;release_count<=0;hold_count<=0;
   short_pressed<=0;long_pressed<=0;
  end else begin
   meta<=key_n;sync<=meta;short_pressed<=0;long_pressed<=0;
   if(sync==stable) debounce_count<=0;
   else if(debounce_count==DEBOUNCE_CYCLES-1) begin
    debounce_count<=0;stable<=sync;
    if(!sync) begin
     if(armed && !cancel) begin held<=1;fired<=0;hold_count<=0;end
     armed<=0;
    end else begin
     if(held && !fired && !cancel) short_pressed<=1;
     held<=0;hold_count<=0;
    end
   end else debounce_count<=debounce_count+1'b1;
   if(sync) begin
    if(release_count<DEBOUNCE_CYCLES) release_count<=release_count+1'b1;
    if(release_count==DEBOUNCE_CYCLES-1) armed<=1;
   end else release_count<=0;
   if(held && !sync && !fired) begin
    if(hold_count==LONG_CYCLES-1) begin long_pressed<=1;fired<=1;end
    else hold_count<=hold_count+1'b1;
   end
   if(cancel) begin held<=0;hold_count<=0;short_pressed<=0;long_pressed<=0;end
  end
 end
endmodule
