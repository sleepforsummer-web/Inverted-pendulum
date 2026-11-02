"""验证固定系数版串口、历史查看和二维模型；不连接串口。"""
from pathlib import Path
import math
import time
import tkinter as tk
from unittest.mock import patch
import db_pid_scope as scope

root=Path(__file__).resolve().parent
capture=bytes(int(v,16) for v in (root/'sim/uart_outer_i.hex').read_text().split())
assert len(capture)==152
parser=scope.Parser();rows=[]
for k in range(0,len(capture),7): rows.extend(parser.feed(capture[k:k+7]))
assert len(rows)==2 and parser.checksum_errors==0
assert len(scope.CH)==len(scope.COLORS)==len(rows[0])==30
assert sorted(scope.DISPLAY_ORDER)==list(range(30))
assert rows[0][23:]==(1.5,200,3,268.25,50.5,17,1)
assert rows[1][23:]==(-2.5,40,4,-536.5,-10,18,7)
assert capture[60:63]==bytes((0,16,12))
bad=bytearray(capture[:76]);bad[25]^=1
parser=scope.Parser();assert len(parser.feed(b'noise'+bad+capture[76:]))==1
assert parser.checksum_errors==1
for kind,length in scope.FRAME_LENGTHS.items():
    if kind in (0x55,0x5E):continue
    body=capture[2:2+length-3];check=0
    for b in body:check^=b
    row=scope.Parser().feed(bytes((0xAA,kind))+body+bytes((check,)))[0]
    assert len(row)==30 and all(math.isnan(v) for v in row[26:])
    if kind!=0x5D:assert all(math.isnan(v) for v in row[24:26])
assert len(scope.Parser().feed(b'\xaa\x55'+bytes(15))[0])==30
original_init=tk.Tk.__init__
def hidden_init(self,*args,**kwargs):
    original_init(self,*args,**kwargs);self.withdraw()
with patch.object(tk.Tk,'__init__',hidden_init), \
     patch.object(scope.serial,'Serial',side_effect=AssertionError('No serial access')), \
     patch.object(scope.serial.tools.list_ports,'comports',return_value=[]):
    app=scope.App()
    try:
        assert set(app.vals)==set(scope.CH) and len(app.axes)==6
        assert app.cur['内环P'][0].axes is app.cur['内环D'][0].axes
        assert '内环I' not in app.cur
        assert len({app.cur[name][0].axes for name in ('外环P','外环I','外环D')})==1
        assert len(app.canvases)==3 and app.model.compact
        assert app.model.calibration.encoder_counts_per_turn==1080
        app.samples.extend([(1.,*rows[0]),(1.1,*rows[1])]);app.after=lambda *a:None
        app.update_ui()
        assert app.vals['实时限幅ADC'].cget('text')=='40'
        assert len(app.cur['外环P'][0].get_ydata())==2
        assert app.model.state is not None
        app.pause_view();app.hover_axis=app.cur['外环P'][0].axes;app.hover_x=1.
        app.refresh_hover(redraw=False)
        assert app.vals['实时限幅ADC'].cget('text')=='200'
        assert app.vals['限幅阶段'].cget('text')=='受扰回落'
        # 暂停历史只影响显示；控制确认始终取最新数据。测试替身不接触串口硬件。
        class FakeSerial:
            port='TEST_ONLY'
            def __init__(self):self.written=[]
            def write(self,data):self.written.append(data);return len(data)
            def close(self):pass
        from serial_commands import encode_position_command
        import struct
        fast=encode_position_command(-180,360,19)
        assert struct.unpack('<BBiH',fast[2:-1])==(1,19,-138240,2765)
        assert struct.unpack('<BBiH',encode_position_command(360,360,20)[2:-1])==(1,20,276480,2765)
        try:encode_position_command(90,361,20)
        except ValueError:pass
        else:raise AssertionError('speed above360 accepted')
        fake=FakeSerial();app.ser=fake
        live=list(rows[0]);live[12]=1
        app.latest=tuple(live);app.latest_received=time.monotonic()
        app.send_position_target()
        assert len(fake.written)==1 and fake.written[0]==encode_position_command(90,45,1)
        assert app.pending_command[0]==1
        live[28]=1;live[29]=1;app.latest=tuple(live)
        app.update_command_status();assert app.pending_command is None
        assert '已接受' in app.command_note.cget('text')
        live[29]=7;app.latest=tuple(live);app.update_command_status()
        assert '轨迹完成' in app.command_note.cget('text')
        app.target_speed.delete(0,tk.END);app.target_speed.insert(0,'0')
        app.send_position_target();assert len(fake.written)==1
        app.update_command_status();assert '5～360' in app.command_note.cget('text')
        app.target_speed.delete(0,tk.END);app.target_speed.insert(0,'45')
        app.send_home();assert fake.written[-1]==encode_position_command(0,45,2)
        app.pending_command=(2,time.monotonic()-3);app.update_command_status()
        assert app.pending_command is None and '超时' in app.command_note.cget('text')
        # 旧固件无指令字段时，不能误显示成功或发送命令。
        live[29]=float('nan');app.latest=tuple(live)
        app.send_position_target();assert len(fake.written)==2
        assert '固件' in app.command_note.cget('text')
        app.ser=None
    finally:app.close()
print('PASS scope: real AA5E RTL frames,30 channels,legacy,history+2D,sending signed target/home,ACK,timeout,validation; no serial opened')
