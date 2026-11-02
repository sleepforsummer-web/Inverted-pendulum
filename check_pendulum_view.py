"""离线验证姿态换算和生产绘图；不连接串口、不创建Tk窗口。"""
import json
import math
from pathlib import Path
import uuid
import time

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
import matplotlib
from matplotlib import font_manager
from pendulum_view import ViewCalibration, ModelSample, PendulumModelView, calculate_pose

fonts={f.name for f in font_manager.fontManager.ttflist}
for candidate in ['Microsoft YaHei','SimHei','Noto Sans CJK SC']:
    if candidate in fonts:
        matplotlib.rcParams['font.sans-serif']=[candidate]; break
matplotlib.rcParams['axes.unicode_minus']=False

def close(actual, expected):
    assert all(abs(a-b)<1e-9 for a,b in zip(actual,expected)),(actual,expected)

c=ViewCalibration()
for pos, endpoint in [(0,(0,-1)),(270,(-1,0)),(540,(0,1)),(810,(1,0)),
                      (1080,(0,-1)),(-270,(1,0)),(1080*5+270,(-1,0))]:
    p=calculate_pose(ModelSample(1,0,pos),c)
    close(p.arm_end,endpoint)
    close(p.pendulum_end_side,(0,1))
    assert abs(p.arm_total_degrees-pos*360/1080)<1e-9
for adc, endpoint in [(0,(0,1)),(1024,(1,0)),(-1024,(-1,0))]:
    p=calculate_pose(ModelSample(1,adc,0),c)
    close(p.pendulum_end_side,endpoint)
p=calculate_pose(ModelSample(1,1024,270,0,-810,1,30,1),c)
close(p.pendulum_end_top,(-1,-1))
close(p.target_arm_end,(0,-1))
assert abs(p.speed_degrees_s-36000/1080)<1e-9
assert calculate_pose(ModelSample(1,math.nan,0),c) is None
assert calculate_pose(ModelSample(1,0,math.inf),c) is None
assert calculate_pose(ModelSample(1,0,0),c).target_arm_end is None
print('PASS: signed ADC angles, clockwise encoder, fractional counts, multi-turn wrap, target and speed units')

# 两侧底部盲区显示向下；离开盲区立即恢复实测角度，不受前一帧影响。
for raw in [0, 100, 3990, 4092, 4095]:
    p=calculate_pose(ModelSample(1,c.center_adc-raw,0),c)
    assert p.in_bottom_blind_zone and p.reconstructed_adc == raw
    close(p.pendulum_end_side,(0,-1))
    close(p.pendulum_end_top,p.arm_end)
    assert p.pendulum_degrees == 180
for raw in [101, 1954, 3989]:
    theta=c.center_adc-raw
    p=calculate_pose(ModelSample(1,theta,0),c)
    assert not p.in_bottom_blind_zone
    close(p.pendulum_end_side,(math.sin(theta/4096*math.tau),math.cos(theta/4096*math.tau)))
stationary=ModelSample(2,100,0)
before=calculate_pose(stationary,c)
calculate_pose(ModelSample(3,1954,0),c)
after=calculate_pose(stationary,c)
close(after.pendulum_end_side,before.pendulum_end_side)
assert not after.in_bottom_blind_zone  # 历史回看、有效区域静止都不误判为底部。
assert calculate_pose(ModelSample(1,1958,0),ViewCalibration(center_adc=1958)).in_bottom_blind_zone
print('PASS: both blind-region boundaries, downward fallback, valid stationary angles and stateless history')

for value in [0,-1,math.nan,math.inf]:
    try: ViewCalibration(value,4096)
    except ValueError: pass
    else: raise AssertionError('invalid calibration accepted')
for args in [dict(center_adc=math.nan),dict(center_adc=4096),dict(blind_low_adc=1954),
             dict(blind_low_adc=-1),dict(blind_high_adc=1954),dict(blind_high_adc=4096),
             dict(blind_low_adc=math.nan),dict(blind_high_adc=math.inf)]:
    try: ViewCalibration(**args)
    except ValueError: pass
    else: raise AssertionError('invalid blind calibration accepted')
path=Path(__file__).with_name(f'.model_settings_{uuid.uuid4().hex}.json')
try:
    assert ViewCalibration.load(path)==c
    custom=ViewCalibration(2000,4000); custom.save(path)
    assert ViewCalibration.load(path)==custom
    path.write_text(json.dumps({'encoder_counts_per_turn':1080,'adc_counts_per_turn':4096}),encoding='utf-8')
    assert ViewCalibration.load(path)==c
    path.write_text('{bad json',encoding='utf-8')
    try: ViewCalibration.load(path)
    except ValueError: pass
    else: raise AssertionError('bad JSON accepted')
finally:
    path.unlink(missing_ok=True)
close(calculate_pose(ModelSample(1,1000,500),ViewCalibration(2000,4000)).arm_end,(-1,0))
close(calculate_pose(ModelSample(1,1000,500),ViewCalibration(2000,4000)).pendulum_end_side,(1,0))
print('PASS: display-only calibration validation, custom scales and saved settings')

fig=Figure(figsize=(9.2,6.4),dpi=100); canvas=FigureCanvasAgg(fig)
view=PendulumModelView(fig,c)
assert view.state is None and not any(a.get_visible() for a in view.artists)
view.set_sample(ModelSample(2,200,420,10,-116.5,2.5,120,1),'离线测试 · 合成数据')
assert view.rod.get_visible() and view.target_arm.get_visible()
assert '536.5' in view.readouts[2].get_text()
canvas.draw()
preview=Path(__file__).with_name('model_view_preview.png')
fig.savefig(preview,dpi=140)
view.set_sample(ModelSample(3,1954,0,0,0,0,0,0),'离线测试 · 底部盲区')
assert '默认竖直向下' in view.status.get_text()
fig.savefig(Path(__file__).with_name('model_bottom_preview.png'),dpi=140)
view.set_sample(ModelSample(4,0,0,0,0,0,0,1),'离线测试 · 有效ADC')
assert not view.state.in_bottom_blind_zone
close(view.state.pendulum_end_side,(0,1))
fig.savefig(Path(__file__).with_name('model_upright_preview.png'),dpi=140)
view.set_sample(ModelSample(4,200,420,10,-116.5,2.5,120,1))
view.set_calibration(ViewCalibration(2000,4000))
assert view.state.arm_total_degrees==420/2000*360
view.set_sample(ModelSample(3,0,-1073))
assert not view.target_arm.get_visible() and not view.target_rod.get_visible()
view.set_sample(None)
assert view.state is None and not any(a.get_visible() for a in view.artists)
assert all(t.get_text()=='--' for t in view.readouts)
start=time.perf_counter()
for i in range(20):
    view.set_sample(ModelSample(i*.05,120*math.sin(i/10),i*20,0,0,2,60,1),'离线测试')
    canvas.draw()
elapsed=time.perf_counter()-start
print(f'PASS: production view renders, updates existing artists, hides unavailable fields and clears;20frames {elapsed:.3f}s')
print('Preview:',preview)
