#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
db_pid_scope.py —— double_loop_pid(1) 补丁版 完全配套上位机
帧: AA5E双向位置控制76字节，兼容AA5D限幅诊断66字节，兼容AA5C/AA5B/AA5A/AA59/AA58/AA57/AA55，115200波特率。
内环P/I/D显示为PWM指令单位；外环P/I/D/目标角显示为ADC计数单位。
依赖: pyserial matplotlib numpy   用法: python db_pid_scope.py
"""
import sys, csv, time, threading, struct
from pathlib import Path
# Ask Windows to render the process at the display's native DPI before Tk exists.
if sys.platform == 'win32':
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (AttributeError, OSError):
        pass
from collections import deque
import numpy as np
import tkinter as tk
from tkinter import ttk
from matplotlib.figure import Figure
from matplotlib.ticker import FuncFormatter
from matplotlib.widgets import SpanSelector
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial, serial.tools.list_ports
from serial_commands import COUNTS_PER_REV, MAX_SPEED_DEGREES_S, COMMAND_STATUS, encode_position_command
from pendulum_view import PendulumModelView, ViewCalibration, ModelSample
from scope_theme import apply_theme, create_scope_figure, build_value_panel, BG, CARD, NAVY, MUTED

# ---------------- matplotlib 中文字体自动适配 ----------------
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from matplotlib import font_manager
_avail = {f.name for f in font_manager.fontManager.ttflist}
for _cand in ['Microsoft YaHei', 'SimHei', 'SimSun', 'Noto Sans CJK SC',
              'WenQuanYi Micro Hei', 'PingFang SC', 'Arial Unicode MS']:
    if _cand in _avail:
        plt.rcParams['font.sans-serif'] = [_cand]
        break
plt.rcParams['axes.unicode_minus'] = False   # 负号正常显示

FRAME_LENGTHS = {0x55: 17, 0x57: 40, 0x58: 44, 0x59: 48, 0x5A: 57, 0x5B: 61, 0x5C: 64, 0x5D: 66, 0x5E: 76}
OUTER_PAYLOAD = struct.Struct('<h i i h h h h i i h i i B')
STOP_PAYLOAD = struct.Struct('<BhB')
ADC_STOP_PAYLOAD = struct.Struct('<HH')
ADC_BLOCK_PAYLOAD = struct.Struct('<HHIB')
OUTER_I_PAYLOAD = struct.Struct('<i')
STOP_REASONS = {0: '无', 1: 'KEY1', 2: 'KEY2', 3: '角度越界', 4: 'KEY3'}
LIMIT_PHASES = {0: '非平衡/未启用', 1: '起摆收敛', 2: '等待受扰', 3: '受扰回落', 4: '等待回区', 5: '运动限幅', 6: '常规小限幅', 7: '常规大限幅'}

def s16(v): return v - 65536 if v >= 32768 else v
CH = ['theta摆角', 'theta_ref目标', '误差error', '位置pos', 'PWM duty',
      '内环P', '内环I', '内环D', '外环位置误差', '旋臂速度', '外环P', '外环D', '运行使能', '外环限幅',
      '停机原因', '停机theta', '停机按键', '停机平均ADC', '停机滤波ADC', '停机采样最小值', '停机采样最大值', '停机采样累加和', '停机采样数', '外环I', '实时限幅ADC', '限幅阶段', '指令终点pos', '旋臂轨迹目标', '指令序号', '指令状态']
COLORS = ['#087a43', '#145db5', '#bd2548', '#235fbb', '#bd5400', '#7a35b0', '#a45a00', '#087d75',
          '#c14b12', '#086b9d', '#8431a7', '#087d75', '#344b62', '#bd2548', '#bd2548', '#bd2548', '#344b62',
          '#805431', '#086b9d', '#805431', '#805431', '#086b9d', '#344b62', '#145db5', '#bd5400', '#344b62', '#6347ae', '#087a43', '#344b62', '#344b62']
OUTER_I_INDEX = CH.index('外环I')
DISPLAY_ORDER = [*range(11), OUTER_I_INDEX, *range(11, 23), 24, 25, 26, 27, 28, 29]
HISTORY_SECONDS = 600
DISPLAY_MS = 50
MIN_VIEW_SECONDS = 0.02
VIEW_SPANS = ('0.02', '0.05', '0.1', '0.2', '0.5', '1', '2', '5', '10', '30', '60', '120', '600')
PLOT_GROUPS = [
    ('摆杆角度内环', [('摆角 / 目标 / 误差（ADC）', [0, 1, 2]),
                  ('电机PWM（虚线：±500）', [4]),
                  ('内环 P / D（PWM指令单位）', [5, 7])]),
    ('旋臂位置外环', [('位置 / 误差 / 轨迹（计数）', [3, 8, 27]),
                  ('旋臂速度（编码器计数 / 10ms）', [9]),
                  ('外环 P / I / D（ADC计数）', [10, OUTER_I_INDEX, 11]),
                  ]),
]

def decode_values(kind, payload):
    if kind == 0x55:
        theta, err, pos, duty, p, i, d = struct.unpack('<7h', payload)
        # Missing outer-loop data is unknown, rather than zero.
        return (theta, theta+err, err, pos, duty, p/16, i/16, d/16, *([float('nan')]*(len(CH)-8)))
    theta, err, pos, duty, p, i, d, ref, pos_err, speed, outer_p, outer_d, flags = OUTER_PAYLOAD.unpack(payload[:OUTER_PAYLOAD.size])
    stop_end = OUTER_PAYLOAD.size + STOP_PAYLOAD.size
    stop_values = STOP_PAYLOAD.unpack(payload[OUTER_PAYLOAD.size:stop_end]) if kind in (0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E) else (float('nan'),)*3
    adc_end = stop_end + ADC_STOP_PAYLOAD.size
    adc_values = ADC_STOP_PAYLOAD.unpack(payload[stop_end:adc_end]) if kind in (0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E) else (float('nan'),)*2
    block_end = adc_end + ADC_BLOCK_PAYLOAD.size
    block_values = ADC_BLOCK_PAYLOAD.unpack(payload[adc_end:block_end]) if kind in (0x5A, 0x5B, 0x5C, 0x5D, 0x5E) else (float('nan'),)*4
    outer_i = OUTER_I_PAYLOAD.unpack(payload[block_end:block_end+OUTER_I_PAYLOAD.size])[0]/256 if kind in (0x5B,0x5C,0x5D,0x5E) else float('nan')
    # 跳过旧倍率保留字节，仅读取限幅状态；保留旧版帧兼容。
    status = payload[block_end+6] if kind in (0x5C,0x5D,0x5E) else 0
    diagnostics = (struct.unpack_from('<H', payload, block_end+7)[0], (status>>2)&7) if kind in (0x5D,0x5E) else (float('nan'),)*2
    remote = (float('nan'),)*4
    if kind == 0x5E:
        goal, trajectory, seq, state = struct.unpack_from('<iiBB', payload, 63)
        remote = (goal/256, trajectory/256, seq, state)
    return (theta, ref, err, pos, duty, p/16, i/16, d/16, pos_err, speed,
            outer_p/256, outer_d/256, flags & 1, (flags >> 1) & 1, *stop_values, *adc_values, *block_values, outer_i, *diagnostics, *remote)

def display_value(value):
    if not np.isfinite(value):
        return '--'
    return str(int(value)) if float(value).is_integer() else f'{value:.3f}'

def display_channel_value(name, value):
    if not np.isfinite(value):
        return '--'
    if name == '停机原因':
        return STOP_REASONS.get(int(value), f'未知{int(value)}')
    if name == '停机按键':
        return f'{int(value):02b}'  # left KEY2, right KEY1; 0 pressed / 1 released
    if name == '指令状态':
        return COMMAND_STATUS.get(int(value), f'未知{int(value)}')
    if name == '限幅阶段':
        return LIMIT_PHASES.get(int(value), f'未知{int(value)}')
    return display_value(value)

def nearest_sample(data, x):
    """Pick an actual received row, including rows omitted from the drawing."""
    if len(data) == 0 or x < data[0, 0] or x > data[-1, 0]:
        return None
    index = int(np.searchsorted(data[:, 0], x))
    if index > 0 and (index == len(data) or x-data[index-1, 0] <= data[index, 0]-x):
        index -= 1
    return data[index]

def plot_indices(values, buckets=2000):
    """Reduce drawing work while preserving positive and negative peaks."""
    if len(values) <= buckets * 2:
        return np.arange(len(values))
    stride = int(np.ceil(len(values) / buckets))
    usable = len(values) // stride * stride
    blocks = values[:usable].reshape(-1, stride)
    starts = np.arange(len(blocks)) * stride
    return np.unique(np.concatenate(([0], starts + blocks.argmin(axis=1),
                                     starts + blocks.argmax(axis=1),
                                     np.arange(usable, len(values)), [len(values)-1])))

class Parser:
    def __init__(self):
        self.buf = bytearray()
        self.bytes_received = 0
        self.valid_frames = 0
        self.checksum_errors = 0
        self.last_protocol = ''
    def feed(self, data):
        self.bytes_received += len(data)
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b'\xaa')
            if i < 0:
                self.buf.clear()
                break
            if len(self.buf) <= i+1:
                self.buf = self.buf[i:]
                break
            kind = self.buf[i+1]
            if kind not in FRAME_LENGTHS:
                self.buf = self.buf[i+1:]
                continue
            length = FRAME_LENGTHS[kind]
            if len(self.buf) < i + length:
                if i > 0: self.buf = self.buf[i:]
                break
            pl, chk = self.buf[i+2:i+length-1], self.buf[i+length-1]
            x = 0
            for value in pl: x ^= value
            if chk != x:
                self.checksum_errors += 1
                self.buf = self.buf[i+1:]
                continue
            self.buf = self.buf[i+length:]
            self.valid_frames += 1
            self.last_protocol = {0x5E: '双向位置控制 AA5E', 0x5D: '固定系数PID AA5D（实时限幅诊断）', 0x5C: '兼容旧版 AA5C（未发送限幅诊断）', 0x5B: '双环AA5B（外环P/I/D）', 0x5A: '双环AA5A（ADC采样诊断）', 0x59: '双环AA59（ADC停机诊断）', 0x58: '双环AA58（停机诊断）', 0x57: '双环AA57',
                                  0x55: '旧版AA55（外环项未发送）'}[kind]
            out.append(decode_values(kind, pl))
        return out

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.ui_style = apply_theme(self)
        self.title('双环PID调试台 — 固定系数PID / 二维姿态 / 历史回看')
        self.geometry(f'{min(1440, self.winfo_screenwidth()-80)}x{min(940, self.winfo_screenheight()-90)}')
        self.minsize(1080, 720)
        self.samples = deque()
        self.t0 = time.monotonic()
        self.paused = False
        self.paused_samples = []
        self.view_end = 0.0
        self.view_dirty = True
        self.setting_slider = False
        self.visible_data = np.empty((0, len(CH)+1))
        self.hover_axis = None
        self.hover_x = None
        self.hover_last = 0.0
        self.active_tab = 0
        self.time_origin = 0.0
        self.display_in_ms = False
        self.selectors = []
        self.ser = None; self.csv_f = None; self.latest = None
        self.lock = threading.Lock()
        self.parser = Parser()
        self.rx_error = ''
        self.closing = False
        self.latest_received = 0.0
        self.command_sequence = 0
        self.pending_command = None
        self.last_command_reply = None
        self.fps_cnt = 0; self.fps_t = time.time()

        header = tk.Frame(self, bg=NAVY); header.pack(side=tk.TOP, fill=tk.X)
        tk.Label(header, text='倒立摆调试台', bg=NAVY, fg='#172b45',
                 font=('Microsoft YaHei', 18, 'bold')).pack(side=tk.LEFT, padx=(20, 16), pady=12)
        tk.Label(header, text='实时采集  /  双环 PID  /  二维姿态', bg=NAVY, fg='#52657a',
                 font=('Microsoft YaHei', 10)).pack(side=tk.LEFT)
        self.connection_badge = tk.Label(header, text='● 未连接', bg=NAVY, fg='#52657a',
                                         font=('Microsoft YaHei', 10, 'bold'))
        self.connection_badge.pack(side=tk.RIGHT, padx=22)
        dashboard = ttk.Frame(self, style='App.TFrame')
        dashboard.pack(fill=tk.BOTH, expand=True, padx=10, pady=6)
        dashboard.columnconfigure(0, weight=1)
        dashboard.columnconfigure(1, weight=0, minsize=700)
        dashboard.rowconfigure(0, weight=1)
        left = ttk.Frame(dashboard, style='App.TFrame')
        left.grid(row=0, column=0, sticky='nsew', padx=(0, 6))
        right = ttk.Frame(dashboard, style='App.TFrame', width=700)
        right.grid_propagate(False)
        right.grid(row=0, column=1, sticky='nsew')
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1, uniform='right')
        right.rowconfigure(1, weight=1, uniform='right')
        bar = ttk.Frame(left, style='Card.TFrame', padding=(6, 3))
        bar.pack(side=tk.TOP, fill=tk.X, padx=14, pady=(12, 6))
        ttk.Label(bar, text='串口', style='Section.TLabel').pack(side=tk.LEFT, padx=(0, 8))
        self.cmb = ttk.Combobox(bar, width=12); self.refresh_ports()
        ttk.Button(bar, text='刷新', command=self.refresh_ports).pack(side=tk.LEFT)
        self.cmb.pack(side=tk.LEFT, padx=2)
        self.btn = ttk.Button(bar, text='连接', command=self.toggle, style='Primary.TButton'); self.btn.pack(side=tk.LEFT, padx=6)
        self.btn_r = ttk.Button(bar, text='记录CSV', command=self.toggle_rec); self.btn_r.pack(side=tk.LEFT)
        ttk.Button(bar, text='清屏', command=self.clear).pack(side=tk.LEFT, padx=4)
        command_bar = ttk.Frame(left, style='Card.TFrame', padding=(6, 3))
        command_bar.pack(side=tk.TOP, fill=tk.X, padx=14, pady=(0, 6))
        ttk.Label(command_bar, text='旋臂目标', style='Section.TLabel').pack(side=tk.LEFT)
        self.target_entry = ttk.Entry(command_bar, width=9)
        self.target_entry.insert(0, '90'); self.target_entry.pack(side=tk.LEFT, padx=6)
        ttk.Label(command_bar, text='°   速度').pack(side=tk.LEFT)
        self.target_speed = ttk.Entry(command_bar, width=6)
        self.target_speed.insert(0, '45'); self.target_speed.pack(side=tk.LEFT, padx=5)
        ttk.Label(command_bar, text=f'°/秒（5～{MAX_SPEED_DEGREES_S}）').pack(side=tk.LEFT)
        self.send_target_btn = ttk.Button(command_bar, text='发送目标', command=self.send_position_target, style='Send.Primary.TButton')
        self.send_target_btn.pack(side=tk.LEFT, padx=7)
        self.home_btn = ttk.Button(command_bar, text='回零', command=self.send_home)
        self.home_btn.pack(side=tk.LEFT)
        self.command_note = ttk.Label(left, text=f'绝对位置 · {COUNTS_PER_REV}计数/圈；先稳定平衡再发送目标', style='Hint.TLabel')
        self.command_note.pack(side=tk.TOP, anchor='w', padx=20, pady=(0, 4))
        self.send_target_btn.state(['disabled']); self.home_btn.state(['disabled'])
        footer = ttk.Frame(left, style='App.TFrame')
        footer.pack(side=tk.BOTTOM, fill=tk.X, padx=18, pady=(4, 9))
        self.lbl = ttk.Label(footer, text='未连接 | 0 Hz', style='Hint.TLabel')
        self.lbl.pack(side=tk.LEFT)

        controls = ttk.Frame(left, padding=(6, 3)); controls.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 1))
        self.pause_btn = ttk.Button(controls, text='暂停显示', command=self.toggle_pause, style='Primary.TButton')
        self.pause_btn.pack(side=tk.LEFT, padx=4)
        ttk.Button(controls, text='前一段', command=lambda: self.shift_view(-1)).pack(side=tk.LEFT)
        ttk.Button(controls, text='后一段', command=lambda: self.shift_view(1)).pack(side=tk.LEFT, padx=4)
        ttk.Label(controls, text='窗口（秒）').pack(side=tk.LEFT)
        self.span_box = ttk.Combobox(controls, values=VIEW_SPANS, width=6, state='readonly')
        self.span_box.set('30'); self.span_box.pack(side=tk.LEFT, padx=4)
        self.span_box.bind('<<ComboboxSelected>>', lambda event: self.mark_view_dirty())
        ttk.Button(controls, text='保存历史CSV', command=self.save_history).pack(side=tk.LEFT, padx=4)
        self.view_label = ttk.Label(controls, text='实时显示 | 保留最近10分钟', style='Muted.TLabel')
        self.view_label.pack(side=tk.LEFT, padx=4)
        zoom_bar = ttk.Frame(left, padding=(6, 3)); zoom_bar.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 1))
        ttk.Button(zoom_bar, text='放大 ×2', command=lambda: self.zoom_view(0.5)).pack(side=tk.LEFT)
        ttk.Button(zoom_bar, text='缩小 ×2', command=lambda: self.zoom_view(2)).pack(side=tk.LEFT, padx=4)
        self.select_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(zoom_bar, text='框选时间段', variable=self.select_var,
                        command=self.toggle_selection).pack(side=tk.LEFT, padx=4)
        ttk.Label(zoom_bar, text='时间单位').pack(side=tk.LEFT, padx=4)
        self.unit_box = ttk.Combobox(zoom_bar, values=('自动', '秒', '毫秒'), width=5, state='readonly')
        self.unit_box.set('自动'); self.unit_box.pack(side=tk.LEFT)
        self.unit_box.bind('<<ComboboxSelected>>', lambda event: self.mark_view_dirty())
        ttk.Label(zoom_bar, text='刷新(ms)').pack(side=tk.LEFT, padx=6)
        self.refresh_box = ttk.Combobox(zoom_bar, values=('50', '100', '200', '500'), width=5, state='readonly')
        self.refresh_box.set(str(DISPLAY_MS)); self.refresh_box.pack(side=tk.LEFT)
        slider_bar = ttk.Frame(left, padding=(6, 2)); slider_bar.pack(side=tk.TOP, fill=tk.X, padx=8)
        ttk.Label(slider_bar, text='历史 ←').pack(side=tk.LEFT)
        self.history_scale = ttk.Scale(slider_bar, from_=0, to=1, command=self.on_history_scroll)
        self.history_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
        ttk.Label(slider_bar, text='→ 最新').pack(side=tk.RIGHT)
        self.cursor_label = ttk.Label(left, text='鼠标移到曲线上查看数值；滚轮放大后可查看毫秒细节', style='Hint.TLabel')
        self.cursor_label.pack(side=tk.TOP, anchor='w', padx=19, pady=(5, 7))

        mid = ttk.Frame(left, style='App.TFrame'); mid.pack(fill=tk.BOTH, expand=True, padx=8)
        mid.columnconfigure(0, weight=1, uniform='plots')
        mid.columnconfigure(1, weight=1, uniform='plots')
        mid.rowconfigure(0, weight=1)
        self.axes = []
        self.canvases = []
        self.cur = {}
        self.plot_channels = {}
        self.cursor_lines = {}
        self.cursor_markers = {}
        self.cursor_notes = {}
        for column, (tab_title, plots) in enumerate(PLOT_GROUPS):
            page = ttk.Frame(mid)
            page.grid(row=0, column=column, sticky='nsew', padx=(0, 6))
            ttk.Label(page, text=tab_title, style='Section.TLabel').pack(anchor='w', padx=8, pady=(4, 0))
            fig, plot_axes = create_scope_figure(plots, self.format_time_tick)
            for ax, (title, indices) in zip(plot_axes, plots):
                self.axes.append(ax)
                for index in indices:
                    name = CH[index]
                    line, = ax.plot([], [], lw=1.0, color=COLORS[index], label=name)
                    self.cur.setdefault(name, []).append(line)
                if indices == [4]:
                    ax.axhline(500, color='red', ls='--', lw=0.6)
                    ax.axhline(-500, color='red', ls='--', lw=0.6)
                ax.legend(fontsize=8, loc='upper right', frameon=False, labelcolor='#172b45', ncol=2)
                self.plot_channels[ax] = indices
                self.cursor_lines[ax] = ax.axvline(0, color='#607c9a', ls='--', lw=0.8, visible=False)
                self.cursor_markers[ax] = {}
                for index in indices:
                    marker, = ax.plot([], [], 'o', color=COLORS[index], ms=4, visible=False,
                                      label='_nolegend_', zorder=10)
                    self.cursor_markers[ax][index] = marker
                self.cursor_notes[ax] = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                                               va='top', ha='left', fontsize=8, zorder=20,
                                               color='#172b45', bbox=dict(boxstyle='round,pad=0.4', fc='#ffffff', ec='#cbd5e1', alpha=0.95),
                                               visible=False)
            canvas = FigureCanvasTkAgg(fig, master=page)
            canvas.get_tk_widget().configure(background=CARD, highlightthickness=0, borderwidth=0)
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            self.canvases.append(canvas)
            for ax in fig.axes:
                selector = SpanSelector(ax, self.select_time_range, 'horizontal',
                                        useblit=True, minspan=MIN_VIEW_SECONDS-1e-9,
                                        props=dict(facecolor='#2196f3', alpha=0.2), button=1)
                selector.set_active(False)
                self.selectors.append(selector)

        model_page = ttk.Frame(right)
        model_page.grid(row=0, column=0, sticky='nsew', pady=(0, 6))
        model_header = ttk.Frame(model_page); model_header.pack(side=tk.TOP, fill=tk.X, padx=5, pady=4)
        ttk.Label(model_header, text='实时二维姿态', style='Section.TLabel').pack(side=tk.LEFT)
        model_settings = tk.Toplevel(self)
        model_settings.title('姿态显示换算设置')
        model_settings.withdraw()
        model_settings.protocol('WM_DELETE_WINDOW', model_settings.withdraw)
        ttk.Button(model_header, text='显示设置', command=model_settings.deiconify).pack(side=tk.RIGHT)
        model_bar = ttk.Frame(model_settings); model_bar.pack(side=tk.TOP, fill=tk.X, padx=8, pady=5)
        settings_path = Path(__file__).with_name('pendulum_view_settings.json')
        calibration_note = ''
        try:
            calibration = ViewCalibration.load(settings_path)
        except (OSError, ValueError, TypeError, KeyError) as exc:
            calibration = ViewCalibration()
            calibration_note = '显示配置无效，暂用1080/4096；应用后可重存'
        ttk.Label(model_bar, text='旋臂计数/圈').pack(side=tk.LEFT)
        self.model_encoder = ttk.Entry(model_bar, width=7); self.model_encoder.pack(side=tk.LEFT, padx=4)
        self.model_encoder.insert(0, f'{calibration.encoder_counts_per_turn:g}')
        ttk.Label(model_bar, text='摆杆ADC/圈').pack(side=tk.LEFT)
        self.model_adc = ttk.Entry(model_bar, width=7); self.model_adc.pack(side=tk.LEFT, padx=4)
        self.model_adc.insert(0, f'{calibration.adc_counts_per_turn:g}')
        ttk.Button(model_bar, text='应用显示换算', command=self.apply_model_calibration).pack(side=tk.LEFT, padx=4)
        self.model_note = ttk.Label(model_bar, text=calibration_note or '仅影响显示，不修改FPGA')
        self.model_note.pack(side=tk.LEFT, padx=4)
        model_adc_bar = ttk.Frame(model_settings)
        model_adc_bar.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 5))
        for attr, label, number in (
            ('model_center', '中心ADC', calibration.center_adc),
            ('model_blind_low', '底部盲区 ADC≤', calibration.blind_low_adc),
            ('model_blind_high', '或 ADC≥', calibration.blind_high_adc),
        ):
            ttk.Label(model_adc_bar, text=label).pack(side=tk.LEFT)
            entry = ttk.Entry(model_adc_bar, width=6)
            entry.pack(side=tk.LEFT, padx=4)
            entry.insert(0, f'{number:g}')
            setattr(self, attr, entry)
        ttk.Label(model_adc_bar, text='盲区显示向下；中心ADC须与烧录工程一致').pack(side=tk.LEFT, padx=4)
        model_figure = Figure(figsize=(9.2, 6.4), dpi=100)
        self.model = PendulumModelView(model_figure, calibration, compact=True)
        model_canvas = FigureCanvasTkAgg(model_figure, master=model_page)
        model_canvas.get_tk_widget().configure(background=CARD, highlightthickness=0, borderwidth=0)
        model_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvases.append(model_canvas)

        panel = ttk.Frame(right, style='App.TFrame')
        panel.grid(row=1, column=0, sticky='nsew')
        self.values_title, self.vals = build_value_panel(panel, CH, COLORS)
        for canvas in self.canvases[:len(PLOT_GROUPS)]:
            canvas.mpl_connect('motion_notify_event', self.on_hover)
            canvas.mpl_connect('scroll_event', self.on_plot_scroll)
            canvas.mpl_connect('figure_leave_event', lambda event: self.clear_hover())
        self.protocol('WM_DELETE_WINDOW', self.close)
        self.after(DISPLAY_MS, self.update_ui)

    def send_home(self):
        self.target_entry.delete(0, tk.END)
        self.target_entry.insert(0, '0')
        self.send_position_target()

    def send_position_target(self):
        with self.lock:
            values = self.latest
            fresh = time.monotonic() - self.latest_received < 1.0
        if not self.ser or values is None or not fresh:
            self.command_note.config(text='未发送：请先连接串口并确认实时数据正在更新。')
            return
        if not np.isfinite(values[29]):
            self.command_note.config(text='未发送：当前固件未提供双向控制，请烧录AA5E新版工程。')
            return
        if values[12] != 1:
            self.command_note.config(text='未发送：当前未运行，请先起摆并稳定进入PID平衡。')
            return
        if self.pending_command is not None:
            self.command_note.config(text='正在等待上一条指令回执，请稍候。')
            return
        seq = (self.command_sequence+1) & 255
        if seq == int(values[28]):
            seq = (seq+1) & 255
        try:
            frame = encode_position_command(self.target_entry.get(), self.target_speed.get(), seq)
            written = self.ser.write(frame)
            if written != len(frame):
                raise OSError('指令未完整写出')
        except (ValueError, OSError, serial.SerialException) as exc:
            self.command_note.config(text=f'未发送成功：{exc}')
            return
        self.command_sequence = seq
        self.pending_command = (seq, time.monotonic())
        self.command_note.config(text=f'指令#{seq}已发送，等待FPGA回执；不会自动重复发送。')

    def update_command_status(self):
        with self.lock:
            values = self.latest
            fresh = time.monotonic()-self.latest_received < 1.0
        connected = self.ser is not None and values is not None and fresh
        supported = connected and np.isfinite(values[29])
        available = supported and values[12] == 1 and self.pending_command is None
        for button in (self.send_target_btn, self.home_btn):
            button.state(['!disabled'] if available else ['disabled'])
        if self.pending_command is not None:
            seq, sent = self.pending_command
            if supported and int(values[28]) == seq:
                self.command_note.config(text=f'指令#{seq}：{COMMAND_STATUS.get(int(values[29]), "未知状态")}')
                self.last_command_reply = (int(values[28]), int(values[29]))
                self.pending_command = None
            elif time.monotonic()-sent > 2:
                self.command_note.config(text=f'指令#{seq}回执超时，未确认是否执行；请检查固件和实时数据。')
                self.last_command_reply = (int(values[28]), int(values[29])) if supported else None
                self.pending_command = None
        elif supported and int(values[29]) != 0 and self.last_command_reply != (int(values[28]), int(values[29])):
            angle = values[26]*360/COUNTS_PER_REV
            self.command_note.config(text=f'指令#{int(values[28])}：{COMMAND_STATUS.get(int(values[29]), "未知状态")}；最近接受终点 {angle:.2f}°')
            self.last_command_reply = (int(values[28]), int(values[29]))

    def refresh_ports(self):
        ps = [p.device for p in serial.tools.list_ports.comports()]
        self.cmb['values'] = ps
        if ps: self.cmb.set('COM5' if 'COM5' in ps else ps[0])

    def toggle(self):
        if self.ser:
            old_ser = self.ser
            self.ser = None
            old_ser.close()
            self.btn.config(text='连接'); self.lbl.config(text='未连接')
            if self.csv_f: self.toggle_rec()
            return
        port = self.cmb.get()
        if not port: return
        try:
            device = serial.Serial(port=None, baudrate=115200, timeout=0.05, write_timeout=0.2)
            device.dtr = False
            device.rts = False
            device.port = port
            device.open()
        except Exception as e:
            self.lbl.config(text=f'打开失败: {e}'); return
        self.ser = device
        with self.lock:
            self.parser = Parser()
            self.rx_error = ''
            self.fps_cnt = 0
            self.fps_t = time.time()
        self.clear()
        threading.Thread(target=self.rx, args=(device, self.parser), daemon=True).start()
        self.btn.config(text='断开'); self.lbl.config(text=f'已连接 {port}')

    def rx(self, device, parser):
        while self.ser is device and not self.closing:
            # Wait for one byte, then drain what is already available. Waiting
            # for512 bytes batched about45ms of data before the UI could see it.
            try: data = device.read(max(1, min(getattr(device, 'in_waiting', 0), 4096)))
            except Exception as exc:
                if self.ser is device:
                    with self.lock:
                        self.rx_error = str(exc)
                break
            if not data: continue
            with self.lock:
                if self.ser is not device:
                    break
                for vals in parser.feed(data):
                    t = time.monotonic() - self.t0
                    self.samples.append((t, *vals))
                    while self.samples and self.samples[0][0] < t - HISTORY_SECONDS:
                        self.samples.popleft()
                    self.latest = vals
                    self.latest_received = time.monotonic()
                    self.fps_cnt += 1
                    if self.csv_f:
                        self.csv_w.writerow([f'{t:.6f}'] + list(vals))

    def toggle_rec(self):
        with self.lock:
            if self.csv_f:
                self.csv_f.close(); self.csv_f = None
                self.btn_r.config(text='记录CSV')
            else:
                csv_path = Path(__file__).resolve().parent / time.strftime('dbl_%Y%m%d_%H%M%S.csv')
                self.csv_f = open(csv_path, 'w', newline='', encoding='utf-8-sig')
                self.csv_w = csv.writer(self.csv_f)
                self.csv_w.writerow(['t_s'] + CH)
                self.btn_r.config(text='停止记录')

    def mark_view_dirty(self):
        self.clear_hover()
        self.view_dirty = True

    def draw_active(self):
        for canvas in self.canvases:
            canvas.draw_idle()

    def apply_model_calibration(self):
        try:
            calibration = ViewCalibration(
                float(self.model_encoder.get()), float(self.model_adc.get()),
                float(self.model_center.get()), float(self.model_blind_low.get()),
                float(self.model_blind_high.get()))
        except ValueError as exc:
            self.model_note.config(text=f'未应用：{exc}')
            return
        self.model.set_calibration(calibration)
        try:
            calibration.save(Path(__file__).with_name('pendulum_view_settings.json'))
            self.model_note.config(text='显示换算已保存；FPGA参数未改')
        except OSError:
            self.model_note.config(text='本次显示已应用，配置未能保存')
        self.draw_active()

    def format_time_tick(self, value, position=None):
        return f'{(value-self.time_origin)*1000:g}' if self.display_in_ms else f'{value:g}'

    def set_view(self, span, end):
        span = min(HISTORY_SECONDS, max(MIN_VIEW_SECONDS, float(span)))
        if not self.paused:
            self.pause_view()
        if not self.paused_samples:
            return
        self.span_box.set(f'{span:g}')
        self.view_end = min(self.paused_samples[-1][0], max(self.paused_samples[0][0], end))
        self.mark_view_dirty()

    def zoom_view(self, factor, anchor=None, fraction=0.5):
        if not self.paused:
            self.pause_view()
        if not self.paused_samples:
            return
        old_span = float(self.span_box.get())
        new_span = min(HISTORY_SECONDS, max(MIN_VIEW_SECONDS, old_span*factor))
        if anchor is None:
            # At startup the nominal window may extend long before the first
            # received point. Center on available history, not that empty area.
            left = max(self.paused_samples[0][0], self.view_end-old_span)
            anchor = (left+self.view_end)/2
        self.set_view(new_span, anchor + (1-fraction)*new_span)

    def on_plot_scroll(self, event):
        if event.inaxes not in self.plot_channels or event.xdata is None:
            return
        left, right = event.inaxes.get_xlim()
        fraction = min(1.0, max(0.0, (event.xdata-left)/(right-left)))
        self.zoom_view(0.5 if event.button == 'up' else 2, float(event.xdata), fraction)

    def toggle_selection(self):
        for selector in self.selectors:
            selector.set_active(self.select_var.get())
            if not self.select_var.get(): selector.clear()
        self.clear_hover()

    def select_time_range(self, left, right):
        if right-left < MIN_VIEW_SECONDS-1e-9:
            return
        self.set_view(right-left, right)
        self.select_var.set(False)
        self.toggle_selection()

    def show_values(self, row, hovered=False):
        title = '鼠标所选时刻数值' if hovered else '当前显示时刻数值'
        if self.values_title.cget('text') != title:
            self.values_title.config(text=title)
        for name, value in zip(CH, row[1:]):
            label = self.vals[name]
            text = display_channel_value(name, value)
            if label.cget('text') != text:
                label.config(text=text)
        values = dict(zip(CH, row[1:]))
        mode = '鼠标定位历史' if hovered else '暂停 / 历史回看' if self.paused else '实时数据'
        if not hovered and not self.paused:
            if not self.ser:
                mode = '未连接 · 最后收到的姿态'
            elif time.monotonic()-self.t0-row[0] > .5:
                mode = '数据已停更 · 保留最后姿态'
        self.model.set_sample(ModelSample(row[0], values['theta摆角'], values['位置pos'],
            values['theta_ref目标'], values['外环位置误差'], values['旋臂速度'],
            values['PWM duty'], values['运行使能']), mode)

    def on_hover(self, event):
        if event.inaxes not in self.plot_channels or event.xdata is None:
            self.clear_hover()
            return
        now = time.monotonic()
        if now-self.hover_last < 0.08:
            return
        self.hover_last = now
        self.hover_axis = event.inaxes
        self.hover_x = float(event.xdata)
        self.refresh_hover()

    def refresh_hover(self, redraw=True):
        if self.hover_axis is None:
            return
        row = nearest_sample(self.visible_data, self.hover_x)
        if row is None:
            self.clear_hover(redraw=redraw)
            return
        figure = self.hover_axis.figure
        for ax, indices in self.plot_channels.items():
            active = ax.figure is figure
            self.cursor_lines[ax].set_visible(active)
            self.cursor_lines[ax].set_xdata([row[0], row[0]])
            for index in indices:
                marker = self.cursor_markers[ax][index]
                marker.set_visible(active and np.isfinite(row[index+1]))
                marker.set_data([row[0]], [row[index+1]])
            self.cursor_notes[ax].set_visible(ax is self.hover_axis)
        note = self.cursor_notes[self.hover_axis]
        tooltip_indices = self.plot_channels[self.hover_axis]
        if any(i in tooltip_indices for i in (10, OUTER_I_INDEX, 11)):
            tooltip_indices = [10, OUTER_I_INDEX, 11, 1, 24, 25]
        time_text = f't = {row[0]:.6f} 秒'
        if self.display_in_ms:
            time_text += f'\n窗口内 = {(row[0]-self.time_origin)*1000:.3f} ms'
        note.set_text(time_text + '\n' + '\n'.join(
            f'{CH[index]} = {display_channel_value(CH[index], row[index+1])}' for index in tooltip_indices))
        left, right = self.hover_axis.get_xlim()
        on_left = row[0] <= (left+right)/2
        note.set_position((0.98 if on_left else 0.02, 0.98))
        note.set_horizontalalignment('right' if on_left else 'left')
        self.show_values(row, hovered=True)
        self.cursor_label.config(text=f'鼠标所选接收时刻：{row[0]:.6f} 秒 / {row[0]*1000:.3f} ms')
        if redraw:
            self.draw_active()

    def clear_hover(self, redraw=True):
        had_hover = self.hover_axis is not None
        self.hover_axis = None
        self.hover_x = None
        for line in self.cursor_lines.values(): line.set_visible(False)
        for markers in self.cursor_markers.values():
            for marker in markers.values(): marker.set_visible(False)
        for note in self.cursor_notes.values(): note.set_visible(False)
        self.cursor_label.config(text='鼠标移到曲线上查看数值；滚轮放大后可查看毫秒细节')
        if len(self.visible_data):
            self.show_values(self.visible_data[-1])
        else:
            self.values_title.config(text='当前显示时刻数值')
        if redraw and had_hover:
            self.draw_active()

    def pause_view(self, end=None):
        with self.lock:
            self.paused_samples = list(self.samples)
        self.paused = True
        self.clear_hover(redraw=False)
        self.view_end = (self.paused_samples[-1][0] if self.paused_samples else 0.0) if end is None else end
        self.view_dirty = True
        self.pause_btn.config(text='继续实时')

    def toggle_pause(self):
        if not self.paused:
            self.pause_view()
        else:
            self.paused = False
            self.paused_samples = []
            self.view_dirty = True
            self.pause_btn.config(text='暂停显示')

    def on_history_scroll(self, value):
        if self.setting_slider:
            return
        self.clear_hover()
        end = float(value)
        if not self.paused:
            self.pause_view(end=end)
        else:
            self.view_end = end
            self.view_dirty = True

    def shift_view(self, direction):
        self.clear_hover()
        if not self.paused:
            self.pause_view()
        if not self.paused_samples:
            return
        self.view_end = min(self.paused_samples[-1][0], max(self.paused_samples[0][0],
                              self.view_end + direction * float(self.span_box.get()) / 2))
        self.view_dirty = True

    def save_history(self):
        with self.lock:
            rows = list(self.paused_samples if self.paused else self.samples)
        if not rows:
            self.view_label.config(text='暂无历史数据')
            return
        path = Path(__file__).resolve().parent / time.strftime('history_%Y%m%d_%H%M%S.csv')
        try:
            with open(path, 'w', newline='', encoding='utf-8-sig') as output:
                writer = csv.writer(output)
                writer.writerow(['t_s'] + CH)
                writer.writerows(rows)
            self.view_label.config(text=f'已保存 {path.name}')
        except OSError as exc:
            self.view_label.config(text=f'保存失败: {exc}')

    def update_ui(self):
        if self.closing:
            return
        try:
            self.update_command_status()
            with self.lock:
                now = time.time()
                elapsed = now - self.fps_t
                if elapsed >= 1:
                    rate = self.fps_cnt / elapsed
                    self.fps_cnt = 0; self.fps_t = now
                    link = f'已连接 {self.ser.port}' if self.ser else '未连接'
                    self.connection_badge.config(text='● 串口已连接' if self.ser else '● 未连接',
                                                 fg='#087a43' if self.ser else '#52657a')
                    self.lbl.config(text=f'{link} | {rate:.0f} Hz | 字节={self.parser.bytes_received} '
                                         f'有效帧={self.parser.valid_frames} 校验错={self.parser.checksum_errors}'
                                         + (f' | {self.parser.last_protocol}' if self.parser.last_protocol else '')
                                         + (f' | 接收失败: {self.rx_error}' if self.rx_error else ''))
                span = float(self.span_box.get())
                history = self.paused_samples if self.paused else self.samples
                earliest = history[0][0] if history else 0.0
                newest = history[-1][0] if history else 0.0
                end = min(newest, max(earliest, self.view_end)) if self.paused else newest
                start = max(earliest, end - span)
                rows = []
                if not self.paused or self.view_dirty:
                    for row in reversed(history):
                        if row[0] > end:
                            continue
                        if row[0] < start:
                            break
                        rows.append(row)
                    rows.reverse()
            self.setting_slider = True
            try:
                self.history_scale.configure(from_=earliest, to=max(earliest+0.001, newest))
                self.history_scale.set(end)
            finally:
                self.setting_slider = False
            if rows:
                data = np.asarray(rows)
                self.visible_data = data
                for column, name in enumerate(CH, 1):
                    if name not in self.cur:
                        continue
                    idx = plot_indices(data[:, column])
                    for line in self.cur[name]:
                        line.set_data(data[idx, 0], data[idx, column])
                left = max(0.0, end-span)
                right = max(left+MIN_VIEW_SECONDS, end+span*0.01)
                self.time_origin = left
                self.display_in_ms = self.unit_box.get() == '毫秒' or (self.unit_box.get() == '自动' and span <= 1)
                for markers in self.cursor_markers.values():
                    for marker in markers.values(): marker.set_visible(False)
                for ax in self.axes:
                    ax.relim(visible_only=True); ax.autoscale_view(scalex=False)
                    ax.set_xlim(left, right)
                    if ax.get_subplotspec().is_last_row():
                        ax.set_xlabel(f'窗口内时间（ms），起点 {left:.6f} 秒' if self.display_in_ms else '时间（秒）', fontsize=8)
                if self.hover_axis is not None:
                    self.refresh_hover(redraw=False)
                else:
                    self.show_values(data[-1])
                self.draw_active()
                mode = '暂停显示（仍在接收）' if self.paused else '实时显示'
                self.view_label.config(text=f'{mode} | {left:.6f}～{end:.6f} 秒 | 窗口 {span*1000:g} ms')
                self.view_dirty = False
            elif self.view_dirty:
                # Never leave the previous segment visible if a narrow window
                # falls inside a receive gap.
                self.visible_data = np.empty((0, len(CH)+1))
                self.model.set_sample(None)
                self.clear_hover(redraw=False)
                for lines in self.cur.values():
                    for line in lines: line.set_data([], [])
                for lb in self.vals.values(): lb.config(text='---')
                self.view_label.config(text='当前时间窗口内暂无接收数据')
                self.draw_active()
                self.view_dirty = False
        except Exception as exc:
            self.lbl.config(text=f'刷新失败: {exc}')
            print(f'刷新失败: {exc}', file=sys.stderr)
        finally:
            if not self.closing:
                self.after(int(self.refresh_box.get()), self.update_ui)

    def clear(self):
        with self.lock:
            self.samples.clear()
            self.latest = None
            self.latest_received = 0.0
        self.pending_command = None
        self.paused = False
        self.paused_samples = []
        self.view_dirty = True
        self.pause_btn.config(text='暂停显示')
        self.visible_data = np.empty((0, len(CH)+1))
        self.model.set_sample(None)
        self.clear_hover(redraw=False)
        for lines in self.cur.values():
            for line in lines: line.set_data([], [])
        for canvas in self.canvases: canvas.draw_idle()
        for lb in self.vals.values(): lb.config(text='---')

    def close(self):
        self.closing = True
        device = self.ser
        self.ser = None
        if device:
            device.close()
        with self.lock:
            if self.csv_f:
                self.csv_f.close()
                self.csv_f = None
        self.destroy()

if __name__ == '__main__':
    App().mainloop()
