"""上位机公共样式与独立曲线布局；不接触采集或控制数据。"""
import tkinter as tk
from tkinter import ttk
from matplotlib.figure import Figure
from matplotlib.ticker import MaxNLocator

BG = '#f3f6fa'
CARD = '#ffffff'
INK = '#172b45'
MUTED = '#52657a'
BORDER = '#d1dbe7'
ACCENT = '#2563eb'
NAVY = '#ffffff'


def apply_theme(root):
    root.configure(background=BG)
    root.option_add('*Font', ('Microsoft YaHei', 10))
    style = ttk.Style(root)
    style.theme_use('clam')
    style.configure('.', font=('Microsoft YaHei', 10), foreground=INK, background=CARD)
    style.configure('TFrame', background=CARD)
    style.configure('App.TFrame', background=BG)
    style.configure('Card.TFrame', background=CARD, borderwidth=0, relief='flat')
    style.configure('TLabel', background=CARD, foreground=INK)
    style.configure('Muted.TLabel', foreground=MUTED)
    style.configure('Hint.TLabel', background=BG, foreground=MUTED, font=('Microsoft YaHei', 9))
    style.configure('Section.TLabel', foreground=INK, font=('Microsoft YaHei', 9, 'bold'))
    style.configure('TButton', padding=(7, 3), background=CARD, foreground=INK,
                    bordercolor=BORDER, lightcolor=CARD, darkcolor=CARD, focuscolor=CARD)
    style.map('TButton', background=[('pressed', '#d1dbe7'), ('active', '#e7eef7')])
    style.configure('Primary.TButton', background=ACCENT, foreground='white', bordercolor=ACCENT,
                    lightcolor=ACCENT, darkcolor=ACCENT, font=('Microsoft YaHei', 9, 'bold'))
    style.map('Primary.TButton', background=[('pressed', '#1d4ed8'), ('active', '#3b82f6')],
              foreground=[('disabled', '#536780'), ('!disabled', 'white')])
    # 发送目标文字在禁用状态也保持白色；用较淡的蓝色表示不可用。
    style.configure('Send.Primary.TButton', background=ACCENT, foreground='white',
                    bordercolor=ACCENT, lightcolor=ACCENT, darkcolor=ACCENT)
    style.map('Send.Primary.TButton',
              foreground=[('disabled', 'white'), ('!disabled', 'white')],
              background=[('disabled', '#829bc0'), ('pressed', '#1d4ed8'), ('active', '#3b82f6')])
    style.configure('TCombobox', padding=(4, 2), fieldbackground=CARD, background='#d1dbe7',
                    bordercolor=BORDER, lightcolor=CARD, darkcolor=CARD, arrowcolor=INK)
    style.map('TCombobox', fieldbackground=[('readonly', CARD)], foreground=[('readonly', INK)],
              selectbackground=[('readonly', CARD)], selectforeground=[('readonly', INK)])
    style.configure('TEntry', padding=(4, 2), fieldbackground=CARD, foreground=INK, insertcolor=INK, bordercolor=BORDER)
    style.configure('TCheckbutton', background=CARD, foreground=INK, padding=(3, 3), indicatorcolor='#e7eef7', indicatorbackground='#e7eef7', indicatorforeground=INK, focuscolor=CARD)
    style.map('TCheckbutton', indicatorcolor=[('selected', ACCENT), ('active', '#d1dbe7')])
    style.map('TCheckbutton', background=[('active', CARD)])
    root.option_add('*TCombobox*Listbox.background', CARD)
    root.option_add('*TCombobox*Listbox.foreground', INK)
    root.option_add('*TCombobox*Listbox.selectBackground', '#2563eb')
    root.option_add('*TCombobox*Listbox.selectForeground', 'white')
    style.configure('TNotebook', background=BG, borderwidth=0, tabmargins=(0, 0, 0, 5))
    style.configure('TNotebook.Tab', background='#edf2f8', foreground=MUTED, padding=(9, 4),
                    font=('Microsoft YaHei', 9, 'bold'), borderwidth=0)
    style.map('TNotebook.Tab', background=[('selected', CARD), ('active', '#dce8f7')],
              foreground=[('selected', INK)])
    style.configure('Horizontal.TScale', background=CARD, troughcolor='#d1dbe7', sliderlength=16)
    style.configure('Vertical.TScrollbar', background='#a7b7ca', troughcolor=CARD,
                    borderwidth=0, arrowcolor=MUTED)
    return style


def create_scope_figure(plots, time_formatter=None):
    """每项独立Y轴，仅底部显示时间刻度，为五行图保留有效绘图高度。"""
    figure = Figure(figsize=(9.2, 6.4), dpi=100, facecolor=CARD)
    axes = []
    for index, (title, channels) in enumerate(plots):
        ax = figure.add_subplot(len(plots), 1, index+1, sharex=axes[0] if axes else None)
        ax.set_facecolor(CARD)
        ax.set_title(title, loc='left', fontsize=10, fontweight='bold', color=INK, pad=7)
        ax.grid(True, color='#e0e7ef', linewidth=.7)
        ax.set_axisbelow(True)
        for edge in ('top', 'right'): ax.spines[edge].set_visible(False)
        for edge in ('left', 'bottom'): ax.spines[edge].set_color(BORDER)
        ax.tick_params(axis='both', labelsize=9, colors=MUTED, length=0, pad=4)
        ax.tick_params(axis='x', labelbottom=index == len(plots)-1)
        ax.yaxis.set_major_locator(MaxNLocator(nbins=3, min_n_ticks=3))
        ax.ticklabel_format(axis='y', useOffset=False)
        if time_formatter:
            from matplotlib.ticker import FuncFormatter
            ax.xaxis.set_major_formatter(FuncFormatter(time_formatter))
        if index == len(plots)-1: ax.set_xlabel('时间（秒）', fontsize=9, color=MUTED)
        axes.append(ax)
    figure.subplots_adjust(left=.09, right=.98, bottom=.08, top=.95, hspace=.65)
    return figure, axes


def scroll_content(page):
    canvas = tk.Canvas(page, background=CARD, highlightthickness=0, width=278)
    scrollbar = ttk.Scrollbar(page, orient='vertical', command=canvas.yview)
    canvas.configure(yscrollcommand=scrollbar.set)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    content = ttk.Frame(canvas)
    item = canvas.create_window((0, 0), window=content, anchor='nw')
    content.bind('<Configure>', lambda event: canvas.configure(scrollregion=canvas.bbox('all')))
    canvas.bind('<Configure>', lambda event: canvas.itemconfigure(item, width=event.width))
    def wheel(event):
        if event.delta:
            canvas.yview_scroll(-1 if event.delta > 0 else 1, 'units')
        return 'break'
    return content, wheel


def build_value_panel(panel, channels, colors):
    title = ttk.Label(panel, text='当前显示时刻数值', style='Hint.TLabel', font=('Microsoft YaHei', 11, 'bold'))
    title.pack(anchor='w', pady=(0, 9))
    tabs = ttk.Notebook(panel); tabs.pack(fill=tk.BOTH, expand=True)
    control_page, stop_page = ttk.Frame(tabs), ttk.Frame(tabs)
    tabs.add(control_page, text='控制数值'); tabs.add(stop_page, text='停机诊断')
    control, control_wheel = scroll_content(control_page)
    stop, stop_wheel = scroll_content(stop_page)
    values = {}
    def value(parent, name, size=13, fixed_width=None):
        widget = tk.Label(parent, text='---', fg=colors[channels.index(name)], bg=CARD,
                          font=('Consolas', 9 if name=='指令状态' else size, 'bold'), anchor='e',
                          wraplength=185 if name=='指令状态' else 0, justify='right',
                          width=fixed_width or 0, relief='flat', borderwidth=0, highlightthickness=0)
        values[name] = widget
        return widget
    def section(caption, names, tiles=False):
        outer = ttk.Frame(control); outer.pack(fill=tk.X, padx=9, pady=(10, 3))
        ttk.Label(outer, text=caption, style='Section.TLabel').pack(anchor='w', pady=(0, 5))
        body = tk.Frame(outer, bg=CARD, highlightthickness=0, bd=0)
        body.pack(fill=tk.X)
        if tiles:
            for column, name in enumerate(names):
                body.columnconfigure(column, weight=1, uniform='pid')
                tile = tk.Frame(body, bg=CARD); tile.grid(row=0, column=column, sticky='nsew', padx=4, pady=7)
                tk.Label(tile, text=name[-1], font=('Consolas', 9, 'bold'), fg=MUTED, bg=CARD).pack(anchor='w')
                value(tile, name, 13).pack(fill=tk.X, pady=(3, 0))
        else:
            for name in names:
                row = tk.Frame(body, bg=CARD); row.pack(fill=tk.X, padx=9, pady=3)
                tk.Label(row, text='外环饱和标志' if name=='外环限幅' else name, bg=CARD, fg=MUTED, font=('Microsoft YaHei', 10)).pack(side=tk.LEFT)
                value(row, name, fixed_width=17 if name=='指令状态' else 10).pack(side=tk.RIGHT)
    section('摆杆与驱动', [channels[i] for i in (0, 1, 2, 4)])
    section('旋臂位置', [channels[i] for i in (3, 8, 9)])
    if '指令终点pos' in channels:
        section('串口目标控制', ['指令终点pos', '旋臂轨迹目标', '指令序号', '指令状态'])
    section('外环限幅诊断 · ADC上限', ['实时限幅ADC', '限幅阶段'])
    section('内环 PID · PWM指令', [channels[i] for i in (5, 6, 7)], True)
    section('外环 PID · ADC计数', ['外环P', '外环I', '外环D'], True)
    section('运行状态', [channels[i] for i in (12, 13)])
    for name in channels[14:23]:
        row = ttk.Frame(stop); row.pack(fill=tk.X, padx=9, pady=6)
        ttk.Label(row, text=name, style='Muted.TLabel').pack(anchor='w')
        value(row, name).pack(fill=tk.X, pady=(2, 0))
    def bind_wheel(widget, callback):
        widget.bind('<MouseWheel>', callback, add='+')
        for child in widget.winfo_children(): bind_wheel(child, callback)
    bind_wheel(control, control_wheel); bind_wheel(stop, stop_wheel)
    return title, values
