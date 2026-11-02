"""实测数据驱动的二维姿态显示；不包含动力学仿真，不发送任何控制指令。"""
from dataclasses import dataclass
import json
import math

from matplotlib.patches import Circle, FancyBboxPatch


@dataclass(frozen=True)
class ViewCalibration:
    encoder_counts_per_turn: float = 1080.0
    adc_counts_per_turn: float = 4096.0
    center_adc: float = 1954.0
    blind_low_adc: float = 100.0
    blind_high_adc: float = 3990.0

    def __post_init__(self):
        for value in (self.encoder_counts_per_turn, self.adc_counts_per_turn):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('每圈计数必须是大于0的有限数值')
        if not (math.isfinite(self.center_adc) and 0 <= self.center_adc <= 4095):
            raise ValueError('中心ADC必须在0～4095之间')
        if not (0 <= self.blind_low_adc < self.center_adc < self.blind_high_adc <= 4095):
            raise ValueError('盲区边界须满足0≤下界<中心ADC<上界≤4095')

    @classmethod
    def load(cls, path):
        if not path.exists():
            return cls()
        values = json.loads(path.read_text(encoding='utf-8-sig'))
        # 兼容只有两项换算系数的旧版配置。
        return cls(**{key: float(values.get(key, field.default))
                      for key, field in cls.__dataclass_fields__.items()})

    def save(self, path):
        path.write_text(json.dumps(self.__dict__, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


@dataclass(frozen=True)
class ModelSample:
    time_s: float
    theta_adc: float
    position_counts: float
    ref_adc: float = math.nan
    position_error_counts: float = math.nan
    speed_counts_per_10ms: float = math.nan
    duty: float = math.nan
    enabled: float = math.nan


@dataclass(frozen=True)
class ModelPose:
    sample: ModelSample
    arm_degrees: float
    arm_total_degrees: float
    pendulum_degrees: float
    speed_degrees_s: float
    arm_end: tuple
    pendulum_end_side: tuple
    pendulum_end_top: tuple
    target_arm_end: tuple | None
    target_pendulum_end_side: tuple | None
    reconstructed_adc: float
    in_bottom_blind_zone: bool


def calculate_pose(sample, calibration):
    """theta已由FPGA归零；编码器为累计计数。正旋臂方向从上往下看为顺时针。"""
    if not all(math.isfinite(v) for v in (sample.time_s, sample.theta_adc, sample.position_counts)):
        return None
    alpha = (sample.position_counts % calibration.encoder_counts_per_turn)/calibration.encoder_counts_per_turn*math.tau
    display_alpha = alpha + math.pi  # 俯视图零位朝下，正计数保持顺时针。
    # 当前FPGA采用INVERT=1：theta=中心ADC-滤波后ADC。串口未传实时原始ADC，
    # 此处还原的只是滤波后ADC；停机时锁存的ADC不可用于实时姿态。
    actual_adc = calibration.center_adc - sample.theta_adc
    blind = actual_adc <= calibration.blind_low_adc or actual_adc >= calibration.blind_high_adc
    beta = sample.theta_adc/calibration.adc_counts_per_turn*math.tau
    arm = (math.sin(display_alpha), math.cos(display_alpha))
    side = (0.0, -1.0) if blind else (math.sin(beta), math.cos(beta))
    # 俯视观察下，正摆角投向旋臂的逆时针切向；侧视图的左右方向保持原定义。
    # 长度只作等长示意，不用于空间距离测量。
    top = (arm[0]-side[0]*math.cos(display_alpha), arm[1]+side[0]*math.sin(display_alpha))
    target_arm = None
    if math.isfinite(sample.position_error_counts):
        target = ((sample.position_counts-sample.position_error_counts) % calibration.encoder_counts_per_turn)
        target = target/calibration.encoder_counts_per_turn*math.tau + math.pi
        target_arm = (math.sin(target), math.cos(target))
    target_side = None
    if math.isfinite(sample.ref_adc):
        ref = sample.ref_adc/calibration.adc_counts_per_turn*math.tau
        target_side = (math.sin(ref), math.cos(ref))
    return ModelPose(sample, math.degrees(alpha), sample.position_counts/calibration.encoder_counts_per_turn*360,
                     180.0 if blind else (math.degrees(beta)+180) % 360-180,
                     sample.speed_counts_per_10ms/calibration.encoder_counts_per_turn*360/0.01,
                     arm, side, top, target_arm, target_side, actual_adc, blind)


def number(value, digits=1):
    return f'{value:.{digits}f}' if math.isfinite(value) else '--'


class PendulumModelView:
    """固定图元只更新坐标，接收线程无需参与绘制；可用Agg后端离线检查。"""
    def __init__(self, figure, calibration=None, compact=False):
        self.figure = figure
        self.compact = compact
        self.calibration = calibration or ViewCalibration()
        self.last_sample = None
        self.mode = '等待数据'
        self.state = None
        figure.set_facecolor('#f4f7fb')
        for x, y, w, h in [(0.035, .335, .445, .55), (.515, .335, .45, .55),
                            (.04, .115, .44, .205), (.52, .115, .445, .205)]:
            figure.add_artist(FancyBboxPatch((x, y), w, h, transform=figure.transFigure,
                boxstyle='round,pad=0.008,rounding_size=0.012', facecolor='white',
                edgecolor='#dce4ee', linewidth=.8, zorder=-1))
        self.heading = figure.text(.05, .96, '倒立摆二维姿态', fontsize=15, weight='bold', color='#172b45')
        self.status = figure.text(.05, .91, '等待串口数据', fontsize=10, color='#596b80')
        self.top = figure.add_axes([.045, .34, .43, .48])
        self.side = figure.add_axes([.525, .34, .43, .48])
        for ax in (self.top, self.side):
            ax.set_aspect('equal', adjustable='box')
            ax.set_axis_off()
        self.top.set(xlim=(-2.05, 2.05), ylim=(-2.05, 2.05))
        self.side.set(xlim=(-1.3, 1.3), ylim=(-1.3, 1.3))
        self.top.set_title('旋臂俯视图', fontsize=12, pad=14, color='#172b45')
        self.side.set_title('摆杆侧视图（随旋臂转动的局部视角）', fontsize=11, pad=14, color='#172b45')
        self.top.add_patch(Circle((0, 0), 1, fill=False, color='#c9d3df', ls='--', lw=1.3))
        for x, y, label in [(0, 1.24, '180°'), (1.25, 0, '270°'), (0, -1.25, '0°'), (-1.30, 0, '90°')]:
            self.top.text(x, y, label, ha='center', va='center', fontsize=9, color='#77869a')
        self.top.plot(0, 0, 'o', color='#172b45', ms=11, zorder=5)
        self.top.text(0, -1.85, '编码器零点向下 · 正计数顺时针', ha='center', fontsize=9, color='#596b80')
        self.side.plot([0, 0], [-1.1, 1.1], '--', color='#c9d3df', lw=1)
        self.side.plot([-.23, .23], [-.09, -.09], color='#172b45', lw=5)
        self.side.text(0, 1.18, '竖直向上 0°', ha='center', fontsize=9, color='#77869a')
        self.side.text(0, -1.22, '正倾角向右 · 负倾角向左', ha='center', fontsize=9, color='#596b80')
        self.arm, = self.top.plot([], [], color='#2979d5', lw=7, solid_capstyle='round', zorder=3)
        self.projection, = self.top.plot([], [], color='#00a879', lw=4, alpha=.85, solid_capstyle='round', zorder=4)
        self.target_arm, = self.top.plot([], [], '--', color='#ed9a24', lw=2, zorder=2)
        self.rod, = self.side.plot([], [], color='#00a879', lw=7, solid_capstyle='round', zorder=3)
        self.tip, = self.side.plot([], [], 'o', color='#00a879', ms=11, zorder=4)
        self.target_rod, = self.side.plot([], [], '--', color='#ed9a24', lw=2, zorder=2)
        self.side.plot(0, 0, 'o', color='#2979d5', ms=10, zorder=5)
        self.artists = [self.arm, self.projection, self.target_arm, self.rod, self.tip, self.target_rod]
        self.readouts = []
        for x in (.055, .535):
            for y in (.285, .235, .185, .135):
                self.readouts.append(figure.text(x, y, '--', fontsize=11, color='#172b45'))
        figure.text(.05, .073, '蓝色：实际旋臂   绿色：实际摆杆 / 水平投影   橙色虚线：控制目标', fontsize=9, color='#596b80')
        self.scale_note = figure.text(.05, .030, '', fontsize=8, color='#77869a')
        if compact:
            # 总览右上区域并排显示两幅姿态图，保持坐标与计算方式不变。
            figure.set_facecolor('#ffffff')
            for patch in figure.artists: patch.set_visible(False)
            self.heading.set_visible(False)
            self.status.set_position((.04, .96)); self.status.set_fontsize(9)
            self.status.set_color('#52657a')
            self.top.set_position([.02, .24, .47, .61])
            self.side.set_position([.52, .24, .46, .61])
            self.top.set_title('旋臂俯视', fontsize=9, pad=2)
            self.side.set_title('摆杆侧视', fontsize=9, pad=2)
            for ax in (self.top, self.side):
                ax.title.set_color('#172b45')
            for text in self.top.texts + self.side.texts:
                text.set_fontsize(8); text.set_color('#52657a')
            self.top.texts[-1].set_visible(False)
            self.side.texts[-1].set_visible(False)
            for text in figure.texts:
                if text not in [self.status, *self.readouts]: text.set_visible(False)
            for index, text in enumerate(self.readouts):
                text.set_visible(index in (0, 4))
                text.set_fontsize(9)
                text.set_color('#172b45')
            self.readouts[0].set_position((.04, .10))
            self.readouts[4].set_position((.52, .10))
        self.set_sample(None)

    def set_calibration(self, calibration):
        self.calibration = calibration
        self.set_sample(self.last_sample, self.mode)

    def set_sample(self, sample, mode='实时数据'):
        self.last_sample, self.mode = sample, mode
        self.state = calculate_pose(sample, self.calibration) if sample else None
        c = self.calibration
        self.scale_note.set_text(f'旋臂 {c.encoder_counts_per_turn:g} 计数/圈；摆杆 {c.adc_counts_per_turn:g} ADC/圈 · 中心 {c.center_adc:g} · ADC≤{c.blind_low_adc:g}或≥{c.blind_high_adc:g}显示向下 · 长度作示意')
        for artist in self.artists:
            artist.set_visible(self.state is not None)
        if self.state is None:
            self.status.set_text('等待串口数据 / 当前窗口无有效姿态')
            for text in self.readouts: text.set_text('--')
            return
        p, s = self.state, sample
        pose_note = '底部盲区：默认竖直向下（非实测角度）' if p.in_bottom_blind_zone else '有效区域：按中心ADC与实测ADC之差显示'
        self.status.set_text(f'{mode}  |  {s.time_s:.3f} s' if self.compact else
                             f'{mode}  |  接收时刻 {s.time_s:.3f} s  |  {pose_note}')
        self.arm.set_data([0, p.arm_end[0]], [0, p.arm_end[1]])
        self.projection.set_data([p.arm_end[0], p.pendulum_end_top[0]], [p.arm_end[1], p.pendulum_end_top[1]])
        self.rod.set_data([0, p.pendulum_end_side[0]], [0, p.pendulum_end_side[1]])
        self.tip.set_data([p.pendulum_end_side[0]], [p.pendulum_end_side[1]])
        for artist, endpoint in ((self.target_arm, p.target_arm_end), (self.target_rod, p.target_pendulum_end_side)):
            artist.set_visible(endpoint is not None)
            if endpoint is not None: artist.set_data([0, endpoint[0]], [0, endpoint[1]])
        target = s.position_counts-s.position_error_counts
        enabled = '使能' if s.enabled == 1 else '未使能' if s.enabled == 0 else '--'
        texts = [f'旋臂圈内角度  {p.arm_degrees:.1f}°',
                 f'累计位置  {number(s.position_counts, 0)} 计数 / {p.arm_total_degrees:.1f}°',
                 f'旋臂目标约  {number(target)} 计数',
                 f'位置误差  {number(s.position_error_counts)} 计数',
                 f'摆杆倾角  {p.pendulum_degrees:+.2f}°',
                 f'theta {number(s.theta_adc, 0)} / 还原ADC {number(p.reconstructed_adc, 0)}',
                 f'旋臂速度  {number(p.speed_degrees_s)}°/s',
                 f'PWM指令  {number(s.duty, 0)} / 控制{enabled}']
        if self.compact and p.in_bottom_blind_zone:
            texts[4] = '摆杆：底部盲区（显示向下）'
        for artist, text in zip(self.readouts, texts): artist.set_text(text)
