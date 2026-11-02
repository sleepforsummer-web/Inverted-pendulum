"""上位机位置命令编码。仅在用户点击发送时使用，不写PID参数。"""
import math
import struct

COUNTS_PER_REV = 1080
MAX_SPEED_DEGREES_S = 360
COMMAND_STATUS = {
    0: '尚无位置指令', 1: '已接受，轨迹移动中', 2: '拒绝：尚未进入PID平衡',
    3: '拒绝：已有运动或正在等待到位', 4: '拒绝：目标或速度超出范围',
    5: '拒绝：启动摆角或旋臂速度过大', 7: '目标轨迹完成，保持目标',
    8: '运动已因停机或失稳取消',
}

def encode_position_command(angle_degrees, speed_degrees_s, sequence):
    angle, speed = float(angle_degrees), float(speed_degrees_s)
    if not math.isfinite(angle) or not math.isfinite(speed):
        raise ValueError('角度和速度必须是有限数值')
    if not 5 <= speed <= MAX_SPEED_DEGREES_S:
        raise ValueError(f'移动速度请输入5～{MAX_SPEED_DEGREES_S}度/秒')
    goal_q8 = round(angle * COUNTS_PER_REV * 256 / 360)
    step_q8 = round(speed * COUNTS_PER_REV * 256 / 360 / 100)
    if not -16777216 <= goal_q8 <= 16776960:
        raise ValueError('目标位置超出编码器可表示范围（-65536～65535计数）')
    payload = struct.pack('<BBiH', 1, sequence & 255, goal_q8, step_q8)
    check = 0
    for byte in payload:
        check ^= byte
    return b'\xa5\x5a' + payload + bytes((check,))
