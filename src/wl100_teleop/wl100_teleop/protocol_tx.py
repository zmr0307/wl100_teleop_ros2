#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WL100 下发协议编码模块 (Jetson -> STM32)

帧格式 (10 字节固定帧):
  [A5] [5A] [Vx_H] [Vx_L] [Vy_H] [Vy_L] [Vz_H] [Vz_L] [XOR] [EE]

编码规则:
  - 所有速度值 = 真实值 × 1000，16 位有符号整数，大端序
  - XOR = Byte[0] ~ Byte[7] 累积异或

遵循规范: /home/nvidia/robot_ws/JETSON_RULES.md
"""

import struct

# ==================== 协议常量 ====================
HEADER1 = 0xA5
HEADER2 = 0x5A
TAIL = 0xEE

# 紧急刹车帧 (全 0 速度，XOR = A5^5A^00^00^00^00^00^00 = 0xFF)
STOP_FRAME = bytearray([0xA5, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xEE])


def clamp(value, min_val, max_val):
    """安全限幅"""
    return max(min_val, min(value, max_val))


def safe_to_int16(value):
    """
    安全转换为 int16，防止静默溢出（遵循 JETSON_RULES 第1条）
    将浮点数四舍五入后钳位到 [-32768, 32767] 范围
    """
    return max(-32768, min(32767, int(round(value))))


def build_cmd_frame(vx, vy, vz, max_linear_vel, max_angular_vel):
    """
    将速度指令编码为 10 字节协议帧

    Args:
        vx: X 方向线速度 (m/s)
        vy: Y 方向线速度 (m/s)
        vz: Z 方向角速度 (rad/s)
        max_linear_vel: 线速度限幅绝对值 (m/s)
        max_angular_vel: 角速度限幅绝对值 (rad/s)

    Returns:
        tuple: (full_frame, clamped_vx, clamped_vy, clamped_vz)
               full_frame 为 bytearray(10)，打包失败时为 None
    """
    # 强制软限幅 (JETSON_RULES 第1条：底层不信任原则)
    c_vx = clamp(vx, -max_linear_vel, max_linear_vel)
    c_vy = clamp(vy, -max_linear_vel, max_linear_vel)
    c_vz = clamp(vz, -max_angular_vel, max_angular_vel)

    # 放大 1000 倍 + int16 溢出保护 (JETSON_RULES 第1条)
    vx_int = safe_to_int16(c_vx * 1000.0)
    vy_int = safe_to_int16(c_vy * 1000.0)
    vz_int = safe_to_int16(c_vz * 1000.0)

    # 大端序封包 ('>h' = big-endian signed short)
    try:
        vx_bytes = struct.pack('>h', vx_int)
        vy_bytes = struct.pack('>h', vy_int)
        vz_bytes = struct.pack('>h', vz_int)
    except struct.error:
        return (None, 0.0, 0.0, 0.0)

    # 组装帧头 + 数据体
    frame_head = bytes([HEADER1, HEADER2])
    data_body = vx_bytes + vy_bytes + vz_bytes
    frame_no_tail = frame_head + data_body

    # XOR 校验 (JETSON_RULES 第7条)
    xor_check = 0
    for b in frame_no_tail:
        xor_check ^= b

    # 完整 10 字节帧
    full_frame = bytearray(frame_no_tail)
    full_frame.append(xor_check)
    full_frame.append(TAIL)

    return (full_frame, c_vx, c_vy, c_vz)
