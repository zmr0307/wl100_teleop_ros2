#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WL100 回传协议解析模块 (STM32 -> Jetson)

帧格式 (11 字节固定帧):
  [AA] [55] [TYPE] [Vx_H] [Vx_L] [Vy_H] [Vy_L] [Vz_H] [Vz_L] [XOR] [EE]

解析策略:
  - 滑窗搜帧：在字节流中滑动查找合法帧头 [AA 55]
  - 三道校验：帧头 + 帧尾 + XOR 异或校验
  - 校验失败仅丢弃，不崩溃 (JETSON_RULES 第7条)

遵循规范: /home/nvidia/robot_ws/JETSON_RULES.md
"""

import struct
from dataclasses import dataclass
from typing import List, Optional

# ==================== 协议常量 ====================
HEADER1 = 0xAA
HEADER2 = 0x55
TAIL = 0xEE
FRAME_LEN = 11

# 消息类型定义
TYPE_ODOM = 0x01      # 里程计速度
TYPE_BATTERY = 0x02   # 电池状态（预留）


# ==================== 数据容器 ====================

@dataclass
class OdomData:
    """里程计速度数据（已还原为真实物理量）"""
    vx: float   # X 方向线速度 (m/s)
    vy: float   # Y 方向线速度 (m/s)
    vz: float   # Z 方向角速度 (rad/s)


@dataclass
class ParseResult:
    """单帧解析结果"""
    msg_type: int
    odom: Optional[OdomData] = None
    # battery: Optional[BatteryData] = None  # 后续迭代扩展


# ==================== 帧解析器 ====================

class FrameParser:
    """
    滑窗搜帧解析器

    使用方法:
        parser = FrameParser(buffer_max=256)
        parser.feed(raw_bytes)
        results, warnings = parser.parse()

    设计原则:
        - 无状态依赖：可以在任意时刻喂入任意长度的字节流
        - 缓冲区保护：超过上限自动清空 (JETSON_RULES 第15条)
        - 容错设计：校验失败仅丢弃并记录，不抛异常
    """

    def __init__(self, buffer_max=256):
        self._buffer = bytearray()
        self._buffer_max = buffer_max

    def feed(self, data: bytes):
        """向缓冲区追加新接收的原始字节"""
        self._buffer.extend(data)

    @property
    def is_overflow(self) -> bool:
        """检查缓冲区是否溢出"""
        return len(self._buffer) > self._buffer_max

    @property
    def buffer_size(self) -> int:
        """当前缓冲区大小"""
        return len(self._buffer)

    def clear(self):
        """强制清空缓冲区"""
        self._buffer.clear()

    def parse(self) -> tuple:
        """
        执行滑窗搜帧 + 三道校验

        Returns:
            tuple: (results: List[ParseResult], warnings: List[str])
        """
        results: List[ParseResult] = []
        warnings: List[str] = []

        while len(self._buffer) >= FRAME_LEN:
            # 第一道防线：帧头 [0xAA, 0x55]
            if self._buffer[0] != HEADER1 or self._buffer[1] != HEADER2:
                del self._buffer[0]
                continue

            # 第二道防线：帧尾 [0xEE]
            if self._buffer[FRAME_LEN - 1] != TAIL:
                del self._buffer[0]
                continue

            # 第三道防线：XOR 校验（对 [0]~[8] 共 9 字节异或，结果应等于 [9]）
            cal_xor = 0
            for i in range(9):
                cal_xor ^= self._buffer[i]

            if cal_xor != self._buffer[9]:
                warnings.append(
                    f'XOR 校验失败: 计算={cal_xor:#04x}, '
                    f'收到={self._buffer[9]:#04x}'
                )
                del self._buffer[0]
                continue

            # ✅ 三道校验全部通过，提取数据
            frame = bytes(self._buffer[:FRAME_LEN])
            del self._buffer[:FRAME_LEN]

            msg_type = frame[2]
            result = ParseResult(msg_type=msg_type)

            if msg_type == TYPE_ODOM:
                vx_raw = struct.unpack('>h', frame[3:5])[0]
                vy_raw = struct.unpack('>h', frame[5:7])[0]
                vz_raw = struct.unpack('>h', frame[7:9])[0]

                result.odom = OdomData(
                    vx=vx_raw / 1000.0,
                    vy=vy_raw / 1000.0,
                    vz=vz_raw / 1000.0
                )

            elif msg_type == TYPE_BATTERY:
                # TYPE=0x02: 电池状态（预留接口，后续迭代实现）
                pass

            else:
                warnings.append(f'未知消息类型: 0x{msg_type:02X}')

            results.append(result)

        return results, warnings
