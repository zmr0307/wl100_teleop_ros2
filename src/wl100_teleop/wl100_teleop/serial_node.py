#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

class Wl100SerialNode(Node):
    def __init__(self):
        super().__init__('wl100_serial_node')
        
        # 声明串口与波特率的 ROS2 参数，并赋予默认值，方便后续修改
        self.declare_parameter('port_name', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port_name = self.get_parameter('port_name').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # 初始化串口
        try:
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=baudrate,
                timeout=0.1
            )
            self.get_logger().info(f'成功打开底盘串口: {port_name} ({baudrate} bps)')
        except serial.SerialException as e:
            self.get_logger().error(f'无法打开串口 {port_name}，请检查设备或权限: {str(e)}')
            self.serial_port = None

        # 订阅/cmd_vel，队列长度设为5防阻塞
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            5
        )
        self.subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg: Twist):
        """
        回调函数，接收 Twist 后打包发出
        """
        if self.serial_port is None or not self.serial_port.is_open:
            return

        # 获取速度并强制软限幅 ±0.2 （安全降速要求）
        vx = self.clamp(msg.linear.x, -0.2, 0.2)
        vy = self.clamp(msg.linear.y, -0.2, 0.2)
        vz = self.clamp(msg.angular.z, -0.2, 0.2)

        # 按照红线要求转化：单位放大1000倍，变整型
        # int(round()) 防止浮点数乘法时出现精度损失问题
        vx_int = int(round(vx * 1000.0))
        vy_int = int(round(vy * 1000.0))
        vz_int = int(round(vz * 1000.0))

        # 封包 - 大端序 struct '>h' (h代表有符号短整型 short，即 int16)
        try:
            vx_bytes = struct.pack('>h', vx_int)
            vy_bytes = struct.pack('>h', vy_int)
            vz_bytes = struct.pack('>h', vz_int)
        except Exception as e:
            self.get_logger().error(f'数据打包异常: {str(e)}')
            return

        # 组合帧前8个字节 (不含最后的校验和尾)
        frame_head = bytes([0xA5, 0x5A])
        data_body = vx_bytes + vy_bytes + vz_bytes
        frame_except_tail = frame_head + data_body

        # 计算 XOR 校验
        xor_check = 0
        for byte in frame_except_tail:
            xor_check ^= byte
        
        # 组装完整 10 字节帧
        full_frame = bytearray(frame_except_tail)
        full_frame.append(xor_check)     # 校验和
        full_frame.append(0xEE)          # 帧尾

        # 向串口写入
        try:
            self.serial_port.write(full_frame)
            # 这行注释掉可防刷屏，调试时可打开：
            # self.get_logger().debug(f'Sent: {full_frame.hex().upper()}')
        except Exception as e:
            self.get_logger().error(f'串口写入异常: {str(e)}')

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))


def main(args=None):
    rclpy.init(args=args)

    # 提前声明，防止初始化失败时 finally 里 node 未定义
    node = None
    try:
        node = Wl100SerialNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # rclpy 最佳实践：捕获后 pass，真正清理交给 finally，避免 rosout 上下文争夺
        pass
    except Exception as e:
        # 捕捉所有非中断异常，防止程序静默崩溃
        if node is not None:
            node.get_logger().error(f'发生无法捕获的错误: {e}')
    finally:
        if node is not None:
            try:
                node.get_logger().info('收到退出信号，准备停止节点...')
                # hasattr 安全锁：防止 __init__ 中途失败导致 serial_port 变量根本不存在
                if hasattr(node, 'serial_port') and node.serial_port and node.serial_port.is_open:
                    # 安全离线：下发全 0 速度，让底盘停车
                    stop_frame = bytearray([0xA5, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xEE])
                    try:
                        node.serial_port.write(stop_frame)
                    except Exception:
                        pass
                    node.get_logger().info('安全刹车指令已下发，串口准备关闭')
                    node.serial_port.close()
            except Exception:
                # 即使日志或串口操作失败，也绝不能阻止节点被销毁
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
