#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WL100 底盘串口通信节点 (Production-Ready v3.1 - Modular)

架构：模块化调度中心
  - protocol_tx.py  → 下发协议编码（帧打包、限幅、XOR 校验）
  - protocol_rx.py  → 回传协议解析（滑窗搜帧、三道校验、TYPE 分发）
  - odometry.py     → 里程计积分（死区推算、/odom 消息构建、TF 变换）
  - serial_node.py  → 本文件：串口管理、ROS2 生命周期、线程调度

功能清单：
  [v1.0] 订阅 /cmd_vel → 限幅 → 10 字节帧下发给 STM32
  [v2.0] 参数化配置、断线自动重连、紧急刹车帧
  [v3.0] 后台线程接收 STM32 回传、/odom 发布、看门狗联动
  [v3.1] 模块化重构：协议编解码与积分逻辑独立为可测试模块

遵循规范：/home/nvidia/robot_ws/JETSON_RULES.md
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_ros
import serial
import time
import os
import threading

# 本包内部模块
from wl100_teleop.protocol_tx import build_cmd_frame, STOP_FRAME
from wl100_teleop.protocol_rx import FrameParser, TYPE_ODOM
from wl100_teleop.odometry import OdometryIntegrator


class Wl100SerialNode(Node):
    def __init__(self):
        super().__init__('wl100_serial_node')

        # ==================== ROS2 参数声明 ====================
        # 串口配置（JETSON_RULES 第3条：禁止硬编码）
        self.declare_parameter('port_name', '/dev/ttyCH341USB0')
        self.declare_parameter('baudrate', 115200)

        # 安全限幅
        self.declare_parameter('max_linear_velocity', 0.2)
        self.declare_parameter('max_angular_velocity', 0.2)

        # 断线重连间隔（秒）
        self.declare_parameter('reconnect_interval', 2.0)

        # 回传与里程计参数
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('rx_buffer_max', 256)

        # ==================== 读取参数 ====================
        self.port_name = self.get_parameter('port_name').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.reconnect_interval = self.get_parameter('reconnect_interval').get_parameter_value().double_value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').get_parameter_value().double_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        rx_buffer_max = self.get_parameter('rx_buffer_max').get_parameter_value().integer_value

        # ==================== 串口初始化 ====================
        self.serial_port = None
        self._last_reconnect_time = 0.0
        self._serial_lock = threading.Lock()  # 串口读写互斥锁（Rule 6）
        self._open_serial()

        # ==================== 连接状态 ====================
        self._is_connected = self.serial_port is not None

        # ==================== 日志节流 ====================
        self._last_vx = None
        self._last_vy = None
        self._last_vz = None

        # ==================== 回传解析器（模块化）====================
        self._frame_parser = FrameParser(buffer_max=rx_buffer_max)

        # ==================== 里程计积分器（模块化）====================
        self._odom_integrator = OdometryIntegrator()
        self._last_odom_time = time.monotonic()

        # ==================== 看门狗状态 ====================
        self._last_rx_time = 0.0
        self._watchdog_active = False
        self._watchdog_warned = False

        # ==================== ROS2 发布器 ====================
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ==================== ROS2 订阅 ====================
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 5
        )

        # ==================== ROS2 定时器 ====================
        self._health_timer = self.create_timer(1.0, self._health_check)
        self._odom_timer = self.create_timer(0.02, self._odom_publish_callback)

        # ==================== 后台接收线程 ====================
        self._rx_stop_event = threading.Event()
        self._rx_thread = threading.Thread(
            target=self._rx_thread_loop,
            name='serial_rx_thread',
            daemon=True
        )
        self._rx_thread.start()

        self.get_logger().info(
            f'节点初始化完成 (v3.1 模块化) | '
            f'限速: 线速 ±{self.max_linear_vel} m/s, '
            f'角速 ±{self.max_angular_vel} rad/s | '
            f'看门狗: {self.watchdog_timeout}s | 回传接收线程已启动'
        )

    # ================================================================
    #                        串口管理
    # ================================================================

    def _open_serial(self):
        """尝试打开串口，失败时不崩溃"""
        try:
            with self._serial_lock:
                self.serial_port = serial.Serial(
                    port=self.port_name,
                    baudrate=self.baudrate,
                    timeout=0.02
                )
            self.get_logger().info(
                f'成功打开底盘串口: {self.port_name} ({self.baudrate} bps)'
            )
            self._is_connected = True
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'无法打开串口 {self.port_name}: {str(e)}')
            with self._serial_lock:
                self.serial_port = None
            self._is_connected = False
            return False

    def _try_reconnect(self):
        """非阻塞式自动重连（Rule 2）"""
        now = time.monotonic()
        if now - self._last_reconnect_time < self.reconnect_interval:
            return False

        self._last_reconnect_time = now
        self.get_logger().warn(f'串口断开，正在尝试重连 {self.port_name}...')

        with self._serial_lock:
            if self.serial_port is not None:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
                self.serial_port = None

        return self._open_serial()

    def _is_serial_ready(self):
        """检查串口是否可用"""
        if self.serial_port is not None and self.serial_port.is_open:
            return True
        return self._try_reconnect()

    def _send_stop_frame(self):
        """发送紧急刹车帧"""
        try:
            with self._serial_lock:
                if self.serial_port is not None and self.serial_port.is_open:
                    self.serial_port.write(STOP_FRAME)
        except Exception:
            pass

    # ================================================================
    #                   主动健康监控（1Hz）
    # ================================================================

    def _health_check(self):
        """检测串口设备文件物理存在性（Rule 2：心跳与看门狗）"""
        device_exists = os.path.exists(self.port_name)

        if self._is_connected and not device_exists:
            self._is_connected = False
            self.get_logger().warn(
                f'⚠️  检测到串口设备 {self.port_name} 物理断开！'
            )
            with self._serial_lock:
                if self.serial_port is not None:
                    try:
                        self.serial_port.close()
                    except Exception:
                        pass
                    self.serial_port = None

        elif not self._is_connected and device_exists:
            self.get_logger().info(
                f'检测到设备 {self.port_name} 重新接入，尝试重连...'
            )
            if self._open_serial():
                self._is_connected = True
                self.get_logger().info('✅ 串口自动重连成功！通信已恢复。')

        elif not self._is_connected and not device_exists:
            pass  # 静默等待

    # ================================================================
    #                  /cmd_vel 下发回调
    # ================================================================

    def cmd_vel_callback(self, msg: Twist):
        """接收 Twist → 看门狗检查 → 限幅打包 → 串口发送"""

        if not self._is_serial_ready():
            return

        # 看门狗检查：底盘失联时拒绝下发
        if self._watchdog_active:
            elapsed = time.monotonic() - self._last_rx_time
            if elapsed > self.watchdog_timeout:
                if not self._watchdog_warned:
                    self.get_logger().warn(
                        f'⚠️  看门狗触发: {elapsed:.1f}s 未收到底盘回传，'
                        f'拒绝下发运动指令并发送刹车帧'
                    )
                    self._watchdog_warned = True
                    self._send_stop_frame()
                return

        # 调用 protocol_tx 模块打包（限幅 + 校验 在模块内完成）
        result = build_cmd_frame(
            msg.linear.x, msg.linear.y, msg.angular.z,
            self.max_linear_vel, self.max_angular_vel
        )
        full_frame, vx, vy, vz = result

        if full_frame is None:
            self.get_logger().error('数据打包异常')
            return

        # 智能日志
        self._log_if_changed(msg, vx, vy, vz, full_frame)

        # 串口写入（加锁）
        try:
            with self._serial_lock:
                self.serial_port.write(full_frame)
        except Exception as e:
            self.get_logger().error(f'串口写入异常，标记断线: {str(e)}')
            with self._serial_lock:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
                self.serial_port = None
            self._is_connected = False

    # ================================================================
    #                  后台接收线程
    # ================================================================

    def _rx_thread_loop(self):
        """
        后台接收线程（Rule 15：异步解析架构）
        职责：读串口 → 喂给 FrameParser → 处理解析结果
        """
        while not self._rx_stop_event.is_set():
            # 串口不可用时等待
            if self.serial_port is None or not self._is_connected:
                time.sleep(0.1)
                continue

            # 加锁读取（锁的持有时间极短）
            chunk = b''
            try:
                with self._serial_lock:
                    if self.serial_port is not None and self.serial_port.is_open:
                        chunk = self.serial_port.read(32)
            except serial.SerialException:
                self._is_connected = False
                time.sleep(0.1)
                continue
            except Exception:
                time.sleep(0.01)
                continue

            if not chunk:
                continue

            # 喂入解析器
            self._frame_parser.feed(chunk)

            # 缓冲区溢出保护
            if self._frame_parser.is_overflow:
                self.get_logger().warn(
                    f'接收缓冲区溢出 ({self._frame_parser.buffer_size} 字节)，'
                    f'执行强制清空'
                )
                self._frame_parser.clear()
                continue

            # 执行解析
            results, warnings = self._frame_parser.parse()

            # 处理警告日志
            for w in warnings:
                self.get_logger().warn(f'回传帧: {w}')

            # 处理解析结果
            for result in results:
                if result.msg_type == TYPE_ODOM and result.odom is not None:
                    # 更新里程计积分器的速度输入
                    now = time.monotonic()
                    dt = now - self._last_odom_time
                    self._last_odom_time = now

                    self._odom_integrator.update(
                        result.odom.vx,
                        result.odom.vy,
                        result.odom.vz,
                        dt
                    )

                    # 喂狗
                    self._last_rx_time = time.monotonic()
                    if not self._watchdog_active:
                        self._watchdog_active = True
                        self.get_logger().info(
                            '✅ 收到首帧底盘回传数据，看门狗已激活'
                        )

                    if self._watchdog_warned:
                        self._watchdog_warned = False
                        self.get_logger().info(
                            '✅ 底盘回传恢复，运动指令下发解除冻结'
                        )

    # ================================================================
    #                  里程计发布定时器（50Hz）
    # ================================================================

    def _odom_publish_callback(self):
        """ROS Timer 回调：发布 /odom 与 /tf"""
        if not self._watchdog_active:
            return

        stamp = self.get_clock().now().to_msg()

        # 构建并发布 Odometry 消息
        odom_msg = self._odom_integrator.build_odom_msg(
            stamp, self.odom_frame_id, self.base_frame_id
        )
        self._odom_pub.publish(odom_msg)

        # 广播 TF: odom -> base_footprint
        tf_msg = self._odom_integrator.build_tf_msg(
            stamp, self.odom_frame_id, self.base_frame_id
        )
        self._tf_broadcaster.sendTransform(tf_msg)

    # ================================================================
    #                     日志工具
    # ================================================================

    def _log_if_changed(self, msg, vx, vy, vz, full_frame):
        """智能日志：仅在速度指令变化时打印（Rule 4：防刷屏）"""
        vx_r = round(vx, 3)
        vy_r = round(vy, 3)
        vz_r = round(vz, 3)

        if vx_r != self._last_vx or vy_r != self._last_vy or vz_r != self._last_vz:
            self._last_vx = vx_r
            self._last_vy = vy_r
            self._last_vz = vz_r

            self.get_logger().info(
                f"[原始指令] Vx:{msg.linear.x:.3f} "
                f"Vy:{msg.linear.y:.3f} Vz:{msg.angular.z:.3f}"
            )
            self.get_logger().info(
                f"[限幅下发] Vx:{vx_r:.3f} Vy:{vy_r:.3f} Vz:{vz_r:.3f}"
            )
            hex_frame = ' '.join([f"{b:02X}" for b in full_frame])
            self.get_logger().info(f"[串口帧]   -> {hex_frame}")
            self.get_logger().info("-" * 40)


# ================================================================
#                   主函数与安全退出
# ================================================================

def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = Wl100SerialNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f'发生未预期的致命错误: {e}')
    finally:
        if node is not None:
            try:
                # 优先通知接收线程退出
                if hasattr(node, '_rx_stop_event'):
                    node._rx_stop_event.set()
                if hasattr(node, '_rx_thread') and node._rx_thread.is_alive():
                    node._rx_thread.join(timeout=1.0)

                # 退出阶段使用 print 而非 rclpy logger
                # 原因：Ctrl+C 后 rclpy 可能已 shutdown，logger 不可用
                print('[wl100_serial_node] 收到退出信号，准备停止节点...')

                # 紧急刹车（Rule 1：E-Stop 兜底）
                if hasattr(node, 'serial_port') and node.serial_port and node.serial_port.is_open:
                    try:
                        node.serial_port.write(STOP_FRAME)
                    except Exception:
                        pass
                    print('[wl100_serial_node] ✅ 安全刹车指令已下发，串口准备关闭')
                    node.serial_port.close()
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
