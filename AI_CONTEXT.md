# WL100_ROS2 (Jetson Xavier NX) 项目上下文与状态记录 (AI_CONTEXT)

> **核心提示给下一次对话的 AI**：
> 请在接手本工作区（`/home/nvidia/robot_ws`）的任何任务前，务必优先阅读本文件，以确保环境配置、硬件连线、协议标准和当前进度的上下文绝对同步，严禁基于错误上下文修改已验证无误的核心代码。

## 1. 硬件架构与连线状态

- **上位机（主控）**：NVIDIA Jetson Xavier NX (16GB)
- **下位机（底盘控制）**：STM32F407
- **底盘硬件**：WL100 全向轮底盘（MANDA）
- **通信物理层**：
  - Jetson 与 STM32：**UART 串口通信**（非 SocketCAN！！不要在 Jetson 跑 CAN 节点）。
  - 接线方式：使用 USB 转 TTL 模块（Jetson 端识别为 `/dev/ttyUSB0`）。
    - Jetson TX  `->` STM32 RX (PB11)
    - Jetson RX  `->` STM32 TX (PB10)
    - GND `->` GND
  - *(备选串口)*：如果不用 USB 转串口模块，Jetson 主板自带的 UART 引脚对应 `/dev/ttyTHS1`。

## 2. 软件环境与依赖

- **操作系统**：Ubuntu 22.04 (Jammy)
- **ROS2 版本**：Humble Hawksbill (非 Foxy)
- **核心依赖**：
  - `rclpy` (ROS2 Python 客户端库)
  - `geometry_msgs` (标准消息库)
  - `pyserial` (Python 串口通信库，当前已全局安装 v3.5)
  - `ros-humble-teleop-twist-keyboard` (用于键盘控制测试)

## 3. 核心功能包信息

- **工作区路径**：`/home/nvidia/robot_ws`
- **控制功能包**：`wl100_teleop`
- **包类型**：`ament_python`
- **核心节点文件**：`src/wl100_teleop/wl100_teleop/serial_node.py`
- **执行入口**：`ros2 run wl100_teleop serial_node` （支持动态传参：`--ros-args -p port_name:=/dev/ttyUSB0 -p baudrate:=115200`）

## 4. 通信协议标准（Jetson -> STM32）

- **波特率**：`115200 bps`
- **订阅话题**：`/cmd_vel` (`geometry_msgs/msg/Twist`)
- **速度安全锁**：代码层面实施了严格的**软限幅 `±0.2 m/s` (平移) 及 `±0.2 rad/s` (旋转)**。
- **自定义 10 字节串口协议帧格式**：
  - 格式：`[帧头1] [帧头2] [Vx_H] [Vx_L] [Vy_H] [Vy_L] [Vz_H] [Vz_L] [XOR校验] [帧尾]`
  - `Byte 0`: `0xA5`
  - `Byte 1`: `0x5A`
  - `Byte 2~7`: Vx, Vy, Vz，单位放大 1000 倍，使用 **大端序 (Big Endian) 16位有符号整型 (int16)**。
  - `Byte 8`: XOR 校验和，计算范围为 `Byte 0` 到 `Byte 7` 的累积异或。
  - `Byte 9`: `0xEE`
  - *紧急刹车帧（全0速度）已硬编码并在退出时下发：* `0xA5 0x5A 0x00 0x00 0x00 0x00 0x00 0x00 0xFF 0xEE`

## 5. 当前工程状态与已知里程碑 (2026-03-06)

- **代码健壮性**：`serial_node.py` 已完成至少 6 轮严谨的审查与重构。
  - 已彻底解决 `rclpy` 退出时的 `rosout context invalid` 问题（使用了最严格的 `try...except...finally` 最佳实践隔离日志与节点清理过程）。
  - 已解决 `pyserial` 偶尔丢失引发的节点初始化中断问题（加入了 `hasattr` 安全锁）。
  - `colcon build` 编译完美通过。
- **下一步行动点（TODO）**：
  - [ ] **连线实测**：将 Jetson 与 STM32 通过 USB-TTL 正式物理连接。
  - [ ] **启动测试**：分别运行 `ros2 run wl100_teleop serial_node` 和键盘节点，观察小车底盘是否正常接收 10 字节指令并以安全限幅低速移动。
  - [ ] **开发回传回路（未来扩展）**：当前节点仅支持向 STM32 **发送**速度指令，未来需要增加解析 STM32 回传给 Jetson 的传感器数据（如里程计、电池状态），并发布为对应的 ROS2 Topic。
