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
  - `nav_msgs` (里程计 Odometry 消息，v3.0 新增)
  - `tf2_ros` (TF 变换广播，v3.0 新增)
  - `pyserial` (Python 串口通信库，当前已全局安装 v3.5)
  - `ros-humble-teleop-twist-keyboard` (用于键盘控制测试)

## 3. 核心功能包信息

- **工作区路径**：`/home/nvidia/robot_ws`
- **控制功能包**：`wl100_teleop`
- **包类型**：`ament_python`
- **核心节点文件**：`src/wl100_teleop/wl100_teleop/serial_node.py`
- **执行入口**：`ros2 run wl100_teleop serial_node` （支持动态传参：`--ros-args -p port_name:=/dev/ttyUSB0 -p baudrate:=115200`）

## 4. 通信协议标准

### 4.1 Jetson -> STM32 下发协议（10 字节固定帧）

- **波特率**：`115200 bps`
- **订阅话题**：`/cmd_vel` (`geometry_msgs/msg/Twist`)
- **速度安全锁**：代码层面实施了严格的**软限幅 `±0.2 m/s` (平移) 及 `±0.2 rad/s` (旋转)**。
- **帧格式**：`[A5] [5A] [Vx_H] [Vx_L] [Vy_H] [Vy_L] [Vz_H] [Vz_L] [XOR] [EE]`
  - `Byte 2~7`: Vx, Vy, Vz，单位放大 1000 倍，**大端序 int16**。
  - `Byte 8`: XOR 校验和（Byte 0~7 累积异或）。
  - *紧急刹车帧：* `0xA5 0x5A 0x00 0x00 0x00 0x00 0x00 0x00 0xFF 0xEE`

### 4.2 STM32 -> Jetson 回传协议（11 字节固定帧，v3.0 新增解析）

- **帧格式**：`[AA] [55] [TYPE] [Vx_H] [Vx_L] [Vy_H] [Vy_L] [Vz_H] [Vz_L] [XOR] [EE]`
  - `TYPE = 0x01`：里程计速度数据（50Hz 上报）
  - `TYPE = 0x02`：电池状态数据（预留，尚未实现）
  - `Byte 3~8`: Vx, Vy, Vz，单位放大 1000 倍，**大端序 int16**。
  - `Byte 9`: XOR 校验和（Byte 0~8 共 9 字节累积异或）。
- **Jetson 端解析方式**：后台接收线程滑窗搜帧 + 三道校验（帧头/帧尾/XOR）
- **发布话题**：`/odom` (`nav_msgs/msg/Odometry`) + TF 广播 (`odom` → `base_link`)

- **硬件连线状态 (2026-03-08 更新)**：
  - 加载了 WCH 原厂 CH341 驱动，USB-TTL 设备名从标准的 `/dev/ttyUSB0` 变为 **`/dev/ttyCH341USB0`** (或 USB1)。
  - 彻底卸载了干扰串口的系统服务 `brltty`。
  - 已将当前用户 `nvidia` 加入 `dialout` 组，解决了串口永久访问权限问题。

## 5. 当前工程状态与已知里程碑 (2026-03-08)

- **通信链路打通**：成功解决 Jetson 内核缺失 CH340/CH341 驱动的问题，通过源码编译实现了串口通信闭环。
- **可视化调试增强**：`serial_node.py` 已集成"三位一体"日志监控，并实现了智能节流打印。
- **v2.0 工程化重构 (2026-03-08)**：实现了参数化限速、串口断线非阻塞自动重连、以及 int16 溢出保护逻辑。
- **工程化规范确立**：建立了 `/home/nvidia/robot_ws/JETSON_RULES.md` 铁律文档，确立了 15 条机器人开发核心准则。
- **v3.0 回传与里程计 (2026-03-08)**：新增后台接收线程解析 STM32 回传的 `AA 55 01` 里程计帧，实现死区推算积分并发布 `/odom` 话题与 `odom→base_link` TF 变换。引入看门狗联动机制，底盘失联时自动冻结运动指令。colcon build 编译通过，零警告。

## 6. v3.0 新增 ROS2 参数清单

| 参数名 | 类型 | 默认值 | 用途 |
|--------|------|--------|------|
| `watchdog_timeout` | double | `0.5` | 回传超时阈值（秒） |
| `odom_frame_id` | string | `odom` | 里程计父坐标系 |
| `base_frame_id` | string | `base_link` | 机器人基座坐标系 |
| `rx_buffer_max` | int | `256` | 接收缓冲区字节上限 |

## 7. 下一步行动点（TODO）

- [x] **代码优化：断线自愈**：已在 v2.0 版本中完成。
- [x] **开发回传回路（里程计）**：v3.0 已实现 TYPE=0x01 里程计解析、积分与 /odom 发布。
- [ ] **回传实车联调验证**：需连接 STM32 实车，验证 /odom 数据与底盘真实运动的方向和数值一致性。
- [ ] **回传扩展：电池状态 (TYPE=0x02)**：解析 STM32 上报的 SOC、电压、电流、故障码。
- [ ] **物理压力测试**：验证在长时间运行和频繁插拔下的稳定性。
- [ ] **工程化部署**：编写 `systemd` 脚本实现开机自启。
