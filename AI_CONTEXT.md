# WL100_ROS2 (Jetson Xavier NX) 项目上下文与状态记录 (AI_CONTEXT)

> **核心提示给下一次对话的 AI**：
> 请在接手本工作区（`/home/nvidia/robot_ws`）的任何任务前，务必优先阅读本文件，以确保环境配置、硬件连线、协议标准和当前进度的上下文绝对同步，严禁基于错误上下文修改已验证无误的核心代码。

## 1. 硬件架构与连线状态

- **上位机（主控）**：NVIDIA Jetson Xavier NX (16GB)
- **下位机（底盘控制）**：STM32F407
- **底盘硬件**：WL100 全向轮底盘（MANDA）
- **激光雷达**：宇树 Unitree 4D LiDAR-L2（360°×90° FOV，30m 量程，128000 点/秒，内置 IMU）
- **通信物理层**：
  - Jetson 与 STM32：**UART 串口通信**（非 SocketCAN！！不要在 Jetson 跑 CAN 节点）。
  - 接线方式：使用 USB 转 TTL 模块（Jetson 端识别为 `/dev/ttyUSB0`）。
    - Jetson TX  `->` STM32 RX (PB11)
    - Jetson RX  `->` STM32 TX (PB10)
    - GND `->` GND
  - *(备选串口)*：如果不用 USB 转串口模块，Jetson 主板自带的 UART 引脚对应 `/dev/ttyTHS1`。
  - Jetson 与 LiDAR-L2：**以太网 UDP 通信**（RJ45 直连 `eno1` 网口）。
    - 雷达 IP：`192.168.1.62`，发送端口 `6101`
    - Jetson IP：`192.168.1.2`，接收端口 `6201`
    - 雷达供电：**12V DC 稳压**（稳态 10W，自加热峰值 13W）

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
  - `libpcl-dev` (Point Cloud Library 1.12.1，雷达点云处理)
  - `unilidar_sdk2` v2.0.10 (宇树 LiDAR-L2 官方 SDK，预编译 aarch64 静态库)
- **网络配置 (2026-03-10 持久化)**：
  - WiFi (`wlP1p1s0`)：`192.168.1.26/24`，连接家庭路由，用于 SSH 和外网
  - 有线 (`eno1`)：`192.168.1.2/32`，直连雷达，NetworkManager 连接名 `lidar-eno1`
  - 专用路由：`192.168.1.62/32 dev eno1`（仅雷达流量走有线，WiFi 不受影响）
  - ⚠️ **路由策略说明**：WiFi 和雷达同在 `192.168.1.x` 网段，使用 `/32` 掩码 + 主机路由避免冲突，`ipv4.never-default=yes` 确保默认路由仍走 WiFi

## 3. 核心功能包信息

- **工作区路径**：`/home/nvidia/robot_ws`
- **工作区结构**：
  ```
  src/
  ├── wl100_description/   # 机器人 URDF 描述包
  ├── wl100_bringup/       # 统一启动 + 参数配置包
  ├── wl100_teleop/        # 底盘串口通信节点
  └── unilidar_sdk2/       # 雷达 SDK（独立 colcon 工作区）
  ```

### 3.1 底盘通信功能包 (wl100_teleop)

- **包类型**：`ament_python`
- **核心节点文件**：
  - `serial_node.py`：串口管理、ROS2 生命周期、线程调度中心
  - `protocol_tx.py`：下发协议编码、安全限幅、校验打包
  - `protocol_rx.py`：回传协议解析、滑窗搜帧过滤
  - `odometry.py`：全向底盘死区推算积分器、TF 及里程计发布
- **执行入口**：`ros2 run wl100_teleop serial_node` （支持动态传参：`--ros-args -p port_name:=/dev/ttyCH341USB0 -p baudrate:=115200`）

### 3.2 雷达功能包 (unitree_lidar_ros2)

- **SDK 仓库路径**：`/home/nvidia/robot_ws/src/unilidar_sdk2/`（GitHub 克隆，v2.0.10）
- **ROS2 包路径**：`src/unilidar_sdk2/unitree_lidar_ros2/src/unitree_lidar_ros2/`
- **包类型**：`ament_cmake` (C++17)
- **编译位置**：`src/unilidar_sdk2/unitree_lidar_ros2/`（独立 colcon 工作区）
- **可执行文件**：`unitree_lidar_ros2_node`
- **发布话题**：
  - `/unilidar/cloud` (`sensor_msgs/PointCloud2`)：3D 点云，~12 Hz，坐标系 `unilidar_lidar`
  - `/unilidar/imu` (`sensor_msgs/Imu`)：IMU 数据，~246 Hz，坐标系 `unilidar_imu`
  - `/tf`：IMU 初始 → IMU 实时、IMU → 点云 坐标变换
- **关键参数**：`initialize_type:=2`（UDP 模式）、`lidar_ip`、`local_ip`、`lidar_port`、`local_port`
- **⚠️ 注意**：确保启动前无残留雷达进程占用 UDP 6201 端口，否则会出现 `bind failed` 和 `WARNING` 刷屏

### 3.3 机器人描述包 (wl100_description)

- **包类型**：`ament_cmake`
- **URDF 模型**：`urdf/wl100.urdf.xacro`（xacro 格式，全参数化，官方规格数据 + 实测校准）
- **建模精度**：基本几何体精确还原，底盘 box + 4 轮 cylinder + 型材支架 + 绿色灯带 + LiDAR 两段圆柱
- **TF 树结构**：`base_footprint → base_link → {wheel×4, rail×2, led, unilidar_lidar}`（全 fixed joint）
- **底盘尺寸**：L730×W500×H365mm，离地间隙 120mm，轴距 480mm，轮距 380mm，8 寸轮毂电机/200mm 实心轮胎
- **launch**：`display.launch.py`（启动 `robot_state_publisher` 发布 TF）
- **预留目录**：`meshes/`（3D 模型）、`rviz/`（RViz 预设配置）
- **依赖**：`robot_state_publisher`、`xacro`、`joint_state_publisher`

### 3.4 统一启动包 (wl100_bringup)

- **包类型**：`ament_cmake`
- **launch**：`robot.launch.py`（一键启动：URDF + 底盘节点 + 雷达节点）
- **参数配置**：
  - `config/chassis_params.yaml`：底盘串口、限速、看门狗参数
  - `config/lidar_params.yaml`：雷达 IP、端口、工作模式参数
- **⚠️ 注意**：`unitree_lidar_ros2` 在独立工作区，运行前需额外 `source` 其 `setup.bash`

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
- **发布话题**：`/odom` (`nav_msgs/msg/Odometry`) + TF 广播 (`odom` → `base_footprint`)

- **硬件连线状态 (2026-03-08 更新)**：
  - 加载了 WCH 原厂 CH341 驱动，USB-TTL 设备名从标准的 `/dev/ttyUSB0` 变为 **`/dev/ttyCH341USB0`** (或 USB1)。
  - 彻底卸载了干扰串口的系统服务 `brltty`。
  - 已将当前用户 `nvidia` 加入 `dialout` 组，解决了串口永久访问权限问题。

## 5. 当前工程状态与已知里程碑

- **通信链路打通 (2026-03-08)**：成功解决 Jetson 内核缺失 CH340/CH341 驱动的问题，通过源码编译实现了串口通信闭环。
- **可视化调试增强**：`serial_node.py` 已集成"三位一体"日志监控，并实现了智能节流打印。
- **v2.0 工程化重构 (2026-03-08)**：实现了参数化限速、串口断线非阻塞自动重连、以及 int16 溢出保护逻辑。
- **工程化规范确立**：建立了 `/home/nvidia/robot_ws/JETSON_RULES.md` 铁律文档，确立了 15 条机器人开发核心准则。
- **v3.0 回传与里程计 (2026-03-08)**：新增后台接收线程解析 STM32 回传的 `AA 55 01` 里程计帧，实现死区推算积分并发布 `/odom` 话题与 `odom→base_link` TF 变换。引入看门狗联动机制，底盘失联时自动冻结运动指令。colcon build 编译通过，零警告。
- **v3.1 代码解耦与模块化重构 (2026-03-08)**：将原先的 `serial_node.py` 进行逻辑拆分。剥离出 `protocol_tx.py`（打包下发）、`protocol_rx.py`（解析回传）、`odometry.py`（位姿积分）三个独立模块。主节点文件化身为纯粹的调度中心，极大提升了代码可测试性与安全性。
- **LiDAR-L2 雷达集成 (2026-03-10)**：
  - 完成 Jetson `eno1` 网口静态 IP 配置（`192.168.1.2/32`）并持久化（NetworkManager `lidar-eno1`）。
  - 解决了 WiFi 与雷达同网段 `192.168.1.x` 的路由冲突（`/32` 掩码 + 主机路由策略）。
  - 克隆并编译官方 `unilidar_sdk2` v2.0.10 ROS2 包，零警告。
  - 验证 `/unilidar/cloud` 点云 ~12Hz 稳定、`/unilidar/imu` ~246Hz 稳定。
  - RViz2 3D 点云可视化验证通过。
  - 节点 Ctrl+C 正常退出，无残留进程。
- **工程化包结构建立 (2026-03-10)**：
  - 创建 `wl100_description` 包（URDF xacro 模板 + robot_state_publisher launch）。
  - 创建 `wl100_bringup` 包（统一 launch + 参数 YAML 配置文件）。
  - 两个包 `colcon build` 编译通过，零错误。xacro 解析验证通过。
  - 经两轮严格代码审查，修复 5 个 BUG（import 路径、YAML 命名空间、邮箱格式、跨工作区依赖、空目录安装）。
- **参数体系统一化 (2026-03-10)**：
  - 修复 `chassis_params.yaml` 与代码中参数名不匹配的 BUG（`max_linear_vel` → `max_linear_velocity`）。
  - 补全 YAML 中缺失的 `reconnect_interval` 参数，完善全部注释。
  - 参数清单在代码、YAML、文档三端完全对齐（共 9 项）。
- **URDF 精确建模 (2026-03-10 最新)**：
  - 基于官方产品尺寸图 + 规格参数表 + 实测数据，完成 WL100 底盘的精确 URDF 描述。
  - 车体 box (0.73×0.50×0.245m) + 4 轮 cylinder (R0.10×W0.055m) + 型材支架 (0.66×0.025×0.01m) + 绿色灯带标识车头。
  - LiDAR-L2 两段式建模：Φ75×24mm 圆柱底座 + Φ60×41mm 上半部，尺寸对齐官方 L2 机械尺寸图。
  - 含惯性参数（chassis 75kg + wheel 3kg×4），xacro 解析 + colcon build 零警告。

## 6. 完整 ROS2 参数清单

| 参数名 | 类型 | 默认值 | 用途 |
|--------|------|--------|------|
| `port_name` | string | `/dev/ttyCH341USB0` | 串口设备路径 |
| `baudrate` | int | `115200` | 串口波特率 (bps) |
| `max_linear_velocity` | double | `0.2` | 平移速度限幅绝对值 (m/s) |
| `max_angular_velocity` | double | `0.2` | 旋转速度限幅绝对值 (rad/s) |
| `reconnect_interval` | double | `2.0` | 断线重连尝试间隔 (秒) |
| `watchdog_timeout` | double | `0.5` | 回传超时阈值 (秒) |
| `odom_frame_id` | string | `odom` | 里程计父坐标系 |
| `base_frame_id` | string | `base_footprint` | 里程计子坐标系 (REP-105) |
| `rx_buffer_max` | int | `256` | 接收缓冲区字节上限 |

## 7. 下一步行动点（TODO）

- [x] **代码优化：断线自愈**：已在 v2.0 版本中完成。
- [x] **开发回传回路（里程计）**：v3.0 已实现 TYPE=0x01 里程计解析、积分与 /odom 发布。
- [x] **代码解耦模块化**：v3.1 已将通信协议的收发解析逻辑、里程计推算逻辑从 ROS2 主节点中完全抽离。
- [x] **LiDAR-L2 雷达驱动集成**：网络配置持久化 + SDK 编译 + 点云/IMU 话题验证 + RViz2 可视化确认。
- [x] **雷达 TF 对接**：URDF 中已定义 `base_link → unilidar_lidar` fixed joint，`robot_state_publisher` 启动后自动发布。雷达安装位置 lidar_x/y/z 待实测确认后填入 xacro 参数。
- [ ] **统一 Launch 文件验证**：`robot.launch.py` 已编写，需实际启动验证全链路（底盘 + 雷达 + URDF）。
- [ ] **回传实车联调验证**：需连接 STM32 实车，验证 /odom 数据与底盘真实运动的方向和数值一致性。
- [ ] **回传扩展：电池状态 (TYPE=0x02)**：解析 STM32 上报的 SOC、电压、电流、故障码。
- [ ] **物理压力测试**：验证在长时间运行和频繁插拔下的稳定性。
- [ ] **工程化部署**：编写 `systemd` 脚本实现开机自启。

## 8. 常用运行指令 (Quick Start)

为了确保节点正常运行，每次启动前请按照以下标准流程执行：

### 8.1 编译与环境初始化

```bash
cd /home/nvidia/robot_ws
colcon build --packages-select wl100_teleop
source install/setup.bash
```

### 8.2 标准启动指令 (默认使用 CH341 驱动串口)

```bash
ros2 run wl100_teleop serial_node --ros-args -p port_name:=/dev/ttyCH341USB0 -p baudrate:=115200
```

*提示：如果使用普通 USB-TTL 模块，请将 `port_name:=/dev/ttyUSB0`*

### 8.3 LiDAR-L2 雷达启动指令

```bash
# 编译（首次或代码变更后）
cd /home/nvidia/robot_ws/src/unilidar_sdk2/unitree_lidar_ros2
colcon build

# 启动雷达节点
source install/setup.bash
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args \
  -p initialize_type:=2 \
  -p lidar_ip:="192.168.1.62" \
  -p local_ip:="192.168.1.2" \
  -p lidar_port:=6101 \
  -p local_port:=6201
```

*⚠️ 启动前确保无残留雷达进程：`ps aux | grep unitree`，如有则先 `killall -9 unitree_lidar_ros2_node`*

### 8.4 辅助调试指令

* **键盘控制测试**：

  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

* **查看里程计数据回传**：

  ```bash
  ros2 topic echo /odom
  ```

* **实时查看坐标变换 (TF)**：

  ```bash
  ros2 run tf2_ros tf2_echo odom base_link
  ```

* **查看雷达点云频率**：

  ```bash
  ros2 topic hz /unilidar/cloud
  ```

* **查看雷达 IMU 频率**：

  ```bash
  ros2 topic hz /unilidar/imu
  ```

* **RViz2 点云可视化** (需 X11 转发，如 MobaXterm)：

  ```bash
  rviz2
  # Fixed Frame 设为 unilidar_lidar，Add → By topic → /unilidar/cloud → PointCloud2
  ```

---
*本文件由 AI 在 2026-03-10 20:04 完成最后一次更新并校验。*
