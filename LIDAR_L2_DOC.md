# 宇树 Unitree 4D LiDAR-L2 开发记录

## 1. 硬件规格与默认参数 (来自官方手册)

- **雷达类型**：4D 激光雷达 (3D 位置 + 1D 灰度)
- **型号**：Unitree 4D LiDAR-L2
- **视场角 (FOV)**：水平 360°，垂直 90° (负角度模式可达 96°)
- **探测距离**：0.05m ~ 30m (90% 反射率)
- **采样频率**：128000 点/秒
- **有效频率**：64000 点/秒
- **扫描频率**：周向 5.55Hz，竖直 216Hz
- **物理规格**：尺寸 75×75×65mm，重量约 230g
- **内置 IMU**：3 轴加速度 + 3 轴陀螺仪 (1KHz 采样，500Hz 上报)
- **测量精度**：≤2.0cm (测距分辨率 4.5mm)
- **角分辨率**：0.64°
- **盲区距离**：0.05m (非常小，利于近战避障)
- **最大量程**：30m (@90% 反射率) / 15m (@10% 反射率)
- **环境要求**：工作温度 -10℃~50℃，防护等级 IP54，抗强光 100Klux
- **供电供耗**：12V DC，稳态 10W
- **安全等级**：Class 1 (IEC 60825-1:2014) 人眼安全
- **⚠️ 温度警告**：底盖温度不可超过 85℃，否则会触发过温保护导致停机。当环境在 -10℃~30℃ 时雷达会自动开启自加热模式，此时峰值功耗可达 13W（注意底盘电源余量）。
- **TTL 波特率**：4,000,000 bps (4Mbps，强烈建议改用网口通信)

### 出厂默认配置（关键！）

- **工作模式**：3D 模式、负角度模式
- **输出接口**：ENET 网口输出 (UDP)
- **启动模式**：SELF START (上电自启)
- **附加数据**：GRAY ON (灰度使能)、IMU Disable (默认关闭 IMU)

## 2. 硬件接口与线序定义 (一出三线缆)

雷达出线包含三个端子，具体排线与颜色定义如下（**⚠️严禁接错，特别是电源！**）：

### (1) DC3.4-1.35 供电母头 (电源口)

- **正极**：红色线 (接 12V 稳压输出)
- **负极**：电源线 (GND)

### (2) RJ45 母头 (网口，默认数据口)

- **Pin 1**：ETHTX+ (白橙)
- **Pin 2**：ETHTX- (橙)
- **Pin 3**：ETHRX+ (白绿)
- **Pin 6**：ETHRX- (绿)

### (3) GH1.25-4Y 公头 (串口，调试/可选口)

- **Pin 2**：UART_GND (粉色)
- **Pin 3**：UART_RX (白色)
- **Pin 4**：UART_TX (棕色)
- **Pin 1**：悬空未定义
*(⚠️ 如果使用串口调试，一定要和串口模块共地，也就是连接 Pin 2 粉色线)*

### (4) 坐标系定义 (O-XYZ)

- **原点 (O)**：雷达底部中心位置。
- **+X 轴**：指出线口（电源/网线/串口出线位置）的**正相反方向**。
- **+Y 轴**：+X 轴逆时针旋转 90° 方向。
- **+Z 轴**：垂直底面向上（满足右手坐标系）。

## 3. 开发部署与网络配置 (Jetson Xavier NX)

### 3.1 必须配置的静态 IP (ENET UDP 模式)

雷达出厂把目标上位机的 IP 地址写死了，因此必须把 Jetson 接雷达的网口配置为以下参数，否则收不到 UDP 包：

- **Jetson 的 IP 地址 (目标服务器)**：`192.168.1.2`
- **子网掩码**：`255.255.255.0`
- **网关**：`192.168.1.1` *(注：通常直连不配网关也可)*

*参考雷达自身 IP 为 `192.168.1.62`，UDP 发送端口 `6101`，Jetson 接收端口 `6201`。*

### 3.2 出厂默认模式（⚠️ 待实机验证）

三个官方来源对默认模式说法不一致：

| 来源 | NEGA 模式 | IMU |
|------|-----------|-----|
| 产品概述页 | 默认开启 | 默认关闭 |
| 开发指南 | 默认**不**开启 | — |
| Unilidar 2 软件手册 | 默认开启 | 默认关闭 |
| SDK 文档 (workMode=0) | 标准 FOV (不开启) | **使能** |

- **结论**：可能是不同固件批次间的差异，**必须实际连上雷达后才能确认真实状态**。
- **操作建议**：首次使用前，用 Windows 电脑 + Unilidar 2 软件连上雷达，查看当前实际配置，并根据需要手动开启 NEGA 和 IMU → SetMode → Restart。

### 3.3 驱动层选择

- 我们将在 Jetson 上部署官方的 **Unilidar SDK 2**。它能负责接收点云（距离mm、夹角、反射率）并解析出 IMU 数据，然后包装成 ROS2 的 `sensor_msgs/PointCloud2` 话题供我们使用。

### 3.4 运行状态指示灯 (LED) 与功耗模式

雷达工作时可以通过顶部的 LED 灯环直观判断当前模式：
- **正常 3D 模式 (标准 FOV)**：6 段灯环常亮
- **负角度 3D 模式 (广角)**：3 段灯环常亮
- **2D 模式**：灯环缓慢闪烁
- **待机状态 (Standby)**：由软件下发待机命令后进入，功耗低于 1W，LED 熄灭，电机停转，IMU 及恒温功能同时关闭。

### 3.5 Jetson 网口现状 (2026-03-09 诊断)

- WiFi (`wlP1p1s0`): `192.168.1.26/24`，连接 `ChinaNet-CMLc-5G`，外网正常
- 有线网口 (`eno1`): **未插网线 (NO-CARRIER)**，后续接雷达用
- ⚠️ **路由冲突风险**：WiFi 和雷达目标 IP 在同一子网 `192.168.1.x`，配置 `eno1` 为 `192.168.1.2` 后，系统可能无法正确区分流量走向
- **解决方案**：将 WiFi 连接改为不同子网（如路由器改为 `192.168.0.x` 或 `10.x.x.x`），或在 `eno1` 上添加针对 `192.168.1.62` 的专用路由策略 (`ip route add 192.168.1.62 dev eno1`)

## 4. 开发指南核心摘要

### 4.1 SDK 三种接口 (我们选第③种)

| # | 接口类型 | 适用场景 |
|---|---------|---------|
| ① | 原始 C++ SDK | 自定义平台/嵌入式开发 |
| ② | ROS 软件包 | ROS1 环境 |
| ③ | **ROS2 软件包** | ✅ **我们的选择 (Humble)** |

### 4.2 通信协议总览

- 支持 **UDP (网口)** 和 **UART (串口)** 两种硬件接口，**软件协议完全相同**
- 串口波特率: 4Mbps (数据位8, 停止位1, 无校验)
- ⚠️ **使用网口时，串口 PC 端 TX 引脚不要浮空，否则雷达会输出错误 ACK 包**
- 所有数据包共享统一帧结构：
  - **帧头 (Header)**: 12 字节，标识 `0x55 0xAA 0x05 0x0A` + packet_type + packet_size
  - **帧尾 (Tail)**: 12 字节，CRC32 校验 + 保留字段 + 帧尾标识 `0x00 0xFF`

### 4.3 九种通信数据包

#### 数据输出类 (雷达 → 主机，需解析)

| 包名 | 功能 | 长度 | 发送间隔 |
|------|------|------|---------|
| `LidarPointDataPacket` | **3D 点云** (距离+反射率+角度+标定+脏污指数) | 1044B | ~4.63ms |
| `Lidar2DPointDataPacket` | 2D 点云 (1800点/包) | 5536B | ~28ms |
| `LidarImuDataPacket` | **IMU** (四元数+角速度+加速度) | 80B | ~2ms (500Hz) |
| `LidarAckDataPacket` | 指令应答 | 40B | 按需 |
| `LidarVersionDataPacket` | 版本号 | 104B | 按需 |

#### 配置类 (主机 → 雷达，掉电保存)

| 包名 | 功能 | 长度 |
|------|------|------|
| `LidarWorkModeConfigPacket` | 工作模式 (3D/2D/NEGA/IMU/ENET等) | 28B |
| `LidarIpAddressConfigPacket` | IP/端口/网关/子网配置 | 44B |
| `LidarMacAddressConfigPacket` | MAC 地址配置 | 32B |

#### 命令类 (主机 → 雷达，即时生效)

| 包名 | 功能 | 长度 |
|------|------|------|
| `LidarUserCtrlCmdPacket` | 复位/StandBy/获取版本/延迟测量/恢复默认 | 32B |

### 4.4 3D 点云包关键字段 (LidarPointDataPacket)

**点云显示数据：**

- `com_horizontal_angle_start`: 首点水平角 (rad)
- `com_horizontal_angle_step`: 相邻点水平角差 (rad)
- `angle_min`: 垂直方向首点角 (rad)
- `angle_increment`: 垂直相邻点角差 (rad)
- `time_increment`: 相邻点时间差 (s)
- `scan_period`: 数据包周期 (s)
- `range_min` / `range_max`: 最近/最远测量距离 (m)
- `point_num`: 单包点数量 (2D 模式下一般为 1800)
- `Ranges[]`: 距离数据 (mm)
- `Intensities[]`: 反射率 (0~255)
- 采用**空间极坐标**方式输出，数据量小但点数多

**雷达内部状态 (LidarInsideState，每包随附)：**

| 字段 | 含义 | 备注 |
| --- | --- | --- |
| `sys_rotation_period` | 高速电机旋转周期 (μs) | 监控电机健康 |
| `com_rotation_period` | 低速电机旋转周期 (μs) | 监控电机健康 |
| `dirty_index` | 脏污指数 | 光学窗口清洁度 |
| `packet_lost_up` | 光通信上行信号质量 | 通信质量诊断 |
| `packet_lost_down` | 光通信下行信号质量 | 通信质量诊断 |
| `apd_temperature` | APD 温度 | 温控参考 |
| `apd_voltage` | APD 电压 | 硬件状态 |
| `laser_voltage` | 激光发射电压 | 硬件状态 |
| `imu_temperature` | IMU 当前温度 | 温控参考 |

> 注：Seq 序列号最大值为 1024，标定参数 `LidarCalibParam` 由 SDK 内部处理，无需手动关注。

> ⚠️ **官方文档自相矛盾警告**：上位机手册中 `sys_rotation_period` 被描述为"低速电机"、`com_rotation_period` 被描述为"高速电机"，与开发指南恰恰相反；`imu_temperature` 在手册中被错标为"电压 (V)"而非温度。以实际 SDK 源码 `unitree_lidar_protocol.h` 中的定义为准。

### 4.5 IMU 包关键字段 (LidarImuDataPacket)

- `quaternion[4]`: 四元数姿态
- `angular_velocity[3]`: 3 轴角速度
- `linear_acceleration[3]`: 3 轴加速度

### 4.6 用户命令类型 (LidarUserCtrlCmdPacket)

| 命令 | 功能 |
| --- | --- |
| `USER_CMD_RESET_TYPE` | 雷达复位 |
| `USER_CMD_STANDBY_TYPE` | 切换 StandBy 待机 |
| `USER_CMD_VERSION_GET` | 获取版本号 |
| `USER_CMD_LATENCY_TYPE` | 计算通信延时 |
| `USER_CMD_CONFIG_RESET` | 恢复默认配置 (掉电保存) |

### 4.7 ACK 应答状态码 (LidarAckDataPacket)

雷达收到配置包或命令包后会回复 ACK 包，其中 `status` 字段含义如下：

| 状态码 | 含义 |
| --- | --- |
| `ACK_SUCCESS` | 通信已被成功处理 |
| `ACK_CRC_ERROR` | 通信包 CRC 校验异常 |
| `ACK_HEADER_ERROR` | 通信包帧头异常 |
| `ACK_BLOCK_ERROR` | 通信包阻塞异常 |
| `ACK_WAIT_ERROR` | 包已接收但应答数据尚未生成，需稍后重试 |

### 4.8 时间同步机制

- 雷达完全启动后的时刻为起始点，每包数据都带 `Stamp` 时间戳
- SDK 包含同步系统时间的功能，但需注意网络延时误差

## 5. Unilidar 2 上位机软件关键信息 (Windows)

### 5.1 出厂默认值确认表

| 参数项 | 默认值 | 说明 |
|--------|--------|------|
| Work Mode | **NEGA Mode** | 负角度 360°×96° |
| 3D/2D | **3D Mode** | 三维点云 |
| IMU | **Disable** | 需手动开启 |
| ENET/UART | **ENET** | 网口 UDP 输出 |
| Power On | **SELF START** | 上电自启 |
| Gray | **Gray ON** | 灰度使能 |

### 5.2 模式修改操作流程

所有模式修改都必须遵循以下三步:

1. 在界面中选择参数
2. 点击 **"SetMode"** 下发
3. 点击 **"Restart"** 重启雷达 → 才生效

### 5.3 IP 忘记/改乱后的恢复方法

如果忘记了雷达的 IP 设置，可以：

1. 通过 **串口 (UART)** 物理线连接雷达
2. 在 Unilidar 2 软件中选 SerialMode 打开串口
3. 在 "Config Setting" 中重新配 IP 或点 **"Restore Factory Defaults"** 恢复出厂
4. 出厂默认: IP `192.168.1.62`, 目标 `192.168.1.2`, 端口 `6101/6201`

### 5.4 IMU 数据字段

- **四元数**: Q1, Q2, Q3, Q4 (旋转姿态)
- **加速度计**: ACC1(X), ACC2(Y), ACC3(Z)
- **陀螺仪**: GYRO1(X), GYRO2(Y), GYRO3(Z)

## 6. Unilidar SDK 2 关键信息

### 6.1 SDK 仓库结构

- `unitree_lidar_sdk` — 原始 C++ SDK
- `unitree_lidar_ros` — ROS1 软件包
- `unitree_lidar_ros2` — **ROS2 软件包 (我们要用的)**

### 6.2 工作模式位掩码 (workMode: uint32_t)

| Bit | 功能 | 0 | 1 |
|-----|------|---|---|
| 0 | FOV | 标准 180° | **广角 NEGA 192°** |
| 1 | 模式 | **3D** | 2D |
| 2 | IMU | **使能** | 关闭 |
| 3 | 通信 | **ENET 网口** | UART 串口 |
| 4 | 启动 | **上电自启** | 等待命令 |

- 出厂默认 `workMode=0` (全0) = 标准FOV + 3D + IMU使能 + ENET + 自启
- ⚠️ **与 Unilidar 2 软件手册矛盾**：软件说默认 NEGA+IMU关闭，SDK 说默认全0。可能是固件批次差异，**必须实际连雷达确认**。

### 6.3 IMU → 点云坐标系变换 (纯平移)

IMU 原点在点云坐标系中的位置 (米)：

- `x = -0.007698`
- `y = -0.014655`
- `z =  0.00667`

### 6.4 点云数据格式

每个点: `(x, y, z, intensity, time, ring)`

- 每包约 5200 个点，18 个 ring
- 距离单位: 米 (SDK 已转换)

### 6.5 C++ SDK 编译 (参考)

```bash
cd unitree_lidar_sdk
mkdir build && cd build
cmake .. && make -j2
```

### 6.6 UDP 模式运行前提

Jetson 网口必须配置为 `192.168.1.2`，然后运行 `example_lidar_udp`。

### 6.7 串口模式补充说明

- **默认串口设备名**：`/dev/ttyACM0`（使用官方转接板连接时）
- ⚠️ **重要依赖关系**：出厂默认为 ENET 网口模式。如需切换到串口模式，**必须先通过网口与雷达建立通信**，然后将 `workMode` 设为 `8`（Bit3=1），保存后掉电重启，雷达才会切到串口输出。无法在未连通的情况下直接使用串口。

## 7. unitree_lidar_ros2 软件包 (核心！)

### 7.1 官方验证环境

- Ubuntu 20.04 + ROS2 **Foxy** + PCL-1.10
- ⚠️ **我们是 Ubuntu 22.04 + ROS2 Humble**，编译时可能需要适配

### 7.2 依赖

- PCL (Point Cloud Library)
- ROS2
- `unitree_lidar_sdk` (C++ SDK，作为底层依赖)

### 7.3 默认 ROS2 话题与坐标系

| 数据 | 话题名 | 坐标系 (frame_id) |
|------|--------|-------------------|
| 点云 | `unilidar/cloud` | `unilidar_lidar` |
| IMU | `unilidar/imu` | `unilidar_imu` |

### 7.4 配置文件路径

```
unitree_lidar_ros2/src/unitree_lidar_ros2/launch/launch.py
```

可修改：工作模式、话题名、坐标系名等。

### 7.5 编译与运行

```bash
# 编译
cd unilidar_sdk/unitree_lidar_ros2
colcon build

# 运行
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

### 7.6 后续与底盘对接要点

- 雷达坐标系 `unilidar_lidar` 需要通过 **静态 TF** 连接到我们底盘的 `base_link`
- IMU 坐标系 `unilidar_imu` 与点云坐标系仅有平移偏差 (见 6.3)

## 8. 疑难解答 (官方 FAQ)

| 问题 | 解决方案 |
|------|---------|
| UART 无数据 | 检查线材连接 → 确认 12V/1A 供电 → 确认输出模式为 UART → 重启雷达和软件 |
| UDP 无数据 | 检查线材 → 确认 12V/1A 供电 → 确认 IP 无冲突 → 确认端口 6201 未被占用 → 确认输出为 ENET |
| 忘记 IP 配置 | 网口连接 → Unilidar 2 → Config Setting → Restore Factory Defaults (多点几次防丢包) → Restart；或串口连接重新配 |
| 能检测串口但打不开 | 检查线材 + 供电 → 重启雷达和软件 |
| 有点云但无 IMU | IMU Display Enable 设为了 Disable，需改为 Enable → SetMode → Restart |
| 外力干扰后停转 | 松手后通常自动恢复，否则重启雷达 |

---
*所有官方资料（产品概述+开发指南+上位机手册+SDK+ROS2+FAQ）已全量阅读完毕！*
