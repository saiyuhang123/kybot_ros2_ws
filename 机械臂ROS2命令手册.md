# 天链机械臂 ROS2 包（tl_driver）命令与功能手册

适用包：`tl_robot_ros2`（tl_driver + tl_ros2_interface + tl_bringup 等）
驱动节点：`tl_driver`，通过 TCP 连接控制器（指令口 6001，实时口 7000）

> 双臂时所有服务/话题加前缀 `/arm_left`（192.168.1.13）或 `/arm_right`（192.168.1.14）。
> 下文示例均以左臂（/arm_left）为例；操作右臂时把 arm_left 换成 arm_right 即可。单臂模式下去掉前缀，直接用 /tl_driver/...。

## 〇、常见问题速答

| 需求 | 有没有 | 用哪个 |
|---|---|---|
| 独立 IK 解算服务（给位姿出关节角） | **没有独立服务** | 但 IK 已内嵌在笛卡尔运动里（moveL/servoL 自动解算）；`get_pos_reachable` 可校验位姿可达性 |
| 笛卡尔末端移动 | 有 | `moveL`（直线）、`job_insert_moveC`（圆弧）、`servoL`（实时伺服直线）、moveJ 也接受直角坐标目标 |
| 单关节转动 | 有 | `start_jogging`/`stop_jogging`（点动），或 `set_servoj_pos` 只改一个关节角 |
| 姿态表示互转 | 有 | `rpy2quat`、`quat2rpy`、`rpy2r`、`r2tr`、`tr2r`、`coord_transform` |

## 一、生命周期（开机必走流程）

```bash
ros2 service call /arm_left/tl_driver/connect_arm std_srvs/srv/Trigger    # 连接控制器（启动后通常自动连）
ros2 service call /arm_left/tl_driver/clear_error std_srvs/srv/Trigger    # 有报警先清错
ros2 service call /arm_left/tl_driver/power_on std_srvs/srv/Trigger       # 上电（伺服使能）
ros2 service call /arm_left/tl_driver/get_robot_state tl_ros2_interface/srv/GetRobotState {}  # 查状态
ros2 service call /arm_left/tl_driver/power_off std_srvs/srv/Trigger      # 下电
ros2 service call /arm_left/tl_driver/disconnect_arm std_srvs/srv/Trigger # 断开连接
```

## 二、运动控制

### 1. 单关节转动（点动 jogging）

```bash
# 开始转：axis=关节号1~6，direction=true正转/false反转
ros2 service call /arm_left/tl_driver/start_jogging tl_ros2_interface/srv/Jogging "{axis: 3, direction: true}"
# 停止该关节
ros2 service call /arm_left/tl_driver/stop_jogging tl_ros2_interface/srv/Jogging "{axis: 3}"
```

### 2. 关节空间运动 moveJ（话题）

`tl_ros2_interface/msg/MoveCommand`，关键字段：

- `target_pos_value`：话题实时控制时**前 n 位直接填点位值**（关节模式=各轴角度[度]，直角模式=[X mm,Y,Z,RX rad,RY,RZ]），其余置 0。（job 文件模式的 meta 布局见厂家文档，话题接口不用）
- `coord`：坐标系选择，同上 0~3
- `velocity`：速度百分比

```bash
# 关节模式：6 轴目标角度（度）放在 [7~12]
ros2 topic pub --once /arm_left/tl_driver/moveJ tl_ros2_interface/msg/MoveCommand "{
  target_pos_value: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  10.0, -20.0, 30.0, 0.0, 45.0, 0.0, 0.0],
  coord: 0, velocity: 20.0, acc: 0, dec: 0, pl: 0, time: 0,
  tool_num: 0, user_num: 0, ...}"
```

### 3. 笛卡尔末端移动 moveL（话题）

同一个 MoveCommand 消息，`coord` 用直角坐标系（1），`[7~13]` 填 `[x, y, z, rx, ry, rz]`。控制器内部自动做 IK 解算。

```bash
ros2 topic pub --once /arm_left/tl_driver/moveL tl_ros2_interface/msg/MoveCommand "{
  target_pos_value: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.30, 0.0, 0.40, 180.0, 0.0, 0.0, 0.0],
  coord: 1, velocity: 20.0, ...}"
```

### 4. 实时伺服控制（高频流式）

```bash
# 先打开关节跟踪模式
ros2 service call /arm_left/tl_driver/open_servoj tl_ros2_interface/srv/OpenServoJ ...

# 关节伺服：持续发 6 关节目标角（单关节动 = 数组里只改一个值）
ros2 topic pub /arm_left/tl_driver/set_servoj_pos std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 笛卡尔伺服：节点自动取当前位姿、按 step_size 插值并 IK 成关节角后下发
ros2 topic pub /arm_left/tl_driver/set_servol_pos tl_ros2_interface/msg/ServolMove \
  "{target_pose: [0.30, 0.0, 0.40, 3.14, 0.0, 0.0], step_size: 2.0, coord: 1}"

# 用完关闭
ros2 service call /arm_left/tl_driver/close_servoj std_srvs/srv/Trigger
```

> servoL 的 `target_pose` 单位：位置 mm、姿态 rad；`coord`：1=基座标 2=工具 3=用户。

### 5. 位姿可达性检查（相当于 IK 可行性校验）

```bash
ros2 service call /arm_left/tl_driver/get_pos_reachable tl_ros2_interface/srv/GetPosReachable \
  "{pos: [0.30, 0.0, 0.40, 3.14, 0.0, 0.0], move_type: 'moveL'}"
```

## 三、状态反馈话题

| 话题 | 类型 | 内容 |
|---|---|---|
| `/arm_left/joint_states` | sensor_msgs/JointState | 6 关节角度（默认 10Hz 级，频率由配置 `publish_rate` 决定） |
| `/arm_left/tcp_pose` | tl_ros2_interface/CartesianPose | 末端位姿：position + rpy + arm_angle |
| `/arm_left/arm_status` | tl_ros2_interface/ArmStatus | 臂运行状态 |

```bash
ros2 topic echo /arm_left/tcp_pose --once
ros2 topic hz /arm_left/joint_states
```

## 四、查询类服务（部分）

```bash
ros2 service call /arm_left/tl_driver/get_speed tl_ros2_interface/srv/GetSpeed {}          # 当前速度比
ros2 service call /arm_left/tl_driver/get_current_mode tl_ros2_interface/srv/GetCurrentMode {}
ros2 service call /arm_left/tl_driver/get_current_motor_torque tl_ros2_interface/srv/GetCurrentMotorTorque {}
ros2 service call /arm_left/tl_driver/get_dh_param tl_ros2_interface/srv/GetDHParam {}     # DH 参数
ros2 service call /arm_left/tl_driver/get_digital_input_output tl_ros2_interface/srv/GetDigitalInputOutput {}
```

## 五、设置类服务（部分）

- `set_speed`：全局速度比
- `set_tool_param` / `set_user_coord` / `set_current_coord`：工具坐标系、用户坐标系
- `set_drag_mode`：拖拽示教模式开关（配合 `get_drag_status`）
- `set_digital_output` / `modbus_read` / `modbus_write`：IO 与 Modbus
- `set_axis_zero_pos` / `restore_default_dh_param`：标定类，**慎用**

## 六、程序（job）与轨迹

- `job_insert_imove` / `job_insert_moveC` / ...：把运动指令插入 job 文件（支持圆弧 moveC）
- `job_run` / `job_delete` / `get_all_job_filename`：job 运行与管理
- `track_save` / `track_playback`：轨迹录制与复现（拖拽示教后回放用）
- `queue_motion_movej` / `queue_motion_set_status` / `queue_motion_stop`：队列式运动

## 七、安全测试指令（首次调试推荐）

> 原则：**先看当前位姿 → 只做小幅度相对运动 → 低速 → 手放在急停附近**

### 1. 先读当前状态

```bash
ros2 topic echo /arm_left/joint_states --once     # 当前 6 关节角（度）
ros2 topic echo /arm_left/tcp_pose --once         # 当前末端位姿
```

### 2. 安全 moveJ（关节空间，只动一个关节 +10°，速度 10%）

把读到的当前关节角填进 `[7~12]`，只把其中一个关节（示例为 joint6）加 10°：

```bash
ros2 topic pub --once /arm_left/tl_driver/moveJ tl_ros2_interface/msg/MoveCommand "{
  target_pos_value: [J1, J2, J3, J4, J5, J6加10,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  target_pos_name: '', target_pos_type: 0, coord: 0,
  velocity: 10.0, velocity_sync: 0.0, acc: 20.0, dec: 20.0, pl: 0, time: 0,
  tool_num: 0, user_num: 0, posidtype: 0, configuration: 0, spin: 0, para_sync: false}"
```

字段含义：话题实时控制时数组**前 6 位直接是六关节目标角（单位：度）**，其余置 0。（注意：job 文件模式的 `[7~13]` 布局不适用于话题接口）

### 3. 安全 moveL（笛卡尔，末端沿 z 抬 50mm，速度 10%）

把 `/arm_left/tcp_pose` 读到的值填入，位置单位 **mm**、姿态单位 **rad**（示例 z 加 50mm）：

```bash
ros2 topic pub --once /arm_left/tl_driver/moveL tl_ros2_interface/msg/MoveCommand "{
  target_pos_value: [Xmm, Ymm, Zmm加50, RXrad, RYrad, RZrad,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  target_pos_name: '', target_pos_type: 0, coord: 1,
  velocity: 10.0, velocity_sync: 0.0, acc: 20.0, dec: 20.0, pl: 0, time: 0,
  tool_num: 0, user_num: 0, posidtype: 0, configuration: 0, spin: 0, para_sync: false}"
```

话题实时控制时数组**前 6 位直接是 [X(mm), Y, Z, RX(rad), RY, RZ]**，其余置 0。下发前建议先用 `get_pos_reachable` 校验目标可达。

### 4. 停止命令

```bash
# 运动中的急停（断伺服使能，电机抱闸，臂立刻停住）
ros2 service call /arm_left/tl_driver/power_off std_srvs/srv/Trigger

# 点动模式的停止（start_jogging 之后用）
ros2 service call /arm_left/tl_driver/stop_jogging tl_ros2_interface/srv/Jogging "{axis: 3}"

# 队列运动模式的停止（仅限 queue_motion 开启后下发的运动）
ros2 service call /arm_left/tl_driver/queue_motion_stop std_srvs/srv/Trigger {}
```

> 注意：普通 moveJ/moveL 走的是 `robot_movej/robot_movel` 直发通道，`queue_motion_stop` 停不了它们；
> 运动失控就直接 `power_off`。恢复时重新 `clear_error` → `power_on`。

## 八、坐标/姿态转换服务
| 服务 | 功能 |
|---|---|
| `rpy2quat` / `quat2rpy` | RPY 角 ↔ 四元数 |
| `rpy2r` / `r2tr` / `tr2r` | RPY ↔ 旋转矩阵 ↔ 齐次变换矩阵 |
| `coord_transform` | 坐标系间位姿换算 |

## 八、完整服务清单

详细字段说明见厂家文档：`tl_robot_ros2/src/arm_left/tl_driver/doc/`（服务与话题说明书）。
全部服务名可用命令查看：`ros2 service list | grep tl_driver`
