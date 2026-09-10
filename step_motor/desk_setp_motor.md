---

# 升降桌 ROS 2 控制节点使用文档

本项目基于串口（RS-485 / 虚拟串口协议），提供电动升降桌的 ROS 2 驱动控制节点。支持动作控制服务、精准高度控制话题以及实时高度反馈。

---

## 1. 硬件与环境配置

| 参数项 | 当前设备配置 | 备注说明 |
| :--- | :--- | :--- |
| **设备端口** | `/dev/ttyDESK` | udev 固定别名，规则见 `/etc/udev/rules.d/99-ttydesk.rules`（沁恒 CH343，VID:PID=1a86:55d3，SN=575A018668） |
| **波特率** | `38400` | 驱动板通信波特率 |
| **数据位/校验/停止** | `8 - N - 1` | 8位数据位，无校验，1位停止位 |
| **系统依赖** | `pip install pyserial` | Python 串口通信库 |

### 串口别名说明（2026-09-10 起）

设备已通过 udev 绑定固定别名 `/dev/ttyDESK`（按 VID+PID+序列号匹配），
插拔后无论分配到 ttyACM0 还是 ttyACM1 都不影响使用。
底盘 Arduino 为 `/dev/ttyARDUINO`（规则 `99-ttyarduino.rules`），两者不会串号。
**注意**：若更换 485 适配器（即使同型号），序列号不同会导致别名失效，需更新规则中的 SN。

### 串口免密权限配置（初次使用必须执行）
```bash
sudo chmod 666 /dev/ttyDESK
# 或永久赋予当前用户串口权限（推荐）：
sudo usermod -aG dialout $USER
newgrp dialout
```

---

## 2. 节点启动方式

### 方式一：默认参数启动（直接运行）
```bash
python3 desk_controller.py
```

### 方式二：命令行指定参数启动（推荐）
```bash
python3 desk_controller.py --ros-args -p port:=/dev/ttyDESK -p baudrate:=38400
```

---

## 3. ROS 2 服务接口列表 (Services)

所有基础动作均采用 ROS 2 标准服务类型 **`std_srvs/srv/Trigger`**，无需自定义服务接口，开箱即用。

| 服务名称 | 接口类型 | 对应底层报文 | 功能说明 |
| :--- | :--- | :--- | :--- |
| `/desk/up` | `std_srvs/srv/Trigger` | `aa 03 02 00 ff` | 控制桌子持续上升 |
| `/desk/down` | `std_srvs/srv/Trigger` | `aa 03 03 00 ff` | 控制桌子持续下降 |
| `/desk/stop` | `std_srvs/srv/Trigger` | `aa 03 04 00 ff` | **紧急停止**当前动作 |
| `/desk/reset` | `std_srvs/srv/Trigger` | `aa 03 08 00 ff` | 系统复位（归零校准） |
| `/desk/get_error` | `std_srvs/srv/Trigger` | `aa 03 09 00 ff` | 查询并清除错误代码（过流/不同步等） |
| `/desk/query_height` | `std_srvs/srv/Trigger` | `aa 03 01 00 ff` | 查询当前高度（结果经 `/desk/current_height` 发布） |

### 命令行调用示例：
```bash
# 1. 向上升
ros2 service call /desk/up std_srvs/srv/Trigger

# 2. 停止（升到合适高度后停止）
ros2 service call /desk/stop std_srvs/srv/Trigger

# 3. 向下降
ros2 service call /desk/down std_srvs/srv/Trigger

# 4. 系统复位
ros2 service call /desk/reset std_srvs/srv/Trigger

# 5. 查询错误
ros2 service call /desk/get_error std_srvs/srv/Trigger
```

---

## 4. ROS 2 话题接口列表 (Topics)

### ① 订阅控制话题（向桌子下发指令）

| 话题名称 | 消息类型 | 取值示例 | 说明 |
| :--- | :--- | :--- | :--- |
| `/desk/cmd_height` | `std_msgs/msg/Float32` | `75.0` | **直接运行到指定绝对高度**（单位：cm） |
| `/desk/cmd_pos` | `std_msgs/msg/Int32` | `1` 或 `2` | 运行到指定记忆档位（1号位、2号位等） |

#### 命令行下发示例：
```bash
# 运行到指定目标高度 75.0 cm
ros2 topic pub --once /desk/cmd_height std_msgs/msg/Float32 "{data: 75.0}"

# 运行到指定目标高度 102.5 cm
ros2 topic pub --once /desk/cmd_height std_msgs/msg/Float32 "{data: 102.5}"

# 运行到 1 号预设记忆位
ros2 topic pub --once /desk/cmd_pos std_msgs/msg/Int32 "{data: 1}"
```

---

### ② 发布状态话题（接收桌子实时高度）

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/desk/current_height` | `std_msgs/msg/Float32` | 驱动板回传的实时桌面高度（单位：cm） |

> **注意（2026-09-10 实测修正）**：高度是**应答式**的，板子不主动上传。
> 节点已内置自动轮询（默认 5Hz，参数 `poll_rate`，0 关闭），静止时也有数据。
> 也可手动触发：`ros2 service call /desk/query_height std_srvs/srv/Trigger`

#### 终端实时监听示例：
```bash
ros2 topic echo /desk/current_height
```

---

## 5. 底层 485/串口通信协议速查

| 指令功能 | 完整十六进制帧 (Hex) | 字段解析 |
| :--- | :--- | :--- |
| **上升** | `aa 03 02 00 ff` | 帧头 `aa`，长度 `03`，命令 `02`，尾 `ff` |
| **下降** | `aa 03 03 00 ff` | 命令 `03` |
| **停止** | `aa 03 04 00 ff` | 命令 `04` |
| **复位** | `aa 03 08 00 ff` | 命令 `08` |
| **查错误码** | `aa 03 09 00 ff` | 命令 `09`（回包中 `01`不同步，`02`过流） |
| **查询高度** | `aa 03 01 00 ff` | 命令 `01`，回包为高度帧 `aa 07 01 ...`（应答式，板子不主动上传） |
| **运行到高度** | `aa 07 80 30 37 35 2e 30 ff` | 命令 `80`，后接 5 位 ASCII 码（`075.0` 表示 75.0cm） |
| **运行到记忆位** | `aa 03 06 01 ff` | 命令 `06`，`01` 代表 1 号位置 |
| **高度回传帧** | `aa 07 01 30 37 35 2e 30 ff` | 命令 `01`，驱动板上传当前高度数据 |