# 双目深度相机（Orbbec Gemini 305 × 2）使用文档

## 一、硬件与安装

| 项目 | 左目 | 右目 |
|---|---|---|
| 型号 | Gemini 305 | Gemini 305 |
| 序列号 | `CV2L360000A4` | `CV2L360000R4` |
| USB 端口 | 2-3.3.4 | 2-3.3.3 |

- 必须插 **USB 3.0**（蓝色/SS 口），USB 2.0 带宽不够深度流
- 相机按**序列号**区分，换 USB 口插不影响左右分配
- udev 规则已装：`/etc/udev/rules.d/99-obsensor-libusb.rules`（免 root 访问）

## 二、启动

每个新终端先：

```bash
source /home/kybot/kybot_ros2_ws/install/setup.bash
```

### 双目同时启动（推荐）

```bash
ros2 launch orbbec_camera dual_gemini305.launch.py
```

序列号写死在 `OrbbecSDK_ROS2/orbbec_camera/launch/dual_gemini305.launch.py`，
也可临时覆盖：`ros2 launch orbbec_camera dual_gemini305.launch.py left_serial:=xxx right_serial:=yyy`

### 单目启动

```bash
ros2 launch orbbec_camera gemini_301_series.launch.py                        # 默认第一台
ros2 launch orbbec_camera gemini_301_series.launch.py serial_number:=CV2L360000A4  # 指定序列号
```

## 三、话题（每台相机一套，前缀区分）

| 话题 | 内容 |
|---|---|
| `/camera_left/color/image_raw` | 彩色图（30fps） |
| `/camera_left/depth/image_raw` | 深度图（16 位单通道，30fps） |
| `/camera_left/depth/points` | 深度点云 |
| `/camera_left/*/camera_info` | 相机内参 |
| `/camera_left/device_status` | 设备状态 |

右目把 `camera_left` 换成 `camera_right`。

TF 坐标系各自独立：`camera_left_link`、`camera_left_depth_optical_frame` 等
（`*_optical_frame` 是标准光学系：z 朝前、x 朝右、y 朝下）。

## 四、常用命令

```bash
# 列出已连接的 Orbbec 设备及序列号
bash OrbbecSDK_ROS2/orbbec_camera/scripts/list_ob_devices.sh

# 看帧率
ros2 topic hz /camera_left/depth/image_raw

# 看画面（rqt）
ros2 run rqt_image_view rqt_image_view /camera_left/color/image_raw
ros2 run rqt_image_view rqt_image_view /camera_right/color/image_raw

# 验证 TF
ros2 run tf2_ros tf2_echo camera_left_link camera_left_depth_optical_frame
```

## 五、验证与标定

- **物理左右确认**：RViz/rqt 打开 `/camera_left/color/image_raw`，在左手相机前晃手，
  画面对不上就交换 launch 文件里的两个序列号。
- **手眼标定**：相机装在机械臂末端/手上时，需要标定
  `arm_left 末端 -> camera_left_link` 的外参（暂未做）。
- 深度图在 rqt 里显示偏暗/偏色是正常的（16 位单通道）。

## 六、排障

| 现象 | 排查 |
|---|---|
| lsusb 看不到相机 | 换 USB3 口；`sudo dmesg -w` 重插看内核日志；查线 |
| 只有一台能识别 | 两台不要挂同一级 USB Hub，换到不同物理口 |
| 帧率低/掉帧 | 确认在 USB3 口（lsusb -t 应显示 5000M） |
| 无权限打开设备 | 确认 udev 规则已装并重插相机 |
| launch 报找不到设备 | `list_ob_devices.sh` 核对序列号是否与 launch 中一致 |


