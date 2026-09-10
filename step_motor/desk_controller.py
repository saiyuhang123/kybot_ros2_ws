#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32, Int32
import serial
import threading
import time

class DeskController(Node):
    def __init__(self):
        super().__init__('desk_controller')

        # 1. 声明并获取参数（默认匹配你的设置）
        self.declare_parameter('port', '/dev/ttyDESK')          # 已绑定 udev 别名 ttyDESK (CH343, SN 575A018668)
        self.declare_parameter('baudrate', 38400)       # 38400 波特率
        self.declare_parameter('poll_rate', 5.0)        # 高度自动轮询频率 Hz，0 = 关闭

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # 2. 初始化串口
        self.serial_lock = threading.Lock()
        self.ser = None
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.get_logger().info(f'成功打开串口: {port}, 波特率: {baudrate}')
        except Exception as e:
            self.get_logger().error(f'无法打开串口: {e}')
            return

        # 3. 创建 ROS 2 服务 (Trigger 类型)
        self.srv_up = self.create_service(Trigger, '/desk/up', self.handle_up)
        self.srv_down = self.create_service(Trigger, '/desk/down', self.handle_down)
        self.srv_stop = self.create_service(Trigger, '/desk/stop', self.handle_stop)
        self.srv_reset = self.create_service(Trigger, '/desk/reset', self.handle_reset)
        self.srv_get_err = self.create_service(Trigger, '/desk/get_error', self.handle_get_error)
        self.srv_query_height = self.create_service(Trigger, '/desk/query_height', self.handle_query_height)

        # 4. 创建话题订阅 (用于输入目标高度和档位)
        self.sub_cmd_height = self.create_subscription(
            Float32, '/desk/cmd_height', self.handle_cmd_height, 10
        )
        self.sub_cmd_pos = self.create_subscription(
            Int32, '/desk/cmd_pos', self.handle_cmd_pos, 10
        )

        # 5. 创建话题发布者 (发布实时高度)
        self.pub_height = self.create_publisher(Float32, '/desk/current_height', 10)

        # 6. 启动后台线程监听串口回包
        self.running = True
        self.rx_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.rx_thread.start()

        # 7. 高度自动轮询（板子不主动上传，需查询触发回传）
        poll_rate = self.get_parameter('poll_rate').value
        if poll_rate > 0:
            self.create_timer(
                1.0 / poll_rate,
                lambda: self.send_command(bytes([0xAA, 0x03, 0x01, 0x00, 0xFF]), log=False)
            )
            self.get_logger().info(f'高度轮询已开启: {poll_rate} Hz')

        self.get_logger().info('升降桌 ROS 2 控制服务已就绪！')

    # ========== 串口收发底层封装 ==========
    def send_command(self, cmd_bytes: bytes, log: bool = True):
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                self.ser.write(cmd_bytes)
                if log:
                    self.get_logger().info(f'已发送报文: {cmd_bytes.hex(" ")}')
            else:
                self.get_logger().warn('串口未打开，发送失败')

    # ========== 服务回调函数 ==========
    def handle_query_height(self, request, response):
        """查询当前高度（应答式协议：查询后板子回传，经 /desk/current_height 发布）"""
        self.send_command(bytes([0xAA, 0x03, 0x01, 0x00, 0xFF]))
        response.success = True
        response.message = "高度查询命令已下发，结果见 /desk/current_height"
        return response

    def handle_up(self, request, response):
        """上升指令"""
        self.send_command(bytes([0xAA, 0x03, 0x02, 0x00, 0xFF]))
        response.success = True
        response.message = "上升命令已下发"
        return response

    def handle_down(self, request, response):
        """下降指令"""
        self.send_command(bytes([0xAA, 0x03, 0x03, 0x00, 0xFF]))
        response.success = True
        response.message = "下降命令已下发"
        return response

    def handle_stop(self, request, response):
        """停止指令"""
        self.send_command(bytes([0xAA, 0x03, 0x04, 0x00, 0xFF]))
        response.success = True
        response.message = "停止命令已下发"
        return response

    def handle_reset(self, request, response):
        """系统复位"""
        self.send_command(bytes([0xAA, 0x03, 0x08, 0x00, 0xFF]))
        response.success = True
        response.message = "系统复位命令已下发"
        return response

    def handle_get_error(self, request, response):
        """获取错误码"""
        self.send_command(bytes([0xAA, 0x03, 0x09, 0x00, 0xFF]))
        response.success = True
        response.message = "已发送查询错误码命令"
        return response

    # ========== 话题回调函数 ==========
    def handle_cmd_height(self, msg: Float32):
        """运行到指定高度（例如 75.0 cm）"""
        target = max(0.0, min( msg.data, 200.0)) # 限制安全范围
        # 格式化为 5 位 ASCII 码，例如 75.0 -> '075.0'
        ascii_str = f"{target:05.1f}"
        cmd = bytes([0xAA, 0x07, 0x80]) + ascii_str.encode('ascii') + bytes([0xFF])
        self.send_command(cmd)

    def handle_cmd_pos(self, msg: Int32):
        """运行到第 n 号记忆位（例如 1 或 2）"""
        pos_id = int(msg.data) & 0xFF
        cmd = bytes([0xAA, 0x03, 0x06, pos_id, 0xFF])
        self.send_command(cmd)

    # ========== 串口数据解析后台线程 ==========
    def receive_loop(self):
        buffer = bytearray()
        while self.running and self.ser and self.ser.is_open:
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    data = self.ser.read(waiting)
                    buffer.extend(data)

                    # 寻找完整帧 (以 0xAA 开头，0xFF 结尾)
                    while len(buffer) >= 5:
                        if buffer[0] != 0xAA:
                            buffer.pop(0)
                            continue

                        # 检查是否有帧尾
                        if 0xFF in buffer:
                            end_idx = buffer.index(0xFF)
                            frame = buffer[:end_idx + 1]
                            buffer = buffer[end_idx + 1:]
                            self.parse_frame(frame)
                        else:
                            break
                time.sleep(0.02)
            except Exception as e:
                self.get_logger().error(f'接收异常: {e}')
                break

    def parse_frame(self, frame: bytearray):
        """解析驱动板上传的数据"""
        if len(frame) < 5 or frame[0] != 0xAA or frame[-1] != 0xFF:
            return

        cmd = frame[2]
        # 命令 0x01: 驱动板上传当前高度 (帧长 9 字节: AA 07 01 D0 D1 D2 D3 D4 FF)
        if cmd == 0x01 and len(frame) == 9:
            try:
                height_str = frame[3:8].decode('ascii')
                height_val = float(height_str)
                msg = Float32()
                msg.data = height_val
                self.pub_height.publish(msg)
            except Exception:
                pass
        # 命令 0x09: 返回错误码
        elif cmd == 0x09 and len(frame) == 5:
            err_code = frame[3]
            err_map = {0: "正常 (无错误)", 1: "不同步", 2: "过流"}
            err_desc = err_map.get(err_code, f"未知错误代码 {err_code}")
            self.get_logger().warn(f'【驱动板状态/错误码】: {err_desc}')

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DeskController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()