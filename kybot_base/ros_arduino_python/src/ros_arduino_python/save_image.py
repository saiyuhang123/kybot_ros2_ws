#!/usr/bin/env python3

import os
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class PictureSaver(Node):
    def __init__(self):
        super().__init__('save_picture_node_fixed')

        # 保存目录：~/Image
        home_dir = os.path.expanduser("~")
        self.output_dir = os.path.join(home_dir, "Image")
        if not os.path.isdir(self.output_dir):
            try:
                os.makedirs(self.output_dir)
                self.get_logger().info("Created directory: %s" % self.output_dir)
            except OSError as e:
                self.get_logger().error("Failed to create dir %s: %s" % (self.output_dir, e))

        # 文件命名配置
        self.prefix = "shot"
        self.ext = ".jpg"   # 可改为 ".png"
        self.jpeg_quality = 95
        self.png_compress = 3
        self.seq = 0

        # 缓存图像
        self.lock = threading.Lock()
        self.last_img = None

        # 使用 CvBridge 将 ROS 格式的图像转为 OpenCV 格式的图像
        self.bridge = CvBridge()

        # 订阅双目相机的 rgb 图像主题
        self.image_sub = self.create_subscription(
            Image,
            "/camera/rgb/image_raw",
            self.Imagecallback,
            1
        )

        # 订阅拍照事件
        self.event_sub = self.create_subscription(
            Empty,
            "/take_picture/event",
            self.EventCallback,
            10
        )

        self.get_logger().info("PictureSaver ready, saving to %s" % self.output_dir)

    def Imagecallback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn("cv_bridge error: %s" % e)
            return
        with self.lock:
            self.last_img = (cv_img, msg.header.stamp)

    def EventCallback(self, _):
        with self.lock:
            bundle = self.last_img
        if bundle is None:
            self.get_logger().warn("No image yet, skip saving.")
            return

        cv_img, stamp = bundle
        self.seq += 1

        # 文件名：shot_YYYYmmdd_HHMMSS_mmm_nXXXXXX.jpg
        t = datetime.fromtimestamp(stamp.sec + stamp.nanosec / 1e9)
        fname = "{}_{}_n{:06d}{}".format(
            self.prefix, t.strftime("%Y%m%d_%H%M%S_%f")[:-3], self.seq, self.ext
        )
        fpath = os.path.join(self.output_dir, fname)

        # 保存图像
        if self.ext.lower() in [".jpg", ".jpeg"]:
            ok = cv2.imwrite(fpath, cv_img,
                             [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        elif self.ext.lower() == ".png":
            ok = cv2.imwrite(fpath, cv_img,
                             [int(cv2.IMWRITE_PNG_COMPRESSION), self.png_compress])
        else:
            ok = cv2.imwrite(fpath, cv_img)

        if ok:
            self.get_logger().info("Saved: %s" % fpath)
        else:
            self.get_logger().error("Failed to save: %s" % fpath)


def main(args=None):
    rclpy.init(args=args)
    node = PictureSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
