import socket
import struct
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraReceiver(Node):
    def __init__(self):
        super().__init__('camera_receiver')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Connect to Windows server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(("127.0.0.1", 9000))

        self.get_logger().info("Connected to Windows webcam.")

        self.timer = self.create_timer(0.01, self.read_frame)

    def read_frame(self):
        # Receive length
        header = self.sock.recv(4)
        if not header:
            return
        length = struct.unpack(">I", header)[0]

        # Receive image data
        data = b''
        while len(data) < length:
            data += self.sock.recv(length - len(data))

        # Decode frame
        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
