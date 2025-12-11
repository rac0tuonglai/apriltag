import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray

class AprilTagPublisher(Node):
    def __init__(self):
        super().__init__('apriltag_publisher')

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',  # adjust if your topic name is different
            self.detections_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            '/my_apriltag_info',
            10
        )

        self.get_logger().info('apriltag_publisher node started.')

    def detections_callback(self, msg: AprilTagDetectionArray):
        if not msg.detections:
            return

        det = msg.detections[0]

        # Tag ID extraction
        try:
            tag_id = int(det.id[0])
        except Exception:
            tag_id = -1

        try:
            pos = det.pose.pose.pose.position
            x, y, z = float(pos.x), float(pos.y), float(pos.z)
        except Exception:
            self.get_logger().warn('Could not extract pose.')
            return

        payload = {"id": tag_id, "x": x, "y": y, "z": z}

        out = String()
        out.data = json.dumps(payload)

        self.publisher.publish(out)

        self.get_logger().info(f"Published: {out.data}")


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
