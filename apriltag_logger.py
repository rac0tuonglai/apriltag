import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AprilTagLogger(Node):
    def __init__(self):
        super().__init__('apriltag_logger')

        self.subscription = self.create_subscription(
            String,
            '/my_apriltag_info',
            self.callback,
            10
        )

        self.get_logger().info('apriltag_logger node started.')

    def callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            tag_id = data.get("id", -1)
            x = data.get("x", 0.0)
            y = data.get("y", 0.0)
            z = data.get("z", 0.0)

            self.get_logger().info(
                f"Detected Tag {tag_id} at ({x:.2f}, {y:.2f}, {z:.2f})"
            )
        except json.JSONDecodeError:
            self.get_logger().warn("Received invalid JSON.")


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
