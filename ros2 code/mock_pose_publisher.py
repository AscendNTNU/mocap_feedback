import rclpy
from rclpy.node import Node, Timer
from geometry_msgs.msg import PoseStamped
import time
import math
import numpy as np

class MockPosePublisher(Node):
    i : int
    timer : Timer

    def __init__(self):
        super().__init__('mock_pose_publisher')

        self.publisher = self.create_publisher(
            PoseStamped,
            'pose_topic',
            0
        )
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_pose)
        self.i = 0

    def publish_pose(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "mav_local_enu"
        msg.pose.position.x = 50.0*math.sin(self.i)
        msg.pose.position.y = 50.0*math.cos(self.i) * 0
        msg.pose.position.z = -10 * float(self.i)

        msg.pose.orientation.x = .0
        msg.pose.orientation.y = .0
        msg.pose.orientation.z = float(np.sin(0.5*self.i))
        msg.pose.orientation.w = float(np.cos(0.5*self.i))

        self.publisher.publish(msg)

        self.i += 0.05

def main(args=None):
    rclpy.init(args=args)

    pose_publisher = MockPosePublisher()

    rclpy.spin(pose_publisher)

    pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()