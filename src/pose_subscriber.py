#This node is listening to the topic that the pose is published to 
#It is used to feed to pose to the drone via MAVLink 

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.mgs import PoseStamped #This is the message type of the pose 


class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/qualisys/marlin/pose geometry_msgs/PoseStamped',  #Message type and topic 
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)  #Here the data that the node receives is printed


def main(args=None):
    rclpy.init(args=args)

    pose_subscriber = PoseSubscriber()

    rclpy.spin(pose_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
