from typing import Any
import numpy as np
import time
import threading
import math
from pymavlink import mavutil

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped




class PoseForwarder(Node):
    latest_pose = PoseStamped()

    master : Any
    i : int


    def __init__(self):
        super().__init__('pose_forwarder')

        # Setup MAVLink connection
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
        self.get_logger().info("Waiting for MAVLink heartbeat")
        self.wait_conn()
        self.i = 0
        self.get_logger().info("Connected over MAVLink")

        # Initial "fake" GPS position
        init_pos = [599168469, 107284696, 0]
        # Set initial GPS position
        self.master.mav.set_gps_global_origin_send(self.master.target_system, init_pos[0], init_pos[1], init_pos[2])

        self.subscription = self.create_subscription(
            PoseStamped,
            'pose_topic',
            self.pose_callback,
            10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.publish_pose)
        self.i = 0

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg
        self.get_logger().info("Got POSE")

    def publish_pose(self):
        time_usec = int(round(time.time() * 1e6))
        self.master.mav.att_pos_mocap_send( 
             time_usec, 
            [
                self.latest_pose.pose.orientation.w, 
                self.latest_pose.pose.orientation.x, 
                self.latest_pose.pose.orientation.y,
                self.latest_pose.pose.orientation.z,
            ], 
            self.latest_pose.pose.position.x,
            self.latest_pose.pose.position.y,
            self.latest_pose.pose.position.z)


    def wait_conn(self):
        """
        Sends a ping to stabilish the UDP communication and awaits for a response
        """
        msg = None
        while not msg:
            self.master.mav.ping_send(
                int(time.time() * 1e6), # Unix time in microseconds
                0, # Ping number
                0, # Request ping of all systems
                0 # Request ping of all components
            )
            msg = self.master.recv_match()
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)

    pose_forwarder = PoseForwarder()

    rclpy.spin(pose_forwarder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_forwarder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()