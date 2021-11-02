#This node is listening to the topic that the pose is published to 
#It is used to feed the pose to the drone via MAVLink 

# Import mavutil 
from pymavlink import mavutil
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.mgs import PoseStamped #This is the message type of the pose 


def wait_conn():
    """
    Sends a ping to estabilish serial communication and waits for a response
    """
    msg = None
    while not msg:
        master.mav.ping_send(
            int(time.time() * 1e6), # Unix time in microseconds
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)
        


#print("Setting origin SET_GPS_GLOBAL_ORIGIN to initialize EKF (0,0,0)")

#
#   Arm
#
#print("Feeding some data")

#data = master.mav.att_pos_mocap_encode(time_usec=time.time(),q=[1, 0, 0, 0],x=0,y=0,z=0)

#master.mav.att_pos_mocap_send(np.uint64(time.time()),[1.0, 0.0, 0.0, 0.0],0.0,0.0,0.0) #Time is epoch time


class PoseSubscriber(Node): 
    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/qualisys/marlin/pose geometry_msgs/PoseStamped',  #Message type and topic 
            self.listener_callback,
            10)
        self.master 
    
    def init_connection(self):
        self.master = mavutil.mavlink_connection("/dev/cu.usbserial-0001", baud=57600)
        wait_conn()
        self.master.wait_heartbeat()

    def set_origin(self):
        self.master.mav.set_gps_global_origin_send(self.master.target_system, 0, 0, 0)

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)  #Here the data that the node receives is printed
    



def main(args=None):
    rclpy.init(args=args)

    pose_subscriber = PoseSubscriber()
    pose_subscriber.init_connection()
    #Etabler connection 

    rclpy.spin(pose_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
