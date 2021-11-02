from pymavlink import mavutil
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


        


#print("Setting origin SET_GPS_GLOBAL_ORIGIN to initialize EKF (0,0,0)")

#
#   Arm
#
#print("Feeding some data")

#data = master.mav.att_pos_mocap_encode(time_usec=time.time(),q=[1, 0, 0, 0],x=0,y=0,z=0)

#master.mav.att_pos_mocap_send(np.uint64(time.time()),[1.0, 0.0, 0.0, 0.0],0.0,0.0,0.0) #Time is epoch time


class PoseSubscriber(Node): 
    master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)
    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/qualisys/elvind/pose',  #Message type and topic 
            self.listener_callback,
            10)

    def init_connection(self):
        print("Waiting for connection")
        self.wait_conn()
        print("Connection established")
        self.master.wait_heartbeat()
        print("Heartbeat recieved")
    def wait_conn(self): #Ping untill connection is established
        msg = None
        while not msg:
            self.master.mav.ping_send(
                int(time.time() * 1e6), # Unix time in microseconds
                0, # Ping number
                0, # Request ping of all systems
                0 # Request ping of all components
            )
            print(".")
            msg = self.master.recv_match()
            time.sleep(0.5)


    def set_origin(self):
        self.master.mav.set_gps_global_origin_send(self.master.target_system, 0, 0, 0)

    def listener_callback(self, msg):
        #self.get_logger().info(str(msg.pose.orientation))  #Here the data that the node receives is printed
        #Attitude
        x_a = msg.pose.orientation.x
        y_a = msg.pose.orientation.y
        z_a = msg.pose.orientation.z
        w_a = msg.pose.orientation.w
        quat_a = [x_a,y_a,z_a,w_a]
        
        #Pose
        x_p = msg.pose.position.x
        y_p = msg.pose.position.y
        z_p = msg.pose.position.z        
        time_u = np.uint64(time.time())
        self.master.mav.att_pos_mocap_send(time_u,quat_a,x_p,y_p,z_p)
        #self.master.mav.vision_position_estimate_send(time_u,x_p,y_p,z_p,x_a,y_a,z_a)



def main(args=None):
    rclpy.init(args=args)

    pose_subscriber = PoseSubscriber()
    pose_subscriber.init_connection()
    pose_subscriber.set_origin()
    #Etabler connection 

    rclpy.spin(pose_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
