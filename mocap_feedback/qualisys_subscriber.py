from ctypes import Array
from numpy.core.numeric import NaN
from pymavlink import mavutil
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
​
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped
import math
​
    
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
​
class PoseSubscriber(Node): 
    master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)
    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/qualisys/elvind/pose',  #Message type and topic 
            self.listener_callback,
            10)
        self.subscription=self.create_subscription(
            Int32MultiArray,
            'master',
            self.listener_callback,
            10)
​
​
    def timesync_listener(self):
        ts1=int(time.time())
        self.master.mav.timesync_send(0,ts1)
        return 
​
​
 
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
​
​
​
    def set_origin_home(self):
        self.master.mav.set_gps_global_origin_send(self.master.target_system, 0, 0, 0)
        #self.master.mav.set_home_position(self.master.target_system)
    def request_message_interval(self, message_id: int, frequency_hz: float):
​
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, # The MAVLink message ID
            1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
            0, 0, 0, 0)
​
    def get_message(self, type):
        messg = self.master.recv_match(type=type, blocking=True) ##Should not be true, fix none loggers
        if(messg != None):
            return messg
        
​
    def print_statustext(self):
        statustext =  self.get_message('STATUSTEXT').to_dict()
        if (statustext != None):
            print('Severity: %s\tText:%s'  %  (statustext['severity'],statustext['text']))
​
    #def print_mode(self):
     #   print(self.get_message('MAV_MODE'))
​
    
    def listener_callback(self, msg):
        #self.get_logger().info(str(msg.pose.orientation))  #Here the data that the node receives is printed
    
​
        #Orientation 
        
        x_a = msg.pose.orientation.x
        y_a = msg.pose.orientation.y
        z_a = msg.pose.orientation.z 
        w_a = msg.pose.orientation.w
        att_euler = euler_from_quaternion(x_a, y_a, z_a, w_a)
        
        #Pose (transformed from xyz to ned )
        x_p = msg.pose.position.x  #North
        y_p = msg.pose.position.y   #East       
        z_p = -msg.pose.position.z  #Down
        time_u = msg.header.stamp.sec
​
        self.master.mav.vision_position_estimate_send(time_u,x_p,y_p,z_p,att_euler[0],att_euler[1],att_euler[2])
​
    
​
​
​
​
​
def main(args=None):
    rclpy.init(args=args)
​
    pose_subscriber = PoseSubscriber()
    pose_subscriber.init_connection()
    pose_subscriber.set_origin_home()
    pose_subscriber.timesync_listener()

    rclpy.spin(pose_subscriber)
​

    pose_subscriber.destroy_node()
    rclpy.shutdown()
​
​
if __name__ == '__main__':
    main()
