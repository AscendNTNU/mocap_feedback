#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import sys
from pymavlink import mavutil

i = 0  # number of messages since last message sent
N = int(sys.argv[1])  # send every N msg, throw the rest


def callback(msg):
    global i
    i += 1
    if i == 2:
        publisher.publish(msg)
        i = 0


def set_origin():
    master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)
    master.mav.set_gps_global_origin_send(master.target_system, 0, 0, 0)
    master.mav.set_home_position_send(master.target_system, 0, 0, 0, 0)


rospy.init_node("mocap_feedback")
rospy.Subscriber("/qualisys/elvind/pose", PoseStamped, callback)
publisher = rospy.Publisher("/mavros/vision/pose", PoseStamped, queue_size=1)
set_origin()
rospy.spin()
