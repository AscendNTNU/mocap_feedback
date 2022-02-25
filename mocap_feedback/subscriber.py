#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import sys

i = 0  # number of messages since last message sent
N = int(sys.argv[1])  # send every N msg, throw the rest


def callback(msg):
    i += 1
    if i == 2:
        publisher.publish(msg)
        i = 0


rospy.init_node("mocap_feedback")
rospy.Subscriber("/qualisys/elvind/pose", PoseStamped, callback)
publisher = rospy.Publisher("/mavros/vision/pose", PoseStamped, queue_size=1)
rospy.spin()
