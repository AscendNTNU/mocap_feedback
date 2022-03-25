#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import time
import tf2_ros
import tf2_geometry_msgs

master = mavutil.mavlink_connection("udpin:0.0.0.0:14551")

def callback(msg: PoseStamped):
    #time_usec = int(msg.header.stamp.secs * 1e6 + msg.header.stamp.nsecs / 1e3)
    
    transform = buffer.lookup_transform(
                                       # target frame:
                                       "local_ned",
                                       # source frame:
                                       msg.header.frame_id,
                                       # get the tf at the time the pose was valid
                                       msg.header.stamp,
                                       # wait for at most 1 second for transform, otherwise throw
                                       rospy.Duration(1.0))

    msg_transformed = tf2_geometry_msgs.do_transform_pose(msg, transform)

    time_usec = int(time.perf_counter_ns()* 1e3)

    o = msg_transformed.pose.orientation
    q = [o.w, o.x, o.y, o.z]

    p = msg_transformed.pose.position
    x, y, z = p.x, p.y, p.z

    master.mav.att_pos_mocap_send(time_usec, q, x, y, z)


def set_origin():
    master.mav.set_gps_global_origin_send(master.target_system, 0, 0, 0)
    master.mav.set_home_position_send(
        master.target_system, 0, 0, 0, 0, 0, 0, [0, 0, 0, 0], 0, 0, 0
    )


rospy.init_node("mocap_feedback")
buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)
rospy.Subscriber("/qualisys/elvind/pose", PoseStamped, callback)
set_origin()
rospy.spin()
