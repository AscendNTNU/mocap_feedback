# Import mavutil
from pymavlink import mavutil
import sys
import time
import numpy as np


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

# Create the connection
master = mavutil.mavlink_connection("/dev/cu.usbserial-0001", baud=57600)
wait_conn()
print("Waiting for heartbeat")
master.wait_heartbeat()
print("Setting origin SET_GPS_GLOBAL_ORIGIN to initialize EKF (0,0,0)")
master.mav.set_gps_global_origin_send(master.target_system, 0, 0, 0)

#
#   Arm
#
print("Feeding some data")

#data = master.mav.att_pos_mocap_encode(time_usec=time.time(),q=[1, 0, 0, 0],x=0,y=0,z=0)

master.mav.att_pos_mocap_send(np.uint64(time.time()),[1.0, 0.0, 0.0, 0.0],0.0,0.0,0.0) #Time is epoch time

#print(data)
#master.mav.att_pos_mocap_send(data)


    

