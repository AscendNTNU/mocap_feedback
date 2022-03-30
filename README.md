# mocap_feedback
Repo for the motion capture system (qualisys) in the drone hall. 

## How to use the system 
- Start a `roscore`on your computer (Ubuntu). 

### Qualisys (Windows 10) 
- Login with PIN 0052
- Start the Qualisys software, define a body and start recording.  
- Open a terminal and start by sourcing the local workspace setup.
```
devel\setup.bat
```
- Set the ROS_MASTER_URI
```
setx ROS_MASTER_URI "http://xxx.xxx.x.xxx:xxxxx"
```
- Run the qualisys node 
```
roslaunch mocap_qualisys qualisys.launch
```

### On the Ubuntu computer 
- If you want to use QGroundControl 

```
./mavproxy --master=/dev/ttyUSB0 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
```
- To transform to the correct frame 
```
rosrun tf2_ros static_transform_publisher 0 0 0 0.7071067811865476 0.7071067811865476 0 0 mocap local_ned
```
- Go in to your ROS workspace and run the mocap node 
```
rosrun mocap_feedback subscriber.py
```
Now you should be able to arm and take off. 






