# ROS2-manipulator  
## Install micro_ros_agent  
to do...  
## ESP32
### Via cable
Start connection in terminal window `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6`  
Control with `ros2 topic pub -1 /servo_control std_msgs/msg/Int32MultiArray "{data: [4, 2048, 1000]}" --once`, where 4 - servo ID, 2048 - coords, 1000 - speed  
### WiFi  
- Run agent with UDP (port 8888)  
`ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6`  
- Or specify IP (optional)  
`ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 --host [YOUR_PC_IP] -v6`  

## mDNS on agent  
install  
```
sudo apt update
sudo apt install avahi-daemon
sudo systemctl enable avahi-daemon
sudo systemctl start avahi-daemon
```
test with `ping <hostname>.local`  
