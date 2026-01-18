# ROS2-manipulator  
## ESP32  
Start connection in terminal window `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6`  
Control with `ros2 topic pub -1 /servo_control std_msgs/msg/Int32MultiArray "{data: [4, 2048, 1000]}" --once`, where 4 - servo ID, 2048 - coords, 1000 - speed  

## mDNS on agent  
install  
```
sudo apt update
sudo apt install avahi-daemon
sudo systemctl enable avahi-daemon
sudo systemctl start avahi-daemon
```
test with `ping <hostname>.local`  
