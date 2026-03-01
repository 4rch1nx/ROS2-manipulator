# ROS2-manipulator  
## Install micro_ros_agent  
1. Install build tools and dependencies  
    ```
    sudo apt update
    sudo apt install -y \
        git \
        cmake \
        build-essential \
        libspdlog-dev \
        libboost-dev \
        libssl-dev \
        libelfin-dev \
        python3-dev \
        libpoco-dev
    ```
2. Create a Colcon Workspace
    ```
    mkdir -p ~/microros_ws/src
    cd ~/microros_ws/src
    ```
3. Clone the Repository  
`git clone -b main https://github.com/micro-ROS/micro-ROS-Agent.git`
4. Build the Agent  
    ```
    source /opt/ros/jazzy/setup.bash

    cd ~/microros_ws

    colcon build
    ```
5. Configure Udev Rules (Manual Copy)  
    ```
    sudo cp ~/microros_ws/src/micro-ROS-Agent/udev/* /etc/udev/rules.d/

    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
6. Source the Workspace  
`source ~/microros_ws/install/setup.bash`  
To make this permanent, add it to your .bashrc:
    ```
    echo "source ~/microros_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
## Control ESP32
### Serial (USB)
Upload `esp32_wifi_servo.ino` to your ESP32  
Start connection in terminal window  
`ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6`  
Control with `ros2 topic pub -1 /servo_control std_msgs/msg/Int32MultiArray "{data: [4, 2048, 1000]}" --once`, where 4 - servo ID, 2048 - coords, 1000 - speed  
### UDP (WiFi/Ethernet)
Upload `robot-arm-so101.ino` to your ESP32  
- Run agent with UDP (port 8888)  
`ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6`  
- Or specify IP (optional)  
`ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 --host [YOUR_PC_IP] -v6`  

>**_Note: Don't use mDNS, ESP32 won't connect to agent!_**
### TCP
Not supported yet.  
`ros2 run micro_ros_agent micro_ros_agent tcp4 -p 8888`
