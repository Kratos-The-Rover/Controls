## Drive Subsystem

### Steps to setup the system

#### 1) SSH into the onboard computer 
Connect your system and the onboard computer on the same network.
```
ssh -Y <username_of_onboard_pc>@<ip_of_onboard_pc>
```
You can find the ip alloted to the onboard computer using nmap which can be installed using ``sudo apt-get install nmap``
```
nmap -sn <if_of_your_pc>/24
```
You can find the ip of your system using ifconfig
```
ifconfig
```

#### 2) Run the serial node to transfer data between Arduino node and ROS Master of your system

Give permissions to all the ACM ports.
```
sudo chmod 777 ttyACM*
```
Run the serial node.
```
rosrun rosserial_python serial_node.py _port:="<name_of_the_port>"
```

#### 3) You may use the turtlebot_teleop node for manual control.
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
