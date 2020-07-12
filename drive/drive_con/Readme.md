# PID Controller
## Pid implementation for the autonomous navigation of the rover 
To run the controller in the gazebo simulation start roscore and run the following commands on the terminal

To start turtlebot simulation in gazebo
``` 
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
Run the action server in 1 terminal 
```
rosrun drive_con action_server.py
```
run action_client in another terminal
```
rosrun drive_con action_client.py
```
The rostopic list will show the different topics that are publishing data from the bot like distance , odometry, cmd_vel and actionserver topics can be found 
The topics data can be seen using:
```
rostopic echo <topic name>
```
Further the controller needs to be interfaced to the arduino mega using pyyserial and the linear and angular velocities from the cmd_vel topic needs to be converted to the individual wheel velocities.  
