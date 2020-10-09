## 1. Using the package:
Clone the full repo in src folder of your ROS workspace

```bash
cd ~/name-of-ws/
catkin build
source devel/setup.bash
roslaunch drive husky.launch
```

These set of commands will launch Husky in empty gazebo world and open a new terminal window where the goal coordinates need to be entered.

The bot navigates to the entered point using PID control.

## 2. Changing the parameters:

```bash
cd ~/name-of-ws/src/Controls/Drive/drive/config
```

Open the params.yaml file in your preffered code editor.

Change the values you want to change and repeat the steps in section 1 to see the results.

## 3. Adding a path planner to the stack

```bash
cd ~/name-of-ws/src/Controls/Drive/drive/src
```

Create a ROS node in the src folder of the package. 

The node should publish geometry_msgs/Point values to the topic /path.

```bash
cd ~/name-of-ws/src/Controls/Drive/drive/launch
```

Open the husky.launch file in your preffered code editor

Replace test_path_planner.py with the name of your file and remove " launch-prefix="gnome-terminal --command" "
on line 9 if you don't want the node's outputs to be dispalyed in a new terminal window.

Repeat the steps in section 1 to see the results.

