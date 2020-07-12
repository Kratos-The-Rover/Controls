#!/usr/bin/env python
import rospy
import actionlib
from drive_con.msg import action_demoAction,action_demoResult,action_demoGoal
rospy.init_node("action_client")
client = actionlib.SimpleActionClient('action_demo', action_demoAction)
client.wait_for_server()
rospy.loginfo("Enter the goal to reach")
goal = action_demoGoal()
goal.x = float(input())
goal.y = float(input())
goal.theta = float(input())
client.send_goal(goal)
client.wait_for_result()
print("Goal achieved")