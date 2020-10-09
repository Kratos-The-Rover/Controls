#!/usr/bin/env python

# File created to test the PID controller by passing a point entered by the user

import rospy
from geometry_msgs.msg import Point

rospy.init_node("path_planner")
pub = rospy.Publisher("path", Point, queue_size=5)
rate = rospy.Rate(10)
point = Point()
point.x = float(raw_input("Enter x coordinate of goal:"))
point.y = float(raw_input("Enter y coordinate of goal:"))
point.z = 0
while not rospy.is_shutdown():
    pub.publish(point)
    rate.sleep()
