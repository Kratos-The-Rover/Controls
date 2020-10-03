#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
rospy.init_node('path_planner')
pub=rospy.Publisher('path',Point,queue_size=5)
rate=rospy.Rate(10)
point=Point()
point.x=1
point.y=1
point.z=0
while not rospy.is_shutdown():
    pub.publish(point)
    rate.sleep()