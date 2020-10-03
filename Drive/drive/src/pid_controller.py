#!/usr/bin/env python
import rospy
from math import pi, sqrt, cos, atan2
from utils import *

# this is a custom PID implementation(first rotate then translate)
class PIDController:
    # goal controller uses the current odometry and the goal position and provides with the velocity

    def __init__(self):
        self.a_i=0
        self.a_prev=0
        self.e_i=0
        self.e_prev=0
        self.kP = rospy.get_param("kP", 3.0)
        self.kI = rospy.get_param("kI", 0)
        self.kD = rospy.get_param("kD", 0)
        self.kP_r = rospy.get_param("kP_r", 3.0)
        self.kI_r = rospy.get_param("kI_r", 0)
        self.kD_r = rospy.get_param("kD_r", 0)
        self.max_linear_speed = rospy.get_param("max_linear_speed", 0.2)
        self.min_linear_speed = rospy.get_param("min_linear_speed", 0.001)
        self.max_angular_speed = rospy.get_param("max_angular_speed", 1.0)
        self.min_angular_speed = rospy.get_param("min_angular_speed", 0.001)
        self.linear_tolerance = rospy.get_param("linear_tolerance", 0.1)  # 2.5cm
        self.angular_tolerance =rospy.get_param("angular_tolerance", 0.053) # 3 degrees
        self.forward_movement_only = rospy.get_param("forwardMovementOnly", False)


    def get_velocity(self,current_pose, goal):
        desired = Pose()
        goal_heading = atan2(goal.y - current_pose.y, goal.x -current_pose.x)
        if self.forward_movement_only:
            direction = 1
            goal_heading = normalize_pi(goal_heading)
        else:
            direction = sign(cos(goal_heading))
            goal_heading = normalize_half_pi(goal_heading)

        self.a=goal_heading-current_pose.theta
        self.e=get_goal_distance(current_pose,goal)

        if abs(self.a) > self.angular_tolerance:
            print self.max_linear_speed
            self.a_i+=self.a
            self.a_d=self.a-self.a_prev
            desired.x = 0
            desired.theta = self.kP_r*self.a +self.kI_r*self.a_i +self.kD_r*self.a_d
        elif abs(self.e) > self.linear_tolerance:
            self.e_i+=self.e
            self.e_d=self.e-self.e_prev
            desired.x = (self.kP*self.e +self.kI*self.e_i +self.kD*self.e_d)*direction
            desired.theta =0
        else:
            desired.x = 0
            desired.theta =0

        self.a_prev=self.a
        self.e_prev=self.e
        # Adjust velocities if X velocity is too high.
        # Adjust velocities if too low, so robot does not stall.
        if abs(desired.x) > self.max_linear_speed:
            desired.x = self.max_linear_speed*direction
        elif abs(desired.x) > 0 and abs(desired.x) < self.min_linear_speed:
            desired.x = self.min_linear_speed*direction

        # Adjust velocities if turning velocity too high.
        if abs(desired.theta)>0 and abs(desired.theta) < self.min_angular_speed:
            desired.theta = self.min_angular_speed
        elif abs(desired.theta) > self.max_angular_speed:
            desired.theta = self.max_angular_speed    

        return desired


