#!/usr/bin/env python
import rospy
from math import pi, sqrt, sin, cos, atan2
from geometry_msgs.msg import PoseStamped

# this is a custom PID implementation
class PIDController:
    # goal controller uses the current odometry and the goal position and provides with the velocity

    def __init__(self):
        self.kP = rospy.get_param("~kP", 3.0)
        self.kA = rospy.get_param("~kA", 8.0)
        self.kB = rospy.get_param("~kB", -1.5)
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.2)
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.0)
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0)
        self.max_linear_acceleration = rospy.get_param("~max_linear_acceleration", 0.1)
        self.max_angular_acceleration = rospy.get_param("~max_angular_acceleration", 0.3)
        self.linear_tolerance = rospy.get_param("~linear_tolerance", 0.1)  # 2.5cm
        self.angular_tolerance = rospy.get_param("~angular_tolerance", 10 / 180 * pi)  # 3 degrees
        self.forward_movement_only = rospy.get_param("~forwardMovementOnly", True)

    def get_goal_distance(self, x, y, theta, goal):
        if goal is None:
            return 0
        diffX = x - goal.x
        diffY = y - goal.y
        return sqrt(diffX * diffX + diffY * diffY)

    def at_goal(self, x, y, theta, goal):
        if goal is None:
            return True
        d = self.get_goal_distance(x, y, theta, goal)
        dTh = abs(self.normalize_pi(theta - goal.theta))
        return d < self.linear_tolerance and dTh < self.angular_tolerance

    def get_velocity(self, x, y, theta, goal, dT):
        desired = PoseStamped()

        goal_heading = atan2(goal.y - y, goal.x - x)
        a = -theta + goal_heading

        # In Automomous Mobile Robots, they assume theta_G=0. So for
        # the error in heading, we have to adjust theta based on the
        # (possibly non-zero) goal theta.
        theta_1 = self.normalize_pi(theta - goal.theta)
        b = -theta_1 - a

        d = self.get_goal_distance(x, y, theta, goal)
        if self.forward_movement_only:
            direction = 1
            a = self.normalize_pi(a)
            b = self.normalize_pi(b)
        else:
            direction = self.sign(cos(a))
            a = self.normalize_half_pi(a)
            b = self.normalize_half_pi(b)

        if abs(d) < self.linear_tolerance:
            desired.xVel = 0
            desired.thetaVel = self.kB * theta
        else:
            desired.xVel = self.kP * d * direction
            desired.thetaVel = self.kA * a + self.kB * b

        # Adjust velocities if X velocity is too high.
        if abs(desired.xVel) > self.max_linear_speed:
            ratio = self.max_linear_speed / abs(desired.xVel)
            desired.xVel *= ratio
            desired.thetaVel *= ratio

        # Adjust velocities if turning velocity too high.
        if abs(desired.thetaVel) > self.max_angular_speed:
            ratio = self.max_angular_speed / abs(desired.thetaVel)
            desired.xVel *= ratio
            desired.thetaVel *= ratio

        # TBD: Adjust velocities if linear or angular acceleration
        # too high.

        # Adjust velocities if too low, so robot does not stall.
        if abs(desired.xVel) > 0 and abs(desired.xVel) < self.min_linear_speed:
            ratio = self.min_linear_speed / abs(desired.xVel)
            desired.xVel *= ratio
            desired.thetaVel *= ratio
        elif desired.xVel == 0 and abs(desired.thetaVel) < self.min_angular_speed:
            ratio = self.min_angular_speed / abs(desired.thetaVel)
            desired.xVel *= ratio
            desired.thetaVel *= ratio

        return desired

    def normalize_half_pi(self, alpha):
        alpha = self.normalize_pi(alpha)
        if alpha > pi / 2:
            return alpha - pi
        elif alpha < -pi / 2:
            return alpha + pi
        else:
            return alpha

    def normalize_pi(self, alpha):
        while alpha >= pi:
            alpha -= 2 * pi
        while alpha < -pi:
            alpha += 2 * pi
        return alpha

    def sign(self, x):
        if x >= 0:
            return 1
        else:
            return -1
