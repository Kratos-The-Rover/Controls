#!/usr/bin/env python
import rospy
from math import pi, asin, acos
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
import actionlib
from diff_drive import pid_controller


def path_callback(msg):
    global goal
    goal=PoseStamped()
    goal.pose.position.x=msg.x
    goal.pose.position.y=msg.y
    goal.pose.orientation = quaternion_from_euler(0,0,0) #change third coordinate in tuple to change theta
    goto()


def newOdom(msg):
    global x, y, theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w,
    ]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    theta = yaw

def goto():
    while not rospy.is_shutdown() and goal is not None:

        if controller.at_goal(x, y, theta, goal):
            rospy.loginfo("Goal achieved")
            goal = None
            desired.xVel=0
            desired.thetaVel=0
        else:
            desired = controller.get_velocity(x, y, theta, goal, dT)
            d = controller.get_goal_distance(x, y, theta, goal)
            rospy.loginfo(d)
            dist_pub.publish(d)

        twist.linear.x = desired.xVel
        twist.angular.z = desired.thetaVel
        pub.publish(twist)
        r.sleep()

#initial values for various variables
dT = 0.1
x = 0
y = 0
theta = 0
twist = Twist()
r = rospy.Rate(10)
controller = pid_controller.PIDController()
rospy.init_node("my_controller", anonymous=True)

sub_goal=rospy.Subscriber("/path",Point ,path_callback)
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
dist_pub = rospy.Publisher("~distance_to_goal", Float32, queue_size=10)

