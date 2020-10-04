#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from pid_controller import PIDController
from utils import *
from math import pi

# Callback to be called when msg is published to the topic path
def path_callback(msg):
    global goal, controller
    goal = Pose()
    goal.x = msg.x
    goal.y = msg.y
    goal.theta = 0  # default set to zero ##check later
    # Initialize controller
    controller = PIDController()
    # Call goto
    goto()


# Callback to be called when msg is published to the topic odom or odometry/filtered
def odom_callback(msg):
    # Define current_pose variable
    global current_pose
    current_pose = Pose()
    current_pose.x = msg.pose.pose.position.x
    current_pose.y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w,
    ]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_pose.theta = yaw
    # Subscribe to path
    sub_goal = rospy.Subscriber("/path", Point, path_callback)


# Function to navigate from current position to goal
def goto():
    global goal, current_pose, controller
    # Calculate desired_vel by calling get_velocity and at_goal
    desired_vel = Pose()
    # Goal is none when we are at final point
    while goal is not None:
        # Check if goal is reached
        if at_goal(current_pose, goal, linear_tolerance, angular_tolerance):
            rospy.loginfo("Goal achieved")
            goal = None
            desired_vel.x = 0
            desired_vel.theta = 0
        # Else calculate velocities to reach nearer to goal
        else:
            desired_vel = controller.get_velocity(current_pose, goal)
            d = get_goal_distance(current_pose, goal)
            rospy.loginfo(d)
            dist_pub.publish(d)

        # Publish velocity to cmd_vel as twist messages
        twist.linear.x = desired_vel.x
        twist.angular.z = desired_vel.theta
        pub.publish(twist)
        r.sleep()


# initial values for various variables
rospy.init_node("controller")
linear_tolerance = rospy.get_param("~linear_tolerance", 0.1)  # 2.5cm
angular_tolerance = rospy.get_param("~angular_tolerance", 10 / 180 * pi)  # 3 degrees
twist = Twist()
r = rospy.Rate(10)

# Subscriber/Publisher initializations
sub = rospy.Subscriber(
    "/odometry/filtered", Odometry, odom_callback
)  # odom for kratos/tb for husky odometry/filtered
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
dist_pub = rospy.Publisher("~distance_to_goal", Float32, queue_size=10)
rospy.spin()
