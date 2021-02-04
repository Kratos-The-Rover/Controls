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
    global goal, start,controller,x_prev,y_prev
    goal.x = msg.x
    goal.y = msg.y
    goal.theta = None  # default set to None, wont alter goal theta after reaching
    if x_prev!=goal.x or y_prev!=goal.y:
        # Initialize controller
        controller = PIDController()
        start.x=x_prev
        start.y=y_prev
        x_prev=goal.x
        y_prev=goal.y
        # Call goto
        goto()


# Callback to be called when msg is published to the topic odom or odometry/filtered
def odom_callback(msg):
    # Define current_pose variable
    global current_pose
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
    global goal,start, current_pose, controller,desired_vel
    # Calculate desired_vel by calling get_velocity and at_goal
    # Goal is none when we are at final point
    chk=True
    while chk:
        # Check if goal is reached
        if at_goal(current_pose, goal, linear_tolerance, angular_tolerance):
            rospy.loginfo("Goal achieved")
            chk=False
            desired_vel.x = 0
            desired_vel.theta = 0
        # Else calculate velocities to reach nearer to goal
        else:
            desired_vel = controller.get_velocity(current_pose, start,goal)
            d = get_goal_distance(current_pose, goal)

        # Publish velocity to cmd_vel as twist messages
        twist.linear.x = desired_vel.x
        twist.angular.z = desired_vel.theta
        pub.publish(twist)
        r.sleep()


rospy.init_node("controller")
linear_tolerance = rospy.get_param("linear_tolerance", 0.1)  # 2.5cm
angular_tolerance = rospy.get_param("angular_tolerance", 10 / 180 * pi)  # 3 degrees

# initial values for various variables
global x_prev,y_prev,current_pose,desired_vel,start,goal,twist
twist = Twist()
current_pose = Pose()
desired_vel = Pose()
start=Pose()
goal = Pose()
x_prev=0
y_prev=0
r = rospy.Rate(10)

# Subscriber/Publisher initializations
sub = rospy.Subscriber(
    "/odom", Odometry, odom_callback
)  # odom for kratos/tb3 ,for husky odometry/filtered
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
rospy.spin()
