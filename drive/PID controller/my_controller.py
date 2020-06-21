#!/usr/bin/env python
import rospy
from math import pi, asin, acos
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, PoseStamped,Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
import actionlib
from diff_drive import goal_controller
from diff_drive import pose
dT=0.1
x=0
y=0
theta=0
print("enter goal")
a=float(input())
b=float(input())
controller=goal_controller.GoalController()
kP = rospy.get_param('~kP', 3.0)
kA = rospy.get_param('~kA', 8.0)
kB = rospy.get_param('~kB', -1.5)
controller.set_constants(kP, kA, kB)
controller.set_linear_tolerance(rospy.get_param('~linear_tolerance', 0.05))
controller.set_angular_tolerance(rospy.get_param('~angular_tolerance', 3/180*pi))
controller.set_max_linear_speed(rospy.get_param('~max_linear_speed', 0.2))
controller.set_min_linear_speed(rospy.get_param('~min_linear_speed', 0))
controller.set_max_angular_speed(rospy.get_param('~max_angular_speed', 1.0))
controller.set_min_angular_speed(rospy.get_param('~min_angular_speed', 0))
controller.set_max_linear_acceleration(rospy.get_param('~max_linear_acceleration', 0.1))
controller.set_max_angular_acceleration(rospy.get_param('~max_angular_acceleration', 0.3))

        # Set whether to allow movement backward. Backward movement is
        # safe if the robot can avoid obstacles while traveling in
        # reverse. We default to forward movement only since many
        # sensors are front-facing.
controller.set_forward_movement_only(rospy.get_param('~forwardMovementOnly', True))

rospy.init_node("my_controller",anonymous=True)


def init_pose():
    pose = pose.Pose()
    pose.x = 0
    pose.y = 0
    pose.theta = 0

def newOdom(msg):
    global x,y,theta
    
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    orientation_q=msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    theta=yaw
  
goal=pose.Pose()
goal.x=a
goal.y=b
goal.theta=0

sub=rospy.Subscriber('/odom',Odometry,newOdom)
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
dist_pub=rospy.Publisher('~distance_to_goal',Float32,queue_size=10)
r=rospy.Rate(10)
twist=Twist()

while not rospy.is_shutdown () and goal is not None:
    
    if controller.at_goal(x,y,theta,goal):
        twist.linear.x=0
        twist.angular.z=0
        pub.publish(twist)
    else:
        desired=controller.get_velocity(x,y,theta,goal,dT)
    d=controller.get_goal_distance(x,y,theta,goal)
    print(d)
    dist_pub.publish(d)

    twist.linear.x=desired.xVel
    twist.angular.z=desired.thetaVel
    pub.publish(twist)

    if controller.at_goal(x,y,theta,goal):
        rospy.loginfo('Goal achieved')
        goal=None
    r.sleep()

    