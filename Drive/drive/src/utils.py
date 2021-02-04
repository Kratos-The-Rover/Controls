# Python file that contains all the utility functions used in the package

from math import sqrt, pi

# Function to return distance of current point from the goal
def get_goal_distance(current_pose, goal):
    diffX = current_pose.x - goal.x
    diffY = current_pose.y - goal.y
    return sqrt(diffX ** 2 + diffY ** 2)


# Function to return True if goal is reached, False otherwise
def at_goal(current_pose, goal, linear_tolerance, angular_tolerance):
    d = get_goal_distance(current_pose, goal)
    if goal.theta is None:
        dTh=0
    else:
        dTh = abs(normalize_pi(current_pose.theta - goal.theta))
    return d < linear_tolerance and dTh < angular_tolerance


# Function to return normalized value (between -pi to pi) of an angle alpha
def normalize_half_pi(alpha):
    alpha = normalize_pi(alpha)
    if alpha > pi / 2:
        return alpha - pi
    elif alpha < -pi / 2:
        return alpha + pi
    else:
        return alpha


# Function to return normalized value (between -pi/2 to pi/2) of an angle alpha
def normalize_pi(alpha):
    while alpha >= pi:
        alpha -= 2 * pi
    while alpha < -pi:
        alpha += 2 * pi
    return alpha


# Function to return sign of the passed variable
def sign(x):
    if x >= 0:
        return 1
    else:
        return -1


# Custom classs definition for Pose used in the package
class Pose:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = None

    def __str__(self):
        return str({"x": self.x, "y": self.y, "theta": self.theta})
