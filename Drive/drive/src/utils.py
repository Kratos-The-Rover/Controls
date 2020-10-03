from math import sqrt,pi

def get_goal_distance(current_pose,goal):
    if goal is None:
        return 0
    diffX = current_pose.x - goal.x
    diffY = current_pose.y - goal.y
    return sqrt(diffX**2 + diffY**2)


def at_goal(current_pose,goal,linear_tolerance,angular_tolerance):
    if goal is None:
        return True
    d = get_goal_distance(current_pose,goal) 
    dTh = abs(normalize_pi(current_pose.theta - goal.theta))
    return d < linear_tolerance and dTh < angular_tolerance


def normalize_half_pi(alpha):
    alpha = normalize_pi(alpha)
    if alpha > pi / 2:
        return alpha - pi
    elif alpha < -pi / 2:
        return alpha + pi
    else:
        return alpha

            
def normalize_pi(alpha):
    while alpha >= pi:
        alpha -= 2 * pi
    while alpha < -pi:
        alpha += 2 * pi
    return alpha


def sign(x):
    if x >= 0:
        return 1
    else:
        return -1


class Pose:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

    def __str__(self):
        return str({'x': self.x, 'y': self.y, 'theta': self.theta})
