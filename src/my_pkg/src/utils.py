#!/usr/bin/env python
import math
import numpy as np

def normalize_angle(angle):
    """Normalize angle to be within [-pi, pi]."""
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < -math.pi:
        angle += 2*math.pi
    return angle


def is_facing_goal(initial, goal, theta):
    goal = np.array(goal)
    initial = np.array(initial)
    diff = goal - initial
    mag_diff = np.linalg.norm(diff)

    if mag_diff == 0:
        return True

    diff = diff / mag_diff
    
    h_x = math.cos(theta)
    h_y = math.sin(theta)
    h = np.array([h_x, h_y])
    
    return np.dot(h,diff) > 0