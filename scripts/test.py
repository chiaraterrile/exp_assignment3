#!/usr/bin/env python

# Imports
import roslib
from std_msgs.msg import Bool
from exp_assignment3.msg import ball
from exp_assignment3.msg import command
import time
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String,Float64,Bool
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from tf import transformations
import math
import numpy as np
from scipy.ndimage import filters
import imutils
import cv2
from sensor_msgs.msg import CompressedImage
import actionlib
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion


def coordinates_generator():
        desired_position_normal_ = Point()
        points = [(-1, 6), (-4, 2),(-3,-4),(4,0),(3,-4),(4,-7)]
        couple= random.choice(points)
        desired_position_normal_.x = couple[0]
        desired_position_normal_.y = couple[1]
        return(desired_position_normal_)

class Coordinates :
    normal_xy = coordinates_generator()
    normal_yaw = 1
    sleep_xy = Point()
    sleep_xy.x = 0
    sleep_xy.y = 0
    sleep_yaw = 1
    user_xy = Point()
    user_xy.x = -5
    user_xy.y = 8
    user_yaw = -1.57

def boh(user) :
    print (user)
        
def GoTo():
    normal = Coordinates.normal_xy
    sleep = Coordinates.sleep_xy
    user = Coordinates.user_xy
    boh(user)


   
if __name__ == '__main__':
    try:
        GoTo()
    except rospy.ROSInterruptException:
        pass
