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
from std_msgs.msg import String,Float64
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

def play_command():
    pub = rospy.Publisher('play', Bool, queue_size=10)
    rospy.init_node('play_command', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
   # while not rospy.is_shutdown():
    play_go = True
    pub.publish(play_go)
        #rate.sleep()
   

if __name__ == '__main__':
    try:
        play_command()
    except rospy.ROSInterruptException:
        pass