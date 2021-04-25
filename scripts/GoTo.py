#!/usr/bin/env python
"""!
@section Description
This scripts is a ROS node that implements publisher to send a GoTo + location command to the state_machine node 

"""
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


def GoTo():

    """! This function implements a publisher to the topic /play_command to send a GoTo command to the state_machine """
    pub_command= rospy.Publisher('play_command', command, queue_size=10)
    rospy.init_node('GoTo', anonymous=True) 

    ## variable of type command, that contains the GoTo + location command
    msg = command()
 
    msg.go = 'GoTo'
    
    loc = raw_input('Where do you wanna go? ')
    rospy.loginfo('You have typed : %s', loc)

    msg.location = loc
    pub_command.publish(msg)
   
if __name__ == '__main__':
    try:
        GoTo()
    except rospy.ROSInterruptException:
        pass