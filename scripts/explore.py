#!/usr/bin/env python

import roslaunch
import os

def explore():
   os.system("roslaunch explore_lite explore.launch") 

if __name__ == '__main__':
    try:
        explore()
    except rospy.ROSInterruptException:
        pass

