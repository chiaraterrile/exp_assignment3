#!/usr/bin/env python

# Python libs
import sys
import time
from nav_msgs.msg import Odometry
from exp_assignment3.msg import ball
from geometry_msgs.msg import Point, Twist
# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String,Float64
from std_msgs.msg import Bool

VERBOSE = False



# Global flags
## flag that indicates if the robot is in a state that allows the object detection (NORMAL or FIND) or not
GoDetection = False
## flags that indicate if an object of that color has been already detected or not
red_detected = False
blue_detected = False
green_detected = False
black_detected = False
magenta_detected = False
yellow_detected = False

## variable that contains the position of the ball according to the robot's odometry
ball_pos = Point()
## variable in which are stored the informations about the color of the ball and its position
ball_info= ball()

def clbk_ball_pos(msg):
    """! This callback assigns the informations about the position of the robot (when is near to the ball) to ball_pos variable """    
    global ball_pos
    ball_pos.x = msg.pose.pose.position.x
    ball_pos.y = msg.pose.pose.position.y
    ball_pos.z = msg.pose.pose.position.z
    

def clbk_state(msg):
    """! This callback stores the information about the state of the robot assigning a boolean value to the flag GoDetection """
    global GoDetection
    GoDetection = msg.data
    
         
         


class image_feature:

    def __init__(self):
        global GoDetection
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('object_detection', anonymous=True)
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/cmd_vel",
                                       Twist, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

        self.sub = rospy.Subscriber('/odom', Odometry, clbk_ball_pos)
        ## this subscriber calls the clbk_state in order to check if is possible the detection or not
        self.sub_state = rospy.Subscriber('/state_fsm', Bool, clbk_state)

    def callback(self, ros_data):
        global blue_detected,green_detected,red_detected,magenta_detected,black_detected,yellow_detected,ball_pos,ball_info,GoDetection
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        ## this publisher is used to send to the FSM the informations about a ball
        pub_ball = rospy.Publisher('/ball_info', ball, queue_size=10)
        ## this publisher is used to communicate to the FSM that a new object has been found
        pub_detection = rospy.Publisher('/new_ball_detected', Bool , queue_size=10)
        
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
        
        # the object detetction proceed only if we are in NORMAL or FIND state
        if GoDetection  :

            ## upper and lower for each color (the six colors of the balls in the rooms)
            greenLower = (50, 50, 50)
            greenUpper = (70, 255, 255)

            blueLower  = (100, 50, 50)
            blueUpper = (130, 255, 255)

            redLower = (0, 50, 50)
            redUpper = (5, 255, 255)

            blackLower = (0, 0, 0) 
            blackUpper = (5,50,50)

            magentaLower = (125, 50, 50) 
            magentaUpper = (150, 255, 255)

            
            yellowLower = (25, 50, 50) 
            yellowUpper = (35, 255, 255)


            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            
            ## definition of all the masks for all the colors
            mask_blue = cv2.inRange(hsv, blueLower, blueUpper)
            mask_blue= cv2.erode(mask_blue, None, iterations=2)
            mask_blue = cv2.dilate(mask_blue, None, iterations=2)

            mask_red = cv2.inRange(hsv, redLower, redUpper)
            mask_red = cv2.erode(mask_red, None, iterations=2)
            mask_red = cv2.dilate(mask_red, None, iterations=2)

            mask_green = cv2.inRange(hsv, greenLower, greenUpper)
            mask_green = cv2.erode(mask_green, None, iterations=2)
            mask_green = cv2.dilate(mask_green, None, iterations=2)

            mask_black = cv2.inRange(hsv, blackLower, blackUpper)
            mask_black = cv2.erode(mask_black, None, iterations=2)
            mask_black = cv2.dilate(mask_black, None, iterations=2)

            mask_magenta = cv2.inRange(hsv, magentaLower, magentaUpper)
            mask_magenta = cv2.erode(mask_magenta, None, iterations=2)
            mask_magenta = cv2.dilate(mask_magenta, None, iterations=2)

            mask_yellow = cv2.inRange(hsv, yellowLower, yellowUpper)
            mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
            mask_yellow = cv2.dilate(mask_yellow, None, iterations=2)
            
       
            ##########   GREEN detection  #######################
            cnts_green = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts_green = imutils.grab_contours(cnts_green)
            center = None
            # only proceed if at least one contour was found
            if len(cnts_green) > 0:
                det = False
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts_green, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                
                # only proceed if the color has not been already detected
                if green_detected != True  :
        
                    det = True
                    pub_detection.publish(det)
                    print('I am getting closer to the green object')
                    # only proceed if the radius meets a minimum size
                    if radius > 8:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(image_np, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                        vel = Twist()
                        vel.angular.z = -0.006*(center[0]-400)
                        vel.linear.x = -0.01*(radius-100) 
                        self.vel_pub.publish(vel)
                        # if the robot is near the ball, stores the informations about its color and its position
                        if vel.linear.x < 0.1 :
                            ball_info.x = ball_pos.x
                            ball_info.y = ball_pos.y
                            ball_info.color = 'green' 

                            pub_ball.publish(ball_info)
                            print('published!')
                            time.sleep(5)
                            
                            green_detected = True
                            det = False

            ##########   BLUE detection  #######################
            cnts_blue = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts_blue = imutils.grab_contours(cnts_blue)
            center = None
            # only proceed if at least one contour was found
            if len(cnts_blue) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                det = False
                c = max(cnts_blue, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                
                # only proceed if the color has not been already detected
                if blue_detected != True  :
                    det = True
                    pub_detection.publish(det)
                    print('I am getting closer to the blue object')
                    # only proceed if the radius meets a minimum size 
                    if radius > 8:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(image_np, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                        vel = Twist()
                        vel.angular.z = -0.006*(center[0]-400)
                        vel.linear.x = -0.01*(radius-100) 
                        self.vel_pub.publish(vel)

                        # if the robot is near the ball, stores the informations about its color and its position
                        if vel.linear.x < 0.1 :
                        
                            ball_info.x = ball_pos.x
                            ball_info.y = ball_pos.y
                            ball_info.color = 'blue' 

                            pub_ball.publish(ball_info)
                            print('published!')
                            time.sleep(5)

                            blue_detected = True
                            det = False
                        




            ##########   RED detection  #######################
            cnts_red = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts_red = imutils.grab_contours(cnts_red)
            center = None
            # only proceed if at least one contour was found
            if len(cnts_red) > 0:
                det = False
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts_red, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    
                # only proceed if the color has not been already detected
                if red_detected != True :
                    print('I am getting closer to the red object')
                    det = True
                    pub_detection.publish(det)
                    # only proceed if the radius meets a minimum size
                    if radius > 8:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(image_np, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                        vel = Twist()
                        vel.angular.z = -0.006*(center[0]-400)
                        vel.linear.x = -0.01*(radius-100) 
                        self.vel_pub.publish(vel)

                        # if the robot is near the ball, stores the informations about its color and its position
                        if vel.linear.x < 0.1 :
                            ball_info.x = ball_pos.x
                            ball_info.y = ball_pos.y
                            ball_info.color = 'red' 

                            pub_ball.publish(ball_info)
                            print('published!')
                            time.sleep(5)

                            red_detected = True
                            det = False


            ##########   BLACK detection  #######################
            cnts_black = cv2.findContours(mask_black.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts_black = imutils.grab_contours(cnts_black)
            center = None
            # only proceed if at least one contour was found
        
            if len(cnts_black) > 0:
                det = False
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts_black, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the color has not been already detected
                if black_detected != True :
                    print('I am getting closer to the black object')
                    det = True
                    pub_detection.publish(det)
                    # only proceed if the radius meets a minimum size
                    if radius > 8:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(image_np, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                        vel = Twist()
                        vel.angular.z = -0.002*(center[0]-400)
                        vel.linear.x = -0.01*(radius-100) 
                        self.vel_pub.publish(vel)

                        # if the robot is near the ball, stores the informations about its color and its position
                        if vel.linear.x < 0.1 :
                            ball_info.x = ball_pos.x
                            ball_info.y = ball_pos.y
                            ball_info.color = 'black' 
                            #print(ball_info)
                            pub_ball.publish(ball_info)
                            print('published!')
                            time.sleep(2)

                            black_detected = True
                            det = False

            
            ##########   MAGENTA detection  #######################
            cnts_magenta = cv2.findContours(mask_magenta.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts_magenta = imutils.grab_contours(cnts_magenta)
            center = None
            # only proceed if at least one contour was found
            if len(cnts_magenta) > 0:
                det = False
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts_magenta, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the color has not been already detected
                if magenta_detected != True  :

                    det = True
                    pub_detection.publish(det)
                    print('I am getting closer to the magenta object')
                    # only proceed if the radius meets a minimum size
                    if radius > 8:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(image_np, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                        vel = Twist()
                        vel.angular.z = -0.006*(center[0]-400)
                        vel.linear.x = -0.01*(radius-100) 
                        self.vel_pub.publish(vel)

                        # if the robot is near the ball, stores the informations about its color and its position
                        if vel.linear.x < 0.1 :
                            ball_info.x = ball_pos.x
                            ball_info.y = ball_pos.y
                            ball_info.color = 'magenta' 

                            pub_ball.publish(ball_info)
                            print('published!')
                            time.sleep(5)

                            magenta_detected = True
                            det = False
            

            ##########   YELLOW detection  #######################
            cnts_yellow = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts_yellow = imutils.grab_contours(cnts_yellow)
            center = None
            # only proceed if at least one contour was found
            if len(cnts_yellow) > 0:
                det = False
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts_yellow, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the color has not been already detected
                if yellow_detected != True :

                    det = True
                    pub_detection.publish(det)
                    print('I am getting closer to the yellow object')
                    # only proceed if the radius meets a minimum size
                    if radius > 8:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(image_np, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                        vel = Twist()
                        vel.angular.z = -0.006*(center[0]-400)
                        vel.linear.x = -0.01*(radius-100) 
                        self.vel_pub.publish(vel)
        
                        # if the robot is near the ball, stores the informations about its color and its position
                        if vel.linear.x < 0.1 :
                            ball_info.x = ball_pos.x
                            ball_info.y = ball_pos.y
                            ball_info.color = 'yellow' 

                            pub_ball.publish(ball_info)
                            print('published!')
                            time.sleep(5)

                            yellow_detected = True
                            det = False


            # update the points queue
            # pts.appendleft(center)
            cv2.imshow('window', image_np)
            cv2.waitKey(2)
            

        # self.subscriber.unregister()
        

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
