#!/usr/bin/env python
"""!
@section Description
This scripts is a ROS node that implements a FSM that according to what detects the camera switch to one state or another

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
from actionlib import GoalID
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion

# Global variables

## Coordinates of the home position in the sleep state and of the orientation
desired_position_sleep_ = Point()
desired_position_sleep_.x = -5
desired_position_sleep_.y = 8

desired_orientation_sleep_ = Quaternion()
desired_orientation_sleep_.w = 1

class Room:
    "A structure that can have any fields defined."
    def __init__(self, **entries): self.__dict__.update(entries)

entrance = Room(color="blue",known = False)
closet = Room(color="red",known = False)
living_room = Room(color="green",known = False)
kitchen = Room(color="yellow",known = False)
bathroom = Room(color="magenta",known = False)
bedroom = Room(color="black",known = False)


VERBOSE = False

# flag to indicate that a new ball has been detected

ball_info = ball()

det = False    
flag_play = False 
# FMS functions
def clbk_play(msg):
    global flag_play
    flag_play = msg
    if flag_play != False :
        print('lllllll')
        #rospy.ROSInterruptException()
        pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        canc = GoalID ()
        pub.publish(canc)

        #return ('play')
    

def clbk_ball_info(msg):
        global ball_info
        global det,ball,coord,entrance,bedroom,bathroom,living_room,kitchen,closet,ball_info

        ball_info.x = msg.x
        ball_info.y = msg.y
        ball_info.color = msg.color
        #print(ball_info)
        color_found = ball_info.color
        #print(color_found)

        if color_found == entrance.color:
                print ('Found entrance!')
                entrance = Room(color="blue",known = True,  x = ball_info.x, y = ball_info.y)
                ball_info = ball()
                det = False 

        elif color_found == bedroom.color:
                print ('Found bedroom!')
                bedroom = Room(color="black",known = True, x = ball_info.x, y = ball_info.y)
                ball_info = ball()
                det = False 

        elif color_found == closet.color:
                print ('Found closet!')
                closet = Room(color="red",known = True, x = ball_info.x, y = ball_info.y)
                ball_info = ball()
                det =  False
                

        elif color_found ==living_room.color:
                print ('Found living room!')
                living_room = Room(color="green",known = True ,x = ball_info.x, y = ball_info.y)
                ball_info = ball()
                det = False 


        elif color_found == bathroom.color:
                print ('Found bathroom!')
                bathroom = Room(color="magenta",known = True, x = ball_info.x, y = ball_info.y)
                ball_info = ball()
                det = False 

        elif color_found == kitchen.color:
                print ('Found kitchen!')
                kitchen = Room(color="yellow",known = True, x = ball_info.x, y = ball_info.y)
                ball_info = ball()
                det = False 



def clbk_track(msg):
        global det,ball,coord,entrance,bedroom,bathroom,living_room,kitchen,closet,ball_info
        det = msg

        if det != False :
            print ('############ Substate TRACK ##############')
   
            sub_info = rospy.Subscriber('/ball_info', ball, clbk_ball_info)
       


def user_action():
	"""! this function controls the next state of the FSM  """
        global flag_play
        if flag_play != False :
                return ('play')
        else :
                return ('normal')
        #print('forse ci siamol')	
        #return ('sleep')
  # return random.choice(['normal','sleep'])


# define state RandomlyGoing
class RandomlyGoing(smach.State):
    """! Define the RandomlyGoing state (normal) """
    def __init__(self):
        
	
        smach.State.__init__(self, 
                             #outcomes=['sleep','normal','play'],
                             outcomes=['normal','play'],
                             input_keys=['randomlygoing_counter_in'],
                             output_keys=['randomlygoing_counter_out'])


        self.sub = rospy.Subscriber('/new_ball_detected', Bool, clbk_track)
        self.sub_play = rospy.Subscriber('/play', Bool, clbk_play)
        
    def execute(self, userdata):
    
        """! Normal state execution 
        @section Description
        In this state is generated every time a new Point desired_position_normal_ in a radom way
        This goal position is sent trough an action client to the server that makes the robot move toward the goal position
        @return the user_action
        """
        global det ,ball_info,flag_play
        print(flag_play)
       

        desired_position_normal_ = Point()
        desired_orientation_normal_ = Quaternion()
        
        A= Point ()
        A.x = -1
        A.y = 8
        B = Point()  
        B.x = -4
        B.y = 2
        C = Point()  
        C.x = -3
        C.y = -3
        D = Point()  
        D.x = 4
        D.y = 0
        E = Point()  
        E.x = 4
        E.y = -4
        F = Point()  
        F.x = 4
        F.y = -7
        points = [A,B,C,D,E,F]
        desired_position_normal_= random.choice(points)
        
        desired_orientation_normal_.w = 1

        print('I am moving to random position : ', desired_position_normal_)

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = desired_position_normal_.x
        goal.target_pose.pose.position.y = desired_position_normal_.y
        goal.target_pose.pose.orientation.w = desired_orientation_normal_.w

        client.send_goal(goal)
        wait = client.wait_for_result()

        
        

        print('I am arrived! ')

        return user_action()
                
        # when a new ball is detected the robot switches in the substate Track where ut goes near the ball and stores informations about the ball position
        
        
        
        rospy.loginfo('Executing state RANDOMLYGOING (users = %f)'%userdata.randomlygoing_counter_in)
        userdata.randomlygoing_counter_out = userdata.randomlygoing_counter_in + 1
	
        
    

# define state Sleeping
class Sleeping(smach.State):
    """! Define the Sleeping state  """
    def __init__(self):
	

        smach.State.__init__(self, 
			                 outcomes=['normal'],
                             input_keys=['sleeping_counter_in'],
                             output_keys=['sleeping_counter_out'])
        
    def execute(self, userdata):
        
        """! Sleeping state execution 
        @section Description
        In this state the home position is sent trough an action client to the server that makes the robot move toward the goal position
        @return the user_action
        """
    
        global desired_position_sleep_ ,desired_orientation_sleep_
        
        print('I am moving to home : ', desired_position_sleep_)

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = desired_position_sleep_.x
        goal.target_pose.pose.position.y = desired_position_sleep_.y
        goal.target_pose.pose.orientation.w = desired_orientation_sleep_.w

        client.send_goal(goal)
        wait = client.wait_for_result()
        
        print('I am arrived home ')
        #time.sleep(5)
        return ('normal')
   	
	
        rospy.loginfo('Executing state SLEEPING (users = %f)'%userdata.sleeping_counter_in)
        userdata.sleeping_counter_out = userdata.sleeping_counter_in + 1
	
        
        

# define state Playing
class Playing(smach.State):
    """! Define the Playing state  """
    def __init__(self):
        smach.State.__init__(self, 
			                 outcomes=['sleep'],
                             input_keys=['playing_counter_in'],
                             output_keys=['playing_counter_out'])
       
    def execute(self, userdata):
        """! Playing state execution 
        @section Description
        In this state the robot tracks the ball until it is present, when it cannot detect the ball it returns to the normal state
        @return the normal state in case of absence of the ball
        """
       
        return ('sleep')    

        
        rospy.loginfo('Executing state PLAYING (users = %f)'%userdata.playing_counter_in)
        userdata.playing_counter_out = userdata.playing_counter_in + 1
	
        


        
def main():
  
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('RANDOMLYGOING', RandomlyGoing(), 
                               transitions={'normal':'RANDOMLYGOING','play':'PLAYING'},
                               remapping={'randomlygoing_counter_in':'sm_counter', 
                                          'randomlygoing_counter_out':'sm_counter'})

        smach.StateMachine.add('PLAYING', Playing(), 
                               transitions={'sleep':'SLEEPING'
					    },
                                            
							
                               remapping={'playing_counter_in':'sm_counter',
                                          'plying_counter_out':'sm_counter'})

       
        
        
       
        smach.StateMachine.add('SLEEPING', Sleeping(), 
                               transitions={'normal':'RANDOMLYGOING'},

                               remapping={'sleeping_counter_in':'sm_counter',
                                          'sleeping_counter_out':'sm_counter'})

        

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    
    
    

if __name__ == '__main__':
	#state_machine()
        main()

