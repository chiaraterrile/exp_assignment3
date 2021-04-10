#!/usr/bin/env python
"""!
@section Description
This scripts is a ROS node that implements a FSM that according to what detects the camera switch to one state or another

"""




# Imports
import roslib
import roslaunch
import os
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
desired_position_sleep_.x = 0
desired_position_sleep_.y = 0

desired_orientation_sleep_ = Quaternion()
desired_orientation_sleep_.w = 1

person_position = Point()
person_position.x = -5
person_position.y = 8
person_orientation = -1.57

GoDetection = False

class Room:
    "A structure that can have any fields defined."
    def __init__(self, **entries): self.__dict__.update(entries)

room1 = Room(location = "entrance",color="blue",known = False)
room2 = Room(location = "closet",color="red",known = False)
room3 = Room(location = "living room",color="green",known = False)
room4 = Room(location = "kitchen",color="yellow",known = False)
room5 = Room(location = "bathroom",color="magenta",known = False)
room6 = Room(location = "bedroom",color="black",known = False)


VERBOSE = False

# flag to indicate that a new ball has been detected

ball_info = ball()
command_play = command()

det = False    
flag_play = False 
# FMS functions
def clbk_play(msg):
    global flag_play
    flag_play = msg
    if flag_play != False :
        pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        canc = GoalID ()
        pub.publish(canc)

        #return ('play')

def clbk_go(msg):
    global command_play
    command_play = msg
    #print (command_play)

def clbk_ball_info(msg):
        global ball_info
        global det,ball,coord,room1,room2,room3,room4,room5,room6,ball_info

        ball_info.x = msg.x
        ball_info.y = msg.y
        ball_info.color = msg.color
        #print(ball_info)
        color_found = ball_info.color
        #print(color_found)

        if color_found == room1.color:
                print ('Found ',room1.location)
                room1 = Room(location = "entrance",color="blue",known = True,  x = ball_info.x, y = ball_info.y)
                ball_info = ball()
                det = False 


        elif color_found == room2.color:
                print ('Found ',room2.location)
                room2 = Room(location = "closet",color="red",known = True, x = ball_info.x, y = ball_info.y)
                ball_info = ball()
                det =  False
                

        elif color_found ==room3.color:
                print ('Found ',room3.location)
                room3 = Room(location = "living room",color="green",known = True ,x = ball_info.x, y = ball_info.y)
                ball_info = ball()
                det = False 


        elif color_found == room4.color:
                print ('Found ',room4.location)
                room4 = Room(location = "bathroom",color="magenta",known = True, x = ball_info.x, y = ball_info.y)
                ball_info = ball()
                det = False 

        elif color_found == room5.color:
                print ('Found ',room5.location)
                room5 = Room(location = "kitchen",color="yellow",known = True, x = ball_info.x, y = ball_info.y)
                ball_info = ball()
                det = False 

        elif color_found == room6.color:
                print ('Found ',room6.location)
                room6 = Room(location = "bedroom",color="black",known = True, x = ball_info.x, y = ball_info.y)
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
        

# define state Normal
class Normal(smach.State):
    """! Define the Normal state (normal) """
    def __init__(self):
        
	
        smach.State.__init__(self, 
                             #outcomes=['sleep','normal','play'],
                             outcomes=['normal','play'],
                             input_keys=['normal_counter_in'],
                             output_keys=['normal_counter_out'])


        self.sub = rospy.Subscriber('/new_ball_detected', Bool, clbk_track)
        self.sub_play = rospy.Subscriber('/play', Bool, clbk_play)
        self.pub_state = rospy.Publisher('/state_fsm', Bool, queue_size=10)
        
        
        #state = 'normal'
        #self.pub_state.publish(state)
        #print('xoxoxoxooxo')
    def execute(self, userdata):
    
        """! Normal state execution 
        @section Description
        In this state is generated every time a new Point desired_position_normal_ in a radom way
        This goal position is sent trough an action client to the server that makes the robot move toward the goal position
        @return the user_action
        """
        global det ,ball_info,flag_play,room1,room2,room3,room4,room5,room6,GoDetection
        
        
        #pub_state = rospy.Publisher('/state_fsm', String, queue_size=10)
        #state_normal = 'normal'
        

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
        #desired_position_normal_.x = A.x
        #desired_position_normal_.y = A.y
        
        desired_orientation_normal_.w = 1

        print('I am moving to random position : ', desired_position_normal_)
        time.sleep(2)
        GoDetection = True
        self.pub_state.publish(GoDetection)
        #self.pub_state.publish(state_normal)
        print('published!!!')

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
        #self.sub.unregister()
        return user_action()
                
        # when a new ball is detected the robot switches in the substate Track where ut goes near the ball and stores informations about the ball position
        
        
        
        rospy.loginfo('Executing state NORMAL (users = %f)'%userdata.normal_counter_in)
        userdata.normal_counter_out = userdata.normal_counter_in + 1
	
        
    

# define state Sleeping
class Sleeping(smach.State):
    """! Define the Sleeping state  """
    def __init__(self):
	

        smach.State.__init__(self, 
			                 outcomes=['normal'],
                             input_keys=['sleeping_counter_in'],
                             output_keys=['sleeping_counter_out'])
        
        self.pub_state = rospy.Publisher('/state_fsm', Bool, queue_size=10)
        
        
    def execute(self, userdata):
        
        """! Sleeping state execution 
        @section Description
        In this state the home position is sent trough an action client to the server that makes the robot move toward the goal position
        @return the user_action
        """
    
        global desired_position_sleep_ ,desired_orientation_sleep_,GoDetection
        
        print('I am moving to home : ', desired_position_sleep_)
        time.sleep(2)
        GoDetection = False
        self.pub_state.publish(GoDetection)

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
        return random.choice('normal','sleep')
   	
	
        rospy.loginfo('Executing state SLEEPING (users = %f)'%userdata.sleeping_counter_in)
        userdata.sleeping_counter_out = userdata.sleeping_counter_in + 1
	
        
        

# define state Playing
class Playing(smach.State):
    """! Define the Playing state  """
    def __init__(self):
        smach.State.__init__(self, 
			                 outcomes=['sleep','find','play','normal'],
                             input_keys=['playing_counter_in'],
                             output_keys=['playing_counter_out'])
        #self.sub_go = rospy.Subscriber('/play_command', command, clbk_go)  
        self.pub_state = rospy.Publisher('/state_fsm', Bool, queue_size=10)
        #self.pub_wait_command =  rospy.Publisher('/wait_command', Bool, queue_size=10)
        
    def execute(self, userdata):
        """! Playing state execution 
        @section Description
        In this state the robot tracks the ball until it is present, when it cannot detect the ball it returns to the normal state
        @return the normal state in case of absence of the ball
        """
        global flag_play,command_play,room1,room2,room3,room4,room5,room6,person_position,GoDetection,person_orientation
        
        print('I am moving to the user : ', person_position)
        flag_play = False
        time.sleep(2)
        GoDetection = False
        self.pub_state.publish(GoDetection)

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = person_position.x
        goal.target_pose.pose.position.y = person_position.y
        goal.target_pose.pose.orientation.w = person_orientation

        client.send_goal(goal)
        wait = client.wait_for_result()

        print('I am here, I am waiting for the command!!')

        sub_go = rospy.Subscriber('/play_command', command, clbk_go)
        #print("command received: ", command_play)
        
        play_coordinates = Point()
        room = ' '
        GoNormal = False
        if command_play.go != 'GoTo' :
                t_end = time.time() + 20 
                while time.time() < t_end :
                        
                        
                        GoNormal  = True
                        sub_go = rospy.Subscriber('/play_command', command, clbk_go)
                        if command_play.go == 'GoTo' :
                                print('I have received a command!')
                                print(command_play)
                                GoNormal  = False
                                break
                        
                if GoNormal :
                        print('no GoTo command received!')
                        return ('normal')

        desired_location = command_play.location
        if desired_location == room1.location:
                print(room1.known)
                if room1.known != False:
                        print ('I got the location coordinates!!')
                        room = room1.location
                        play_coordinates.x = room1.x
                        play_coordinates.y = room1.y
                else :
                        command_play = command()
                        print('devo andare in un altro stato')
                        return('find')
               


        elif desired_location == room2.location:

                if room2.known != False:
                        print ('I got the location coordinates!!')
                        room = room2.location
                        play_coordinates.x = room2.x
                        play_coordinates.y = room2.y
                else :
                        command_play = command()
                        print('devo andare in un altro stato')
                        return('find')
                
                

        elif desired_location ==room3.location:

                if room3.known != False:
                        print ('I got the location coordinates!!')
                        room = room3.location
                        play_coordinates.x = room3.x
                        play_coordinates.y = room3.y
                else :
                        command_play = command()
                        print('devo andare in un altro stato')
                        return('find')
                 


        elif desired_location == room4.location:

                if room4.known != False :
                        print ('I got the location coordinates!!')
                        room = room4.location
                        play_coordinates.x = room4.x
                        play_coordinates.y = room4.y
                else :
                        command_play = command()
                        print('devo andare in un altro stato')
                        return('find')

        elif desired_location == room5.location:

                if room5.known != False :
                        print ('I got the location coordinates!!')
                        room = room5.location
                        play_coordinates.x = room5.x
                        play_coordinates.y = room5.y
                else :
                        command_play = command()
                        print('devo andare in un altro stato')
                        return('find')
                

        elif desired_location == room6.location:

                if room6.known != False :
                        print ('I got the location coordinates!!')
                        room = room6.location
                        play_coordinates.x = room6.x
                        play_coordinates.y = room6.y
                else :
                        command_play = command()
                        print('devo andare in un altro stato')
                        return('find')

        print('I am moving to  : ', room, play_coordinates)

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = play_coordinates.x
        goal.target_pose.pose.position.y = play_coordinates.y
        goal.target_pose.pose.orientation.w = 1

        client.send_goal(goal)
        wait = client.wait_for_result()

        command_play = command()
        
        return('play') 


           

        
        rospy.loginfo('Executing state PLAYING (users = %f)'%userdata.playing_counter_in)
        userdata.playing_counter_out = userdata.playing_counter_in + 1
	
        
class Find(smach.State):
    """! Define the Find state  """
    def __init__(self):
	

        smach.State.__init__(self, 
			                 outcomes=['normal'],
                             input_keys=['find_counter_in'],
                             output_keys=['find_counter_out'])
        
        self.pub_state = rospy.Publisher('/state_fsm', Bool, queue_size=10)
        
    def execute(self, userdata):
        
        """! Sleeping state execution 
        @section Description
        In this state the home position is sent trough an action client to the server that makes the robot move toward the goal position
        @return the user_action
        """
        global GoDetection
        print('-----------FIND-------------------')
        time.sleep(2)
        GoDetection = True
        self.pub_state.publish(GoDetection)

        os.system("roslaunch explore_lite explore.launch")

        return('normal')
   	
	
        rospy.loginfo('Executing state FIND (users = %f)'%userdata.find_counter_in)
        userdata.find_counter_out = userdata.find_counter_in + 1
	

        
def main():
  
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'normal':'NORMAL','play':'PLAYING'},
                               remapping={'normal_counter_in':'sm_counter', 
                                          'normal_counter_out':'sm_counter'})

        smach.StateMachine.add('PLAYING', Playing(), 
                               transitions={'sleep':'SLEEPING','find':'FIND','play':'PLAYING','normal':'NORMAL'
					    },
                                            
							
                               remapping={'playing_counter_in':'sm_counter',
                                          'plying_counter_out':'sm_counter'})

       
        
        
       
        smach.StateMachine.add('SLEEPING', Sleeping(), 
                               transitions={'normal':'NORMAL'},

                               remapping={'sleeping_counter_in':'sm_counter',
                                          'sleeping_counter_out':'sm_counter'})
        
        smach.StateMachine.add('FIND', Find(), 
                               transitions={'normal':'NORMAL'},

                               remapping={'find_counter_in':'sm_counter',
                                          'find_counter_out':'sm_counter'})


        

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

