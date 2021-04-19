#!/usr/bin/env python
"""!
@section Description
This scripts is a ROS node that implements a FSM that according to what detects the camera switch to one state or another

"""




# Imports
import roslib
import roslaunch
import os
import subprocess
import signal
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

class Room:
    """! This is class for storing informations about the rooms in the enviroment. """
    def __init__(self, **entries): self.__dict__.update(entries)
    
# Global flags

## flag that indicates if is possible to go in the substate TRACK (set to true the only in state NORMAL or FIND)
GoDetection = False 
## flag that indicates if is possible to launch the explore_lite node (used to avoid the node to be continuosly launched when we are in the FIND state)
LaunchExploration = True
## flag that indicates if the robot is in the FIND state (used in the case of the substate TRACK to differentiate the behaviour from the normal state)
FindState = False
## flag that indicates if a new object has been detected if in NORMAL or FIND state
det = False    
## flag that indicates if a command 'play' has been given by the user
flag_play = False 
## flag that indicates that in the FIND state the desired location has been found
FoundLocation = False

# Global variables
## variable that contains the desired location given by the GoTo command
desired_location =' '
## variable to run and stop the node explore_lite
child = None 
## variable that contains informations about the ball position (found during the object detection) and its color      
ball_info = ball()
## variable that contains the command given by the user ( GoTo command + desired location)
command_play = command()
## variable that contains informations (color, location, coordinates ...) about the object that the robot is tracking during the substate TRACK
room_track = Room()
## variables of the six rooms with their informations
room1 = Room(location = "entrance",color="blue",known = False)
room2 = Room(location = "closet",color="red",known = False)
room3 = Room(location = "living room",color="green",known = False)
room4 = Room(location = "kitchen",color="yellow",known = False)
room5 = Room(location = "bathroom",color="magenta",known = False)
room6 = Room(location = "bedroom",color="black",known = False)
## variable for the timer in the FIND state
t_final = 0






def coordinates_generator():
        """! This function is used to generate the random coordinates for the NORMAL behaviour. (coordinates are chosen among several points that are in the map) """
        desired_position_normal = Point()
        # points known a priori to be in the map
        points = [(-1, 6), (-4, 2),(-3,-3),(4,-5),(4,0),(4,2),(3,-4),(-3,8),(-4,-5),(-2,0),(-3,1),(-4,-3),(4,-7),(-2,-3),(-4,7),(-5,6),(-1,6),(-4,4),(0,4)]
        couple= random.choice(points)
        desired_position_normal.x = couple[0]
        desired_position_normal.y = couple[1]
        return(desired_position_normal)

class Coordinates :
    """! This class contains the fixed positions for the SLEEP behaviour and of the user  """
    sleep_xy = Point()
    sleep_xy.x = 0
    sleep_xy.y = 0
    sleep_yaw = 1
    user_xy = Point()
    user_xy.x = -5
    user_xy.y = 8
    user_yaw = -1.57
        
def Move(position,yaw):
        """!  This funcion implement the MoveBaseAction server having as input the desired location (x and y) and the orientation"""
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = position.x
        goal.target_pose.pose.position.y = position.y
        goal.target_pose.pose.orientation.w = yaw

        client.send_goal(goal)
        wait = client.wait_for_result()


VERBOSE = False


def clbk_play(msg):
    """! This callback is used to check whether or not a command play is received. If flag_play is set to true, the move_base action is cancelled"""
    global flag_play
    flag_play = msg
    if flag_play :
        pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        canc = GoalID ()
        pub.publish(canc)

      
def clbk_go(msg):
    """! This callback is used to save the message sent by the user (the GoTo command)"""
    global command_play
    command_play = msg
    



def clbk_ball_info(msg):
        """! This callback is used to save the informations received by the object_detection node """
        global LaunchExploration,child,det,ball,room_track,room1,room2,room3,room4,room5,room6,ball_info,FindState,FoundLocation
        
        ball_info.x = msg.x
        ball_info.y = msg.y
        ball_info.color = msg.color
        room = ' '
        color_found = ball_info.color
        # for each room check if the color is the one of the new object
        # if it is, assign to the corresponding room the xy coordinates, and set the flag known to true
        if color_found == room1.color:
                rospy.loginfo ('Found %s',room1.location)
                room1 = Room(location = "entrance",color="blue",known = True,  x = ball_info.x, y = ball_info.y)
                print(room1.x , room1.y)
                ball_info = ball()
                det = False 
                room_track = room1


        elif color_found == room2.color:
                rospy.loginfo ('Found %s',room2.location)
                room2 = Room(location = "closet",color="red",known = True, x = ball_info.x, y = ball_info.y)
                print(room2.x , room2.y)
                ball_info = ball()
                det =  False
                room_track = room2
                

        elif color_found ==room3.color:
                rospy.loginfo ('Found %s',room3.location)
                room3 = Room(location = "living room",color="green",known = True ,x = ball_info.x, y = ball_info.y)
                print(room3.x , room3.y)
                ball_info = ball()
                det = False 
                room_track = room3

        elif color_found == room4.color:
                rospy.loginfo ('Found %s',room4.location)
                room4 = Room(location = "kitchen",color="yellow",known = True, x = ball_info.x, y = ball_info.y)
                print(room4.x , room4.y)
                ball_info = ball()
                det = False 
                room_track = room4

        elif color_found == room5.color:
                rospy.loginfo ('Found %s',room5.location)
                room5 = Room(location = "bathroom",color="magenta",known = True, x = ball_info.x, y = ball_info.y)
                print(room5.x , room5.y)
                ball_info = ball()
                det = False 
                room_track = room5

        

        elif color_found == room6.color:
                rospy.loginfo ('Found %s',room6.location)
                room6 = Room(location = "bedroom",color="black",known = True, x = ball_info.x, y = ball_info.y)
                print(room6.x , room6.y)
                ball_info = ball()
                det = False 
                room_track = room6

        # if we are in the FIND state, check if the new room is the desired location
        # assigns a value to the flag FoundLocation and if it is false, put to true the flag for launching the exploration
        if FindState :
                if desired_location == room_track.location :
                        rospy.loginfo('I have found the desired location!')
                        FoundLocation = True

                elif desired_location != room_track.location:
                        rospy.loginfo('The room found is not the desired one!')
                        FoundLocation = False
                        LaunchExploration = True
        # if not in FIND, communicate to the user that is coming back to normal
        else :
                        rospy.loginfo ('Back to NORMAL state')
                        print('I am moving to random position : ', desired_position_normal)

def clbk_track(msg):
        """! This callback is used to check whether or not a new object has been detected by the object_detection. If this is true ( det = True) the robot goes in the substate TRACK and subscirbe to the topic /ball_info to get the coordinates and the color of the object """
        global FindState,child,det,t_final
        det = msg
        
        # if a new object has been detected switches to substate TRACK
        if det :
            # if in FIND state, shut down the explore_lite before switching to TRACK, and reinitialize the timer, in order to let the robot reaching the new ball
            if FindState:
                    t_final = time.time() + 60
                    child.send_signal(signal.SIGINT)     
            rospy.loginfo('############ Substate TRACK ##############')
            
            # at the same time subscribes to /ball_info topic to get informations about the ball
            sub_info = rospy.Subscriber('/ball_info', ball, clbk_ball_info)
        
                
       


def user_action():
	"""! this function controls the next state of the FSM (in the NORMAL state )according to the action of the user. If the flag_play is True the robot goes in the PLAY state."""
        global flag_play
        if flag_play :
                return ('play')
        else :
                return random.choice(['normal','sleep'])
        
# FSM functions 

# define state Normal
class Normal(smach.State):
    """! Define the Normal state (normal) """
    def __init__(self):
        
	
        smach.State.__init__(self, 
                             outcomes=['sleep','normal','play'],
                             input_keys=['normal_counter_in'],
                             output_keys=['normal_counter_out'])

        ## subscriber to the topic /new_ball_detected, used to check wheter or not a ball has been detected by the object_detection node
        self.sub = rospy.Subscriber('/new_ball_detected', Bool, clbk_track)
        ## subscriber to the topic /play, used to check wheter or not a command play has been received by the user
        self.sub_play = rospy.Subscriber('/play', Bool, clbk_play)
        ## publisher to the topic /state_fsm, used to indicate if we are in a state that allows the substate TRACK (possible in NORMAL or FIND) or not
        self.pub_state = rospy.Publisher('/state_fsm', Bool, queue_size=10)
        
        
    def execute(self, userdata):
    
        """! Normal state execution 
        @section Description
        In this state the robot goes in a random position in the map. This position is passed as input to the function Move() which through an action client makes the robot move in that direction avoiding the obstacles present in the enviroment.
        In the mean time, subscribing to the topic /new_ball_detected, every time the flag det is set to True, the robot switches to the substate TRACK.
        @return user_action
        """
        global desired_position_normal,det ,ball_info,flag_play,room1,room2,room3,room4,room5,room6,GoDetection

        desired_position_normal = coordinates_generator()
        
        desired_orientation_normal = 1
         
        print('I am moving to random position : ', desired_position_normal)
        time.sleep(2)
        GoDetection = True
        self.pub_state.publish(GoDetection)
        


        Move(desired_position_normal,desired_orientation_normal)

        #print('I am arrived! ')
        print('I am arrived')
        return user_action()
       
        
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

        ## publisher to the topic /state_fsm, used to indicate if we are in a state that allows the substate TRACK (possible in NORMAL or FIND) or not
        self.pub_state = rospy.Publisher('/state_fsm', Bool, queue_size=10)
        
        
    def execute(self, userdata):
        
        """! Sleeping state execution 
        @section Description
        In this state the robot goes in the sleeping position (fixed). This position is passed as input to the function Move() which through an action client makes the robot move in that direction avoiding the obstacles present in the enviroment.
        @return normal
        """
    
        global GoDetection
        desired_position_sleep = Coordinates.sleep_xy
        desired_orientation_sleep = Coordinates.sleep_yaw

        print('I am moving to home : ', desired_position_sleep)
        time.sleep(2)
        GoDetection = False
        self.pub_state.publish(GoDetection)

        Move(desired_position_sleep,desired_orientation_sleep)

        print('I am arrived home ')

        time.sleep(5)
        return ('normal')
   	
	
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
        ## publisher to the topic /state_fsm, used to indicate if we are in a state that allows the substate TRACK (possible in NORMAL or FIND) or not
        self.pub_state = rospy.Publisher('/state_fsm', Bool, queue_size=10)
        
        
    def execute(self, userdata):
        """! Playing state execution 
        @section Description
        In this state the robot goes to the user position and waits for a GoTo command by the user. It waits the command for a certain amount of time, if it is not received exits from the PLAY state. 
        When a GoTo command is received, is checked if the location in the command is known. If it is known, the robot simply goes there, and then come back to the play state. ( so back to the user waiting for another GoTo command). 
        If the location is unknown, the robot switches to the FIND behaviour. 

        @return the normal state in case of absence of GoTo commands, play state when the desired location has been found, the find state when the location is unknown
        """
        global LaunchExploration,desired_location,flag_play,command_play,room1,room2,room3,room4,room5,room6,GoDetection
        
        person_position = Coordinates.user_xy
        person_orientation = Coordinates.user_yaw

        print('I am moving to the user : ', person_position)
        flag_play = False
        time.sleep(2)
        GoDetection = False
        self.pub_state.publish(GoDetection)

        Move(person_position,person_orientation)

        command_play = command()

        print('I am here, I am waiting for the command!!')

        # subscriber to the topic play_command to store the GoTo command received by the user
        sub_go = rospy.Subscriber('/play_command', command, clbk_go)
        
        play_coordinates = Point()
        room = ' '
        ## flag that indictates if the robot has to go in the NORMAL state, it happens in case of no command received
        GoNormal = False
        # until a GoTo command is not received
        if command_play.go != 'GoTo' :
                # start the timer
                t_end = time.time() + 20 
                while time.time() < t_end :
                        # when the timer elapses, goes in NORMAL
                        GoNormal  = True
                        # keeps subscribing to check if a command is arrived
                        sub_go = rospy.Subscriber('/play_command', command, clbk_go)
                        # if it is exit
                        if command_play.go == 'GoTo' :
                                rospy.loginfo('I have received a command!')
                                print(command_play)
                                GoNormal  = False
                                break
                        
                if GoNormal :
                        rospy.loginfo('no GoTo command received!')
                        return ('normal')
        # assigns the location in the command to the varibale desired_location
        desired_location = command_play.location
        
        #for all the six rooms, check if the location is the same of desired one
        # if it is, check if the coordinates are known or not
        # if they are known, this is the room in which the robot will go
        # if not, return find

        if desired_location == room1.location:
                print(room1.known)
                if room1.known != False:
                        print ('I got the location coordinates!!')
                        room = room1.location
                        play_coordinates.x = room1.x
                        play_coordinates.y = room1.y
                else :
                        command_play = command()
                        print('The location is unknown')
                        LaunchExploration = True
                        return('find')
               


        elif desired_location == room2.location:

                if room2.known != False:
                        print ('I got the location coordinates!!')
                        room = room2.location
                        play_coordinates.x = room2.x
                        play_coordinates.y = room2.y
                else :
                        command_play = command()
                        print('The location is unknown')
                        LaunchExploration = True
                        return('find')
                
                

        elif desired_location == room3.location:
                
                if room3.known != False:
                        print ('I got the location coordinates!!')
                        room = room3.location
                        play_coordinates.x = room3.x
                        play_coordinates.y = room3.y
                else :
                        command_play = command()
                        print('The location is unknown')
                        LaunchExploration = True
                        return('find')
                 


        elif desired_location == room4.location:

                if room4.known != False :
                        print ('I got the location coordinates!!')
                        room = room4.location
                        play_coordinates.x = room4.x
                        play_coordinates.y = room4.y
                else :
                        command_play = command()
                        print('The location is unknown')
                        LaunchExploration = True
                        return('find')

        elif desired_location == room5.location:

                if room5.known != False :
                        print ('I got the location coordinates!!')
                        room = room5.location
                        play_coordinates.x = room5.x
                        play_coordinates.y = room5.y
                else :
                        command_play = command()
                        print('The location is unknown')
                        LaunchExploration = True
                        return('find')
                

        elif desired_location == room6.location:

                if room6.known != False :
                        print ('I got the location coordinates!!')
                        room = room6.location
                        play_coordinates.x = room6.x
                        play_coordinates.y = room6.y
                else :
                        command_play = command()
                        print('The location is unknown')
                        LaunchExploration = True
                        return('find')
        # if the location in the command is invalid ( syntax error or a location that is not in the enviroment) return play
        elif desired_location != room1.location and  desired_location != room2.location and  desired_location != room3.location and  desired_location != room4.location and  desired_location != room5.location and  desired_location != room6.location :
                        rospy.loginfo('Invalid location!! Try again!')
                        return('play')

        # if everything is ok, the robot moves to the desired location
        print('I am moving to  : ', room, play_coordinates)

        Move(play_coordinates,1)

        command_play = command()
        
        return('play') 

        
        rospy.loginfo('Executing state PLAYING (users = %f)'%userdata.playing_counter_in)
        userdata.playing_counter_out = userdata.playing_counter_in + 1
	
        
class Find(smach.State):
    """! Define the Find state  """
    def __init__(self):
	

        smach.State.__init__(self, 
			                 outcomes=['normal','find','play'],
                             input_keys=['find_counter_in'],
                             output_keys=['find_counter_out'])

        ## publisher to the topic /state_fsm, used to indicate if we are in a state that allows the substate TRACK (possible in NORMAL or FIND) or not
        self.pub_state = rospy.Publisher('/state_fsm', Bool, queue_size=10)
        ## subscriber to the topic /new_ball_detected, used to check wheter or not a ball has been detected by the object_detection node
        self.sub = rospy.Subscriber('/new_ball_detected', Bool, clbk_track)
        
    def execute(self, userdata):
        
        """! Find state execution 
        @section Description
        In this state is launched the node explore_lite package in order to explore the enviroment and find the unkwnown location. The robot stays in this state until the location is found or the timer has expired
        @return find if the timer has not expired, play if the desired location has been found
        """
        global t_final,child,GoDetection,desired_location,FindState,det,FoundLocation,room_track,LaunchExploration,process
        rospy.loginfo('I am looking for the location!')
        time.sleep(2)

        # flags to allow the detection and to communicate that the robot is in the FIND state
        GoDetection = True
        FindState = True

        self.pub_state.publish(GoDetection)
        
        # this file is launched just the first time the robot goes in the FINS state, in order to avoid that it is continuosly launched unnecessarly
        if LaunchExploration :
                LaunchExploration = False
                child = subprocess.Popen(["roslaunch","explore_lite","explore.launch"])
                # timer to exit from the FIND state, in case of non-finding of the location
                t_final = time.time() + 60 

        # if the timer elapses, shut down the launch file and return play
        if time.time() > t_final :
                print('Cannot found the ball, try again!!')
                child.send_signal(signal.SIGINT)
                FindState = False 
                det = False
                return ('play')

        # if the location is found, exit from the find state and return play
        if FoundLocation :
                FindState = False 
                return ('play')

        # if none of these conditions are found, keeps on returning find
        return('find')
   	
	
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
                               transitions={'normal':'NORMAL','play':'PLAYING','sleep':'SLEEPING'},
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
                               transitions={'normal':'NORMAL','find':'FIND','play':'PLAYING'},

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

