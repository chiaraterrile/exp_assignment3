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

# Global variables

         
## Coordinates of the home position in the sleep state and of the orientation
#desired_position_normal= Point()


GoDetection = False
LaunchExploration = True
desired_location =' '
child = None 

FindState = False
t_final = None 

class Room:
    "A structure that can have any fields defined."
    def __init__(self, **entries): self.__dict__.update(entries)

room1 = Room(location = "entrance",color="blue",known = False)
room2 = Room(location = "closet",color="red",known = False)
room3 = Room(location = "living room",color="green",known = False)
room4 = Room(location = "kitchen",color="yellow",known = False)
room5 = Room(location = "bathroom",color="magenta",known = False)
room6 = Room(location = "bedroom",color="black",known = False)


def coordinates_generator():
        desired_position_normal = Point()
        points = [(-1, 6), (-4, 2),(-3,-4),(4,0),(3,-4),(4,-7)]
        couple= random.choice(points)
        desired_position_normal.x = couple[0]
        desired_position_normal.y = couple[1]
        return(desired_position_normal)

class Coordinates :
    sleep_xy = Point()
    sleep_xy.x = 0
    sleep_xy.y = 0
    sleep_yaw = 1
    user_xy = Point()
    user_xy.x = -5
    user_xy.y = 8
    user_yaw = -1.57
        


VERBOSE = False

# flag to indicate that a new ball has been detected

ball_info = ball()
command_play = command()

det = False    
flag_play = False 
FoundLocation = False
room_track = Room()
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

def Move(position,yaw):

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


def clbk_ball_info(msg):
        
        global t_final,LaunchExploration,child,det,ball,room_track,coord,room1,room2,room3,room4,room5,room6,ball_info,FindState,FoundLocation
        
        ball_info.x = msg.x
        ball_info.y = msg.y
        ball_info.color = msg.color
        room = ' '
        #print(ball_info)
        color_found = ball_info.color
        #print(color_found)

        if color_found == room1.color:
                print ('Found ',room1.location)
                room1 = Room(location = "entrance",color="blue",known = True,  x = ball_info.x, y = ball_info.y)
                print(room1.x , room1.y)
                ball_info = ball()
                det = False 
                room_track = room1


        elif color_found == room2.color:
                print ('Found ',room2.location)
                room2 = Room(location = "closet",color="red",known = True, x = ball_info.x, y = ball_info.y)
                print(room2.x , room2.y)
                ball_info = ball()
                det =  False
                room_track = room2
                

        elif color_found ==room3.color:
                print ('Found ',room3.location)
                room3 = Room(location = "living room",color="green",known = True ,x = ball_info.x, y = ball_info.y)
                print(room3.x , room3.y)
                ball_info = ball()
                det = False 
                room_track = room3

        elif color_found == room4.color:
                print ('Found ',room4.location)
                room4 = Room(location = "kitchen",color="yellow",known = True, x = ball_info.x, y = ball_info.y)
                print(room4.x , room4.y)
                ball_info = ball()
                det = False 
                room_track = room4

        elif color_found == room5.color:
                print ('Found ',room5.location)
                room5 = Room(location = "bathroom",color="magenta",known = True, x = ball_info.x, y = ball_info.y)
                print(room5.x , room5.y)
                ball_info = ball()
                det = False 
                room_track = room5

        

        elif color_found == room6.color:
                print ('Found ',room6.location)
                room6 = Room(location = "bedroom",color="black",known = True, x = ball_info.x, y = ball_info.y)
                print(room6.x , room6.y)
                ball_info = ball()
                det = False 
                room_track = room6

        if FindState :
                if desired_location == room_track.location :
                        print('I have found the desired location!')
                        FoundLocation = True

                elif desired_location != room_track.location:
                        print('The room found is not the desired one!')
                        FoundLocation = False
                        LaunchExploration = True
        
        else :
                        print('I am moving to random position : ', desired_position_normal)



def clbk_track(msg):
        global desired_position_normal_,t_final,FindState,child,det,ball,coord,entrance,bedroom,bathroom,living_room,kitchen,closet,ball_info
        det = msg
       
        if det != False :
            if FindState :
                    child.send_signal(signal.SIGINT)
                    #child.terminate()
                    t_final = time.time() + 120
                    
            print ('############ Substate TRACK ##############')
            
            sub_info = rospy.Subscriber('/ball_info', ball, clbk_ball_info)
        
                
       


def user_action():
	"""! this function controls the next state of the FSM  """
        global flag_play
        if flag_play != False :
                return ('play')
        else :
                return random.choice(['normal','sleep'])
        

# define state Normal
class Normal(smach.State):
    """! Define the Normal state (normal) """
    def __init__(self):
        
	
        smach.State.__init__(self, 
                             outcomes=['sleep','normal','play'],
                             input_keys=['normal_counter_in'],
                             output_keys=['normal_counter_out'])


        self.sub = rospy.Subscriber('/new_ball_detected', Bool, clbk_track)
        self.sub_play = rospy.Subscriber('/play', Bool, clbk_play)
        self.pub_state = rospy.Publisher('/state_fsm', Bool, queue_size=10)
        
        
    def execute(self, userdata):
    
        """! Normal state execution 
        @section Description
        In this state is generated every time a new Point desired_position_normal_ in a radom way
        This goal position is sent trough an action client to the server that makes the robot move toward the goal position
        @return the user_action
        """
        global desired_position_normal,det ,ball_info,flag_play,room1,room2,room3,room4,room5,room6,GoDetection

        desired_position_normal = coordinates_generator()
        desired_orientation_normal = 1
        
        print('I am moving to random position : ', desired_position_normal)
        time.sleep(2)
        GoDetection = True
        self.pub_state.publish(GoDetection)

        Move(desired_position_normal,desired_orientation_normal)

        print('I am arrived! ')
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
        return random.choice('normal')
   	
	
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
                        print('devo andare in un altro stato')
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
                        print('devo andare in un altro stato')
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
                        print('devo andare in un altro stato')
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
                        print('devo andare in un altro stato')
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
                        print('devo andare in un altro stato')
                        LaunchExploration = True
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
			                 outcomes=['normal','find','play'],
                             input_keys=['find_counter_in'],
                             output_keys=['find_counter_out'])
        
        self.pub_state = rospy.Publisher('/state_fsm', Bool, queue_size=10)
        self.sub = rospy.Subscriber('/new_ball_detected', Bool, clbk_track)
        
    def execute(self, userdata):
        
        """! Sleeping state execution 
        @section Description
        In this state the home position is sent trough an action client to the server that makes the robot move toward the goal position
        @return the user_action
        """
        global t_final,child,GoDetection,desired_location,FindState,det,FoundLocation,room_track,LaunchExploration,process
        print('I am looking for the location!')
        time.sleep(2)
        GoDetection = True
        
        FindState = True
        self.pub_state.publish(GoDetection)
        
        if LaunchExploration :
                LaunchExploration = False
               # os.system("gnome-terminal -x roslaunch explore_lite explore.launch")
                #child = subprocess.Popen(["gnome-terminal","-x","roslaunch","explore_lite","explore.launch"])
                child = subprocess.Popen(["roslaunch","explore_lite","explore.launch"])
                #child = subprocess.run(["lxterminal","-e","roslaunch","explore_lite","explore.launch"])
                t_final = time.time() + 120 

        
        if time.time() > t_final :
                print('Cannot found the ball, try again!!')
                child.send_signal(signal.SIGINT)
                FindState = False 
                return ('play')

        #print(FoundLocation)
        if FoundLocation :
                FindState = False 
                return ('play')
        
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

