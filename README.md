# Final assignment of ExpRob

### Introduction
This project is based on the navigation of a wheeled robot (with an RGB camera and a laser) in an enviroment with walls and colored balls that correspond to a precise location (e.g. living room, kitchen ecc..). The robot is able to switch in different states, according to what it sees, and according to the user commands.

<img src="https://github.com/chiaraterrile/exp_assignment3/blob/main/Images/enviroment.png" alt=" " width="600" height="400"/>

### Robot's behaviours

The robot has four main states (behaviour) :
* **SLEEP** : in this state the robot goes to a fixed position (in this case is the origin (0,0,0)) where is the home. Once reached the home, the robot stays there for a certain amount of time and then switches to NORMAL behaviour.
* **NORMAL** : in this state the robot goes in random positions and every time it detects a new object it switches to the substate TRACK. If nothing has been found, once reached the location, the robot can switch to NORMAL again, or to SLEEP.
* **PLAY** : the robot goes in this state whenever it receives a command *play* by the user. When in this state, the robot goes to the user position (which is fixed) and waits for a GoTo command. When a GoTo command is received, if the location is known, the robot reaches that location and then comes back to the user, waiting for another command. If the locations is unknown, the robot switches to the state FIND.
* **FIND** : in this states the robot moves in  the enviroment, exploring it, and looking for new objects. when it detects a new one, switches in the substate TRACK and if the object corresponds to the desired location returns in the PLAY state, otherwise it keeps looking. If after some time the location is not found, it returns anyway to the PLAY state where it will wait for a new command.

In the substate **TRACK**, the robot recogizes a new object and its color, then it reaches the object and stores information about its position.

**Note.** In all this states, while the robot is moving, it keeps avoiding obstacles using gmapping algorithm.

In advance is known the correspondence between a location and the color of the related ball, which is the following :

- blue -> entrance
- red  -> closet
- green -> living room
- magenta -> bathroom
- black -> bedroom
- yellow -> kitchen

While is unknown the position, that will be stored during the substate TRACK whenever a new ball is detected.

### Software architecture

The main blocks of the software architecture are the following.
<img src="https://github.com/chiaraterrile/exp_assignment3/blob/main/Images/Architecture.png" alt=" " width="600" height="400"/>

All the behaviours are controlled by the State Machine. When passing to a new state, this node communicate to the Object Detection if the state allows the detection (Normal or Find) or not (Play or Sleep). So if in Play or Sleep the Object Detection algortihm is basically in stand-by because we don't want to track anything.
This is done through a pub/sub communication to the topic _/state_fsm_ with a message of type Bool() that is true when the Object Detection is allowed.

The State Machine, according to the state in which is, send a Goal to the Move Base Action server through the topic _/move_base_, in order to reach that position (that can be home position, random position or user position). In instead the robot is in the Find state, the Explore node is launched. This node works with the _explore_lite_ package and allows the robot to explore the unkwnown enviroment. 

When in state Play, the Move Base Action Server is interrupted by publishing to the topic _/move_base/cancel_ which basically makes the server finish its task (the server believes to have reached the goal).

Of course, since there are walls in the enviroment, the robot while moving should avoid them, and to obtain this, is used the block Slam Gmapping, which considering the  informations obtained by the laser (topic _/scan_), sends the transformations frames (_/tf_) to both Move Base and to the Explore blocks.

The Object Detection node, instead communicates to the State Machine whenever a new ball (a not already detected one) is found in the enviroment while moving in Normal or in Find behaviour. This is done with a pub/sub communication to the topic _/new_ball_detected_ and the message sent is of type Bool() that is true when a ball is detected.

This node also communicates to the State Machine the informations about the new detected object, which are about the ball's position and about its color. This is done again with a pub/sub communication to the topic _/ball_info_ and message is of type ball(). This type of message has three fields, two for the x and y position and one for the color.

When the user wants to send a _play_ command, is launched the node Play, that communicates to the State Machine that needs to switch to the Play state. This is done thruogh the topic /play and a message of type Bool() is sent.

Instad, when the user wants to send a _GoTo_ command, is launched the node GoTo that communicates to the State Machine a GoTo + location command. This is done through the topic _/play_command_, and the message is of type command(). This message has two fields : a go field and a field for the location. This is because the user can send a GoTo command in the field go, but with a wrong location (syntax error or a room not present in our enviroment).

For a more complete architecture, write in the shell the following command :
```
$ rqt_graph
```
to see the nodes and active topics in that precise moment of the simulation.
### Packages and file list

The only package for this project is _exp_assignment3_.

In the package there are the followinf folders :
- **config** where there is che configuration file for RViz
- **explore** which is the folder for launching the explore_lite package
- **launch** where there are all the launch files for the simulation
- **msg** where there are the two messages _ball.msg_ for the pub/sub communication between Object Detection and State Machine nodes and _command.msg_ for the pub/sub communication between GoTo and State Machine nodes
- **param** where there are all the files containing the parameters for move_base, local planning and the costmap 
- **scripts** where there are the scripts of all the nodes used :
    - _state_machine.py_ which is the node of the State Machine
    - _object_detection.py_ which is the node for the Object Detection
    - _play.py_ which is the node for Play
    - _GoTo.py_ which is the node for GoTo
 - **urdf** where there are all the .urdf, .gazebo and .xacro that describe the robot with its sensors and the human
 - **worlds** where is the file_ house2.world_ that describes the simulation enviroment

### Installation and running procedure
To run the simulation it's necessary to put the package in a ROS workspace and then in the terminal run:
```
$ catkin_make
```
Then to launch the simulation:
```
$ roslaunch exp_assignment3 simulation.launch
```
this command lauches the Gazebo simulation, RViz, the Object Detection node, Gmapping + MoveBase for the navigation with obstacle avoidance, and the State Machine node.

An alternative command to launch the simulation, is to type :

```
$ ./sim.sh
```
in order to kill all the processes before launching the file and avoide problems with Gazebo. (Be sure to be in the directory _exp_assignment3_ before launching this command in the terminal)

Whenever the user want to make the robot go in the Play state:
```
$ roslaunch exp_assignment3 play.py
```
And to send a _GoTo + location_ command :
```
$ roslaunch exp_assignment3 GoTo.py
```
After launching this file, it will be asked to the user to insert a location that will be the one that the robot will have to reach or find.

### System's limitations
One limitation of the system is related to the fact that the Object Detection node is always active (but in stand-by when in Play or Sleep state), so during the Normal state the topic /new_ball_detected is continuously subscribed and if the flag is true it switches in the substate Track. The problem is that in this case, the Move Base Action Server in not shut down, it is still active, but it is like if the object detection had an higher priority, so the robot first reaches the ball and then come back to reach the goal. This is not a problem, except for the case in which the robot, while tracking the ball, follows a trajectory where there is the goal for the Action Server. In this case, the robot keeps tracking tha ball but switches at the same time to the next state of the Normal state. 
So it would be a problem for the system if the next state would be Sleep or Play, beacuse in that case the camera is not active, so it isn't detecting anything and the Tracking is interrupted.

I have "solved" this problem by putting a flag that indicates wheter or not the robot is in Tracking mode, so that, in the particular case where it reaches the goal while tracking, it doesn't switches to Sleep state, but remains in Normal, allowing the robot to conclude the Tracking and to store the ball's position.

### Possible technical improvements
A possible improvement is to solve the problem mentioned before, providing a way to pause the Action Server in some way, without using the cancel message, to avoid that the robot reaches the goal while tracking. 

Another possible improvement is related to the choice of the paramters for the move base and the local planner. I have changed some of them in order to make the robot faster and to make the response of the Action Server faster too, but I think that there are maybe other paramters that I could have modified in order to make the navigation more fluid and efficient, without risking the robot to remain stacked somewhere, as in some rare cases happens (depending on the direction that is following and on the wall positions).


### Author and contact
Terrile Chiara
mail: **chiaraterrile97@gmail.com**
