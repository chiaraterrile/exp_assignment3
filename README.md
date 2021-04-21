# exp_assignment3

### Introduction
This project is based on the navigation of a wheeled robot in an enviroment with walls (obstacles) and colored balls that correspond to a precise location (e.g. living room, kitchen ecc..). The robot is able to switch in different states, according to what it sees, and according to the user commands.

<img src="https://github.com/chiaraterrile/exp_assignment3/blob/main/Images/enviroment.png" alt=" " width="600" height="400"/>

### Robot's behaviours

The robot has four main states (behaviour) :
* **SLEEP** : in this state the robot goes to a fixed position (in this case is the origin (0,0,0)) where is the home. Once reached the home, the robot stays there for a certain amount of time and then switches to NORMAL behaviour.
* **NORMAL** : in this state the robot goes in random positions and every time it detects a new object it switches to the substate TRACK. If nothing has been found, once reached the location, the robot can switch to NORMAL again, or to SLEEP.
* **PLAY** : the robot goes in this state whenever it receives a command *play* by the user. When in this state, the robot goes to the user position (which is fixed) and waits for a GoTo command. When a GoTo command is received, if the location is known, the robot reaches that location and then comes back to the user, waiting for another command. If the locations is unknown, the robot switches to the state FIND.
* **FIND** : in this states the robot moves in  the enviroment, exploring it, and looking for new objects. when it detects a new one, switches in the substate TRACK and if the object correspond to the desired location returns in the PLAY state, otherwise it keeps looking. If after some time the location is not found, it returns anyway to the PLAY state where it will wait for a new command.

In the substate **TRACK**, the robot recogizes an object and its color, then it reaches the object and stores information about its position.

**Note.** In all this states, while the robot is moving, it keeps avoiding obstacles using gmapping algorthm.



### Software architecture

<img src="https://github.com/chiaraterrile/exp_assignment3/blob/main/Images/Architecture.png" alt=" " width="600" height="400"/>


### Packages and file list



### Installation and running procedure
To run the simulation it's necessary to put the package in a ROS workspace and then in the terminal run:
```
$ catkin_make
```
Then to launch the simulation:
```
$ roslaunch exp_assignment2 gazebo_world.launch
```
In another terminal to run the state machine :
```
$ rosrun exp_assignment2 state_machine.py
```
Whenever the user want to make the robot interact with the ball in another terminal has to run :
```
$ rostopic pub /reaching_goal/goal....
```
in order to send the message to the action server of the ball and position it in the desired position. To make the ball disappear the user has simply to set the z-coordinate of the position to a negative value.


### System's limitations

### Possible technical improvements

### Author and contact
Terrile Chiara
mail: **chiaraterrile97@gmail.com**
