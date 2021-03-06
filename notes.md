This README file is for the Multi-modal Communication of Robot Learning senior design project at Virginia Tech.

The current learning model in teleop.py is naive and does not learn over time, but chooses between a couple of targets as you move the robot towards them. As the user, your goal is to get to your intended target quickly and accurately.

The functionality that needs to be added includes:
* ~~Robot assistance and take over (Teleop.py)~~
* ~~Compile a text file with the goal locations and their associated belief (Teleop.py)~~
* ~~Set up web server for Hololens to Access (Teleop.py and Server.py)~~
* ~~Set up goal locations~~
* ~~Get color gradient working~~
* Find places to call Haptic Functions from haptic_util.py (Teleop.py)

To connect to the haptic device, the command is as follows:
'''
sudo rfcomm connect /dev/rfcomm0 7C:9E:BD:D8:C2:02 1 &
'''
To Debug try the following:
1. sudo killall rfcomm
2. sudo rfcomm release 0
3. Redownload Arduino Code

To Restart after hangup: not working
'''
sudo killall rfcomm
sudo rfcomm release 0
sudo rfcomm bind 0 7C:9E:BD:D8:C2:02
'''

teleop_task1.py is set up to run the first task of the user study. This task will be the simplest of the three. It begins with a cup in the robots grasper mechanism and operates similarly to the above example. As the person gets closer to a goal location the robot will begin assisting. We want the robot to begin tilting the cup to feign ignorance, from which the human needs to correct the rotation to make the robot select the correct goal.

teleop_task1.py functionality:
* Simplified calculations using list comprehensions
* Four goals, two high and two low, two sideways, and two upright
* Favoring tilted end goal (might want better implementation)
* Parameter tuning (might want to lower beta? only requires one really good correction to favor straight and the assistance helps a lot)
* Gripper
* Haptics

teleop_task2.py is set up to run the second task of the user study. This task will involve the user maneuvering an object towards a goal with an obstacle in the way of the robots motion. The robot now needs to learn that it has to avoid this obstacle.

teleop_task2.py functionality:
* Discrete trajectories in Cartesian space
* Assistance (Might need to be messed with to make what we want)
* Gripper
* Haptics

teleop_task3.py is set up to run the third task of the user study. This task will expand on the previous two by 'learning' from the demonstrations in the first. The robot begins running autonomously and the user has to provide corrections while they are distracted. The haptic device will play a key role in alerting the user to what the robot is about to do and its confusion.

teleop_task3.py functionality:
* Two goals, one high and one low (no tilting at the moment)
* Initialize to favor one of the two, pseudo-randomly
* Implemented Critical States (need to be tuned)
* Haptics

**Functionality needs to be added to all tasks to save off data prior to conducting the user study**

Bugs
* Sometimes objects get double created in the hololens ruining everything (maybe fixed)
* Buttons get double triggered (maybe fixed)
* Send white threshold (done)
* Graphics Move after setting Hololens down
