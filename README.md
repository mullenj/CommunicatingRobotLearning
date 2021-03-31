This README file is for the Multi-modal Communication of Robot Learning senior design project at Virginia Tech.

The current learning model in teleop.py is naive and does not learn over time, but chooses between a couple of targets as you move the robot towards them. As the user, your goal is to get to your intended target quickly and accurately.

To connect to the haptic device, the command is as follows:
'''
sudo killall rfcomm
sudo rfcomm release 0
sudo rfcomm connect /dev/rfcomm0 7C:9E:BD:D8:C2:02 1 &
'''
To Debug try redownloading the Arduino Code


teleop_task1.py is set up to run the first task of the user study. It begins with a cup in the robots grasper mechanism and the users goal is to reach a shelf without "spilling" the liquid in the cup. The robot will begin maneuvering towards the midpoint of the four goals, in the middle vertically, and rotated 45 degrees. Once a "critical state" is reached for either of these states the haptic device will be triggered telling the user which direction the robot is confused. Additionally, the hololens will display the confidence in each goal the robot has and provide the user with additional information.

teleop_task2.py is set up to run the second task of the user study. This task will involve the user maneuvering an object towards a goal with an obstacle in the way of the robots motion. The robot will have a critical state when deciding which trajectory to pick and ask the user for help. The hololens displays the top two trajectories during this time and after the user helps will show them which trajectory the robot is taking.

teleop_task3.py is set up to run the third task of the user study. This task will expand on the previous two by 'learning' from the demonstrations in the first, it now knows to keep the cup upright but does now know which shelf or the depth on each shelf to place the cup. The robot is initialized with a prior that may or may not be correct and the user would have to correct it if need be.

**Functionality needs to be added to all tasks to save off data prior to conducting the user study**
