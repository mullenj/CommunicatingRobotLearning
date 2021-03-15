This README file is for the Multi-modal Communication of Robot Learning senior design project at Virginia Tech.

The current learning model in teleop.py is naive and does not learn over time, but chooses between a couple of targets as you move the robot towards them. As the user, your goal is to get to your intended target quickly and accurately.

The functionality that needs to be added includes:
* ~~Robot assistance and take over (Teleop.py)~~
* ~~Compile a text file with the goal locations and their associated belief (Teleop.py)~~
* ~~Set up web server for Hololens to Access (Teleop.py and Server.py)~~
* ~~Set up goal locations~~
* ~~Get color gradient working~~
* Find places to call Haptic Functions from haptic_util.py (Teleop.py)

To connect to the haptic device, the procedure is as follows:
1. x
2. y
3. z

teleop_task1.py is set up to run the first task of the user study. This task will be the simplest of the three. It begins with a cup in the robots grasper mechanism and operates similarly to the above example. As the person gets closer to a goal location the robot will begin assisting. We want the robot to begin tilting the cup to feign ignorance, from which the human needs to correct the rotation to make the robot select the correct goal.

teleop_task1.py functionality:
* Simplified calculations using list comprehensions
* Four goals, two high and two low, two sideways, and two upright
* Favoring tilted end goal (might want better implementation)
* Parameter tuning (might want to lower beta? only requires one really good correction to favor straight and the assistance helps a lot)

To be implemented
* Haptics

teleop_task2.py will be set up to run the second task of the user study. This task will involve the same scenario as the first but with an obstacle in the way of the robots motion. The robot now needs to learn that it has to avoid this obstacle.

teleop_task2.py functionality:

To be implemented
* Discrete trajectories
* Learning?
* Haptics

teleop_task3.py will be set up to run the third task of the user study. This task will expand on the previous two by learning from the demonstrations in the second. However, this time it is not holding a cup and may be 'confused'. The robot begins running autonomously and the user has to provide corrections while they are distracted. This will be fleshed out more after the prior two are completed.
