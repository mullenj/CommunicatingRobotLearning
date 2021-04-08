All of the data is saved here for the user studies. Each user has their data saved in a unique folder corrosponding with their user number. 
In each user folder the data is saved individually for each task. The files are named "data_method_x.pkl" where x is the method.
The methods are as follows:
* A = Monitor feedback
* B = Haptic only feedback
* C = Hololens only feedback
* D = Combined Haptic and Hololens feedback

For task 3, the experiment is run three times, one for each possible prior. This data is saved as "data_method_x_prior_y.pkl" where 
x is the same method value described above and y is the prior value.
The priors are as follows:
* A = Correct Prior
* B = Incorrect Shelf
* C = Incorrect Depth

For task 1 the data is saved in the following format at a timestep of 0.1 seconds.

[Time elapsed, state (joint space), state (xyz space), set of goals, a_h (human input action), a_star (robot action before belief is applied), a_r (robot action with belief applied), belief, z_triggered (boolean on whether z haptic signal has been sent), rot_triggered (boolean on whether rotation haptic signal has been sent)]

For task 2 the data is saved in the following format at a timestep of 0.1 seconds. The trajectories can be recalculated using the pickle files in the previous folder and following the code in teleop_task2.py file.

[Time elapsed, state (joint space), state (xyz space), set of goals, a_h (human input action), a_star (robot action before belief is applied), a_r (robot action with belief applied), belief, y_triggered (boolean on whether y haptic signal has been sent)]

For task 3 the data is saved in the following format at a timestep of 0.1 seconds.

[Time elapsed, state (joint space), state (xyz space), set of goals, a_h (human input action), a_star (robot action before belief is applied), a_r (robot action with belief applied), belief, z_triggered (boolean on whether z haptic signal has been sent), y_triggered (boolean on whether y haptic signal has been sent)]
