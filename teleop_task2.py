import numpy as np
import pickle
import sys
import time
import random
import tkinter as tk
from return_home import return_home
import teleop_utils as utils
import hapticcode.haptic_control as haptic
np.set_printoptions(suppress=True)

"""
 * a script for teleoperating the robot using a joystick in task 2 of the senior
 * design user study. It is set up to have one final location goal and four
 * possible trajectories to get to it. The robot will begin navigating
 * autonomously and the user may need to correct the trajectory if it paths
 * through an obstacle.
 * Dylan Losey, September 2020
 * James Mullen, March 2021

 * To run:
 [1] in one terminal:
    navigate to ~/panda-ws/essentials
    run python3 teleop_task2.py
 [2] in a second terminal:
    navigate to ~/libfranka/build
    run ./collab/velocity_control
    run ./collab/grasp_control
"""
sendfreq = 0.1 # (also sets data saving)
trajectory_files = ["task2_path0", "task2_path1", "task2_path2", "task2_path3"]
home = np.asarray([0.000297636, -0.785294, -0.000218009, -2.3567, 0.000397658, 1.57042, 0.785205])


'''
This function allows us to send all of the necessary information to the hololens.
This information changes based off of what state the program is in, initialized
or in progress.
'''
def send2hololens(belief, coord_curr, initialized, waypoints, latch_point):
    print(belief)
    if initialized:
        with open('robotUpdate.txt', 'w') as f:
            f.write(f"{coord_curr[0]}\t{coord_curr[1]}\t{coord_curr[2]}\n")
            for count, belief_x in enumerate(belief):
                if count < len(belief) - 1:
                    f.write(f"{belief_x}\n")
                else:
                    f.write(f"{belief_x}")
    else:
        points = [[utils.joint2pose(point) for point in waypoint] for waypoint in waypoints]
        goals = [point[4] for point in points]
        with open('robotInit.txt', 'w') as f:
            f.write(f"{latch_point}\t{coord_curr[0]}\t{coord_curr[1]}\t{coord_curr[2]}\n")
            for count, (goal, belief_x) in enumerate(zip(goals, belief)):
                f.write(f"{goal[0]}\t{goal[1]}\t{goal[2]}\t{belief_x}\n")
            for count_1, point in enumerate(points):
                for count_2, p in enumerate(point):
                    if count_2 == 0 or count_2 == 4:
                        continue
                    elif count_1 < len(points) - 1 or count_2 < len(point) - 2:
                        f.write(f"traj\t{count_1 + 1}\t{p[0]}\t{p[1]}\t{p[2]}\n")
                    else:
                        f.write(f"traj\t{count_1 + 1}\t{p[0]}\t{p[1]}\t{p[2]}")


def main():

    participant = sys.argv[1]
    method = sys.argv[2]
    haptics_on = method == "B" or method == "D"

    PORT_robot = 8080
    PORT_gripper = 8081
    action_scale = 0.04
    interface = utils.Joystick()
    print('[*] Connecting to haptic device...')
    hapticconn = haptic.initialize()
    y_triggered = False

    print('[*] Connecting to low-level controller...')

    conn = utils.connect2robot(PORT_robot) # connect to other computer
    conn_gripper = utils.connect2robot(PORT_gripper)

    # readState -> give you a dictionary with things like joint values, velocity, torque
    state = utils.readState(conn)
    # joint2pose -> forward kinematics: convert the joint position to the xyz position of the end-effector
    s_home = np.asarray(utils.joint2pose(state["q"]))
    # print(coord_home)

    # Prepare Trajectories by loading them into the trajectory class and getting an array of points along the trajectory to compare against
    playback_time = 20.0
    proportional_gain = 5.0
    waypoints = [pickle.load(open(traj + ".pkl", "rb")) for traj in trajectory_files]
    trajectories = [utils.Trajectory(waypoint, playback_time) for waypoint in waypoints]
    trajectories = [np.asarray([utils.joint2pose(traj.get(time_i)) for time_i in list(np.linspace(0, playback_time, int(playback_time * 100 + 1)))]) for traj in trajectories]
    belief = np.asarray([0.4, 0.2, 0.2, 0.2])
    BETA = 0.1
    start_mode = True
    gripper_closed = False
    gradient = 0.8

    print('[*] Ready for a teleoperation...')

    # Set up the initial robotInit.txt file
    send2hololens(belief, s_home, False, waypoints, gradient)

    # Set up time counters
    lastsend = time.time() - sendfreq
    lastsave = time.time() - sendfreq
    start_time = time.time()
    task_start_time = time.time()
    start_timer = time.time()
    motion_start = random.uniform(1, 5)

    # Data variable for saving to pickle file
    data = []

    window = tk.Tk()
    window.title("User Study GUI")
    text = tk.Label(text="The below table represents the belief as a percentage for each goal.")
    text.pack()
    b_text = ["Through the cup:  ", "Over the cup:        ", "Left of cup:           ", "Right of cup:        "]
    b_vars = [tk.StringVar(), tk.StringVar(), tk.StringVar(), tk.StringVar()]
    b_labels = [tk.Label(window, textvariable = b_var) for b_var in b_vars]
    [b_var.set(f"{b_t}{b:.3f}") for b_t, b_var, b in zip(b_text, b_vars, belief)]
    [b_label.pack() for b_label in b_labels]
    crit_var = tk.StringVar()
    crit_label = tk.Label(window, textvariable = crit_var)
    crit_label.pack()

    while True:
        window.update()
        [b_var.set(f"{b_t}{b:.3f}") for b_t, b_var, b in zip(b_text, b_vars, belief)]
        # read the current state of the robot + the xyz position
        state = utils.readState(conn)
        s = np.asarray(utils.joint2pose(state["q"]))
        # print(s)

        # get the humans joystick input
        z, mode, grasp, stop = interface.input()
        a_h = [0]*3
        a_h[0] = z[1]
        a_h[1] = z[0]
        a_h[2] = -z[2]
        a_h = np.asarray(a_h)
        if grasp and (time.time() - start_time > 1):
            gripper_closed = not gripper_closed
            start_time = time.time()
            utils.send2gripper(conn_gripper)
        if mode and (time.time() - start_time > 1):
            start_mode = not start_mode
            start_time = time.time()
            start_timer = time.time()
            print("[*] Started")

        # Stop if button pressed or if going to hit the cup thing
        hitting_cup = (s[0] > 0.45 and s[0] < 0.65 and s[1] > 0.02 and s[1] < 0.27 and s[2] < 0.26)
        if stop or hitting_cup or (not start_mode and time.time() - start_timer > 55):
            utils.end()
            pickle.dump(data, open(f"users/user{participant}/task2/data_method_{method}.pkl", "wb"))
            haptic.close(hapticconn)
            return_home(conn, home)
            print("[*] Done!")
            return True

        # get the next location to navigate towards
        def next_loc(trajectory, s, dist):
            try:
                return trajectory[np.argmin(np.linalg.norm(trajectory - s, axis = 1)) + dist]
            except:
                return trajectory[-1]

        # Make Goals at location in the future
        G = [next_loc(trajectory, s, 150) for trajectory in trajectories]

        # this is where we compute the belief
        belief = [b * np.exp(-BETA * utils.cost_to_go(s, a_h, g)) / np.exp(-BETA * utils.cost_to_go(s, 0*a_h, g)) for g, b in zip(G, belief)]
        belief /= np.sum(belief)
        # print(belief)  # This is the robot's current confidence

        # Individual and blended actions
        a_star = proportional_gain * np.asarray([g - s for g in G])
        a_star = [action_scale * a / np.linalg.norm(a) if np.linalg.norm(a) > action_scale else a for a in a_star]
        a_r = np.sum(a_star * belief[:, None], axis = 0)
        # human inputs converted to dx, dy, dz velocities in the end-effector space
        alpha = .6
        a = (1-alpha) * action_scale * a_h + alpha * np.asarray(a_r)
        a = np.pad(np.asarray(a), (0, 3), 'constant')

        # Critical States
        C = sum([b*(utils.cost_to_go(s, a_r, goal_x) - utils.cost_to_go(s, a_star_x, goal_x)) for b, a_star_x, goal_x in zip(belief, a_star, G)])
        id = np.identity(3)
        U_set = [[-1*action_scale*id[:, i], action_scale*id[:, i]] for i in range(3)]
        I_set = [utils.info_gain(BETA, U, s, G, belief) for U in U_set]
        # print(C, np.argmax(I_set))

        if C > 0.0161:
            if np.argmax(I_set) == 1:
                crit_var.set("Critical State X!")
                if not y_triggered and haptics_on:
                    print("Critical State X")
                    haptic.haptic_command(hapticconn, 'horizontal', 3, 1)
                    y_triggered = True

        if start_mode or time.time() - start_timer < motion_start or (s[2] < 0.075):
            a = [0]*6

        if (time.time() - lastsend) > sendfreq:
            # print("[*] Sending Updated Coordinates and Beliefs")
            send2hololens(belief, s, True, None, None)
            lastsend = time.time()

        # convert this command to joint space
        qdot = utils.xdot2qdot(a, state)
        # send our final command to robot
        utils.send2robot(conn, qdot)

        # every so many second save data
        if (time.time() - lastsave) > sendfreq and not start_mode:
            data.append([time.time() - task_start_time, state, s, G, a_h, a_star, a_r, belief, y_triggered])
            lastsave = time.time()


if __name__ == "__main__":
    main()
