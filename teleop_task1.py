import numpy as np
import sys
import time
import pickle
import random
import tkinter as tk
from return_home import return_home
import teleop_utils as utils
import hapticcode.haptic_control as haptic
np.set_printoptions(suppress=True)

"""
 * a script for teleoperating the robot using a joystick in task 1 of the senior
 * design user study. It sets up four goals in two locations with two possible
 * orientations. The user must adapt to the injected rotation to correct it or
 * the code will latch onto a goal that the user does not intend.
 * Dylan Losey, September 2020
 * James Mullen, March 2021

 * To run:
 [1] in one terminal:
    navigate to ~/panda-ws/essentials
    run python3 teleop_task1.py
 [2] in a second terminal:
    navigate to ~/libfranka/build
    run ./collab/velocity_control
    run ./collab/grasp_control
"""
home = np.asarray([0.709588, -0.446052, 0.020361, -2.536814, -1.168517, 0.98433, -0.128633])  # real home
goal1 = np.asarray([0.45, -0.485, 0.65, 1.59, 0.766, 0.058])  # Top Shelf flat
goal2 = np.asarray([0.45, -0.485, 0.222, 1.59, 0.766, 0.058])  # Bottom Shelf flat
goal3 = np.asarray([0.45, -0.485, 0.65, 1.59, -0.795, 0.027])  # Top Shelf sideways
goal4 = np.asarray([0.45, -0.485, 0.222, 1.59, -0.795, 0.027])  # Bottom Shelf sideways
G = [goal1, goal2, goal3, goal4]
sendfreq = 0.1 # (also sets data saving)

'''
This function allows us to send all of the necessary information to the hololens.
This information changes based off of what state the program is in, initialized
or in progress.
'''
def send2hololens(goals, belief, coord_curr, initialized, latch_point):
    print(belief)
    # print(xyz_curr)
    if initialized:
        with open('robotUpdate.txt', 'w') as f:
            f.write(f"{coord_curr[0]}\t{coord_curr[1]}\t{coord_curr[2]}\t{coord_curr[3]}\t{coord_curr[4]}\t{coord_curr[5]}\n")
            for count, (goal, belief_x) in enumerate(zip(goals, belief)):
                if count < len(belief) - 1:
                    f.write(f"{belief_x}\n")
                else:
                    f.write(f"{belief_x}")
    else:
        with open('robotInit.txt', 'w') as f:
            f.write(f"{latch_point}\t{coord_curr[0]}\t{coord_curr[1]}\t{coord_curr[2]}\t{coord_curr[3]}\t{coord_curr[4]}\t{coord_curr[5]}\n")
            for count, (goal, belief_x) in enumerate(zip(goals, belief)):
                if count < len(belief) - 1:
                    f.write(f"{goal[0]}\t{goal[1]}\t{goal[2]}\t{goal[3]}\t{goal[4]}\t{goal[5]}\t{belief_x}\n")
                else:
                    f.write(f"{goal[0]}\t{goal[1]}\t{goal[2]}\t{goal[3]}\t{goal[4]}\t{goal[5]}\t{belief_x}")


def main():

    participant = sys.argv[1]
    method = sys.argv[2]
    haptics_on = method == "B" or method == "D"

    PORT_robot = 8080
    PORT_gripper = 8081
    action_scale = 0.05
    action_scale_rot = 0.25
    interface = utils.Joystick()
    print('[*] Connecting to haptic device...')
    hapticconn = haptic.initialize()
    rot_triggered = False
    z_triggered = False

    print('[*] Connecting to low-level controller...')

    conn = utils.connect2robot(PORT_robot)  # connect to other computer
    conn_gripper = utils.connect2robot(PORT_gripper)

    # readState -> give you a dictionary with things like joint values, velocity, torque
    state = utils.readState(conn)
    # joint2pose -> forward kinematics: convert the joint position to the xyz position of the end-effector
    s_home = np.asarray(utils.joint2posewrot(state["q"]))

    belief = np.asarray([0.2, 0.2, 0.3, 0.3])
    BETA = 0.05
    translation_mode = True
    start_mode = True
    gripper_closed = False
    gradient = 0.65
    dist = 1

    print('[*] Ready for a teleoperation...')

    # Set up the initial robotInit.txt file
    send2hololens(G, belief, s_home, False, gradient)

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
    b_text = ["Top Shelf, Upright:            ", "Bottom Shelf, Upright:      ", "Top Shelf, Sideways:        ", "Bottom Shelf, Sideways:  "]
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
        s = np.asarray(utils.joint2posewrot(state["q"]))
        # print(s)

        # get the humans joystick input
        z, mode, grasp, stop = interface.input()
        a_h = [0]*3
        a_h[0] = z[1]
        a_h[1] = z[0]
        a_h[2] = -z[2]
        a_h = np.asarray(a_h)
        if mode and (time.time() - start_time > 1):
            if start_mode:
                print("[*] Started")
                start_mode = not start_mode
                start_timer = time.time()
            else:
                translation_mode = not translation_mode
            start_time = time.time()
        if grasp and (time.time() - start_time > 1):
            gripper_closed = not gripper_closed
            start_time = time.time()
            utils.send2gripper(conn_gripper)

        if stop or dist < 0.03 or (not start_mode and time.time() - start_timer > 45):
            utils.end()
            pickle.dump(data, open(f"users/user{participant}/task1/data_method_{method}.pkl", "wb"))
            haptic.close(hapticconn)
            return_home(conn, home)
            print("[*] Done!")
            return True

        a_h = np.pad(a_h, (0, 3), 'constant')
        if not translation_mode:
            a_h = np.pad(np.asarray(z), (3, 0), 'constant')
            a_h[3] = 0
            a_h[5] = 0

        # this is where we compute the belief
        belief_trans = [b * np.exp(-BETA * utils.cost_to_go(s[:3], a_h[:3], g[:3])) / np.exp(-BETA * utils.cost_to_go(s[:3], 0*a_h[:3], g[:3])) for g, b in zip(G, belief)]
        belief_rot = [b * np.exp(-2*BETA * utils.cost_to_go(s[3:], a_h[3:], g[3:])) / np.exp(-2*BETA * utils.cost_to_go(s[3:], 0*a_h[3:], g[3:])) for g, b in zip(G, belief)]
        belief = np.sum((belief_trans, belief_rot), axis = 0)
        belief /= np.sum(belief)
        # print(belief)

        # Individual and blended actions
        a_star = [g - s for g in G]
        a_star_trans = np.asarray(a_star)[:, :3]
        a_star_rot = np.asarray(a_star)[:, 3:]
        a_star_trans = [action_scale * a / np.linalg.norm(a) if np.linalg.norm(a) > action_scale else a for a in a_star_trans]
        a_star_rot = [action_scale_rot * a / np.linalg.norm(a) if np.linalg.norm(a) > action_scale_rot else a for a in a_star_rot]
        a_star = np.concatenate((np.asarray(a_star_trans), np.asarray(a_star_rot)), axis = 1)
        a_r = np.sum(a_star * belief[:, None], axis = 0)
        # human inputs converted to dx, dy, dz velocities in the end-effector space
        # xdot = [0]*6
        alpha = 0.4
        a = (1-alpha) * action_scale * a_h + alpha * np.asarray(a_r)

        # Critical States
        C = sum([b*(utils.cost_to_go(s, a_r, goal_x) - utils.cost_to_go(s, a_star_x, goal_x)) for b, a_star_x, goal_x in zip(belief, a_star, G)])
        id = np.identity(6)
        U_set = [[-action_scale*id[:, i], action_scale*id[:, i]] for i in range(6)]
        I_set = [utils.info_gain(BETA, U, s, G, belief) for U in U_set]
        # print(C, np.argmax(I_set))

        # Naive implementation of Haptics
        if C > 0.066:
            if np.argmax(I_set) == 2:
                crit_var.set("Critical State Z!")
                if not z_triggered and haptics_on:
                    print("Critical State Z")
                    haptic.haptic_command(hapticconn, 'vertical', 3, 1)
                    z_triggered = True
            elif np.argmax(I_set) > 2 and C > 0.15:
                crit_var.set("Critical State Rotation!")
                if not rot_triggered and haptics_on:
                    print("Critical State Rotation")
                    haptic.haptic_command(hapticconn, 'circular', 3, 1)
                    rot_triggered = True

        if not translation_mode:
            a = action_scale_rot * np.pad(np.asarray(z), (3, 0), 'constant')
            a[3] = 0
            a[5] = 0

        if start_mode or time.time() - start_timer < motion_start:
            a = [0]*6

        if (time.time() - lastsend) > sendfreq:
            # print("[*] Sending Updated Coordinates and Beliefs")
            send2hololens(G, belief, s, True, None)
            lastsend = time.time()

        # convert this command to joint space
        qdot = utils.xdot2qdot(a, state)
        # send our final command to robot
        utils.send2robot(conn, qdot)

        dist = np.min([np.linalg.norm(s[:3] - g[:3]) for g in G])
        # print(dist)

        # every so many second save data
        if (time.time() - lastsave) > sendfreq and not start_mode:
            data.append([time.time() - task_start_time, state, s, G, a_h, a_star, a_r, belief, z_triggered, rot_triggered])
            lastsave = time.time()


if __name__ == "__main__":
    main()
