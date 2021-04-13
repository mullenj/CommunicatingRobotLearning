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
 * a script for teleoperating the robot using a joystick in task 3 of the senior
 * design user study. It is set up to have two final location goals and the
 * study operator sets which goal it prefers. When hitting a critical state, the
 * user is notified to provide corrections if necessary.
 * Dylan Losey, September 2020
 * James Mullen, March 2021

 * To run:
 [1] in one terminal:
    navigate to ~/panda-ws/essentials
    run python3 teleop.py
 [2] in a second terminal:
    navigate to ~/libfranka/build
    run ./collab/velocity_control
    run ./collab/grasp_control
"""
home = np.asarray([0.709588, -0.446052, 0.020361, -2.536814, -1.168517, 0.98433, -0.128633])  # real home
goal1 = np.asarray([0.45, -0.485, 0.65])  # Top Shelf edge
goal2 = np.asarray([0.45, -0.485, 0.222])  # Bottom Shelf edge
goal3 = np.asarray([0.45, -0.65, 0.65])  # Top Shelf In
goal4 = np.asarray([0.45, -0.65, 0.222])  # Bottom Shelf In
G = [goal1, goal2, goal3, goal4] # , goal3, goal4]
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
            f.write(f"{coord_curr[0]}\t{coord_curr[1]}\t{coord_curr[2]}\n")
            for count, belief_x in enumerate(belief):
                if count < len(belief) - 1:
                    f.write(f"{belief_x}\n")
                else:
                    f.write(f"{belief_x}")
    else:
        with open('robotInit.txt', 'w') as f:
            f.write(f"{latch_point}\t{coord_curr[0]}\t{coord_curr[1]}\t{coord_curr[2]}\n")
            for count, (goal, belief_x) in enumerate(zip(goals, belief)):
                if count < len(belief) - 1:
                    f.write(f"{goal[0]}\t{goal[1]}\t{goal[2]}\t{belief_x}\n")
                else:
                    f.write(f"{goal[0]}\t{goal[1]}\t{goal[2]}\t{belief_x}")


def main():

    participant = sys.argv[1]
    method = sys.argv[2]
    haptics_on = method == "B" or method == "D"
    prior_command = sys.argv[3]

    PORT_robot = 8080
    PORT_gripper = 8081
    action_scale = 0.05
    interface = utils.Joystick()
    print('[*] Connecting to haptic device...')
    hapticconn = haptic.initialize()
    z_triggered = False
    y_triggered = False

    print('[*] Connecting to low-level controller...')

    conn = utils.connect2robot(PORT_robot)  # connect to other computer
    conn_gripper = utils.connect2robot(PORT_gripper)

    # readState -> give you a dictionary with things like joint values, velocity, torque
    state = utils.readState(conn)
    # joint2pose -> forward kinematics: convert the joint position to the xyz position of the end-effector
    s_home = np.asarray(utils.joint2pose(state["q"]))

    BETA = 0.05
    start_mode = True
    gripper_closed = False
    gradient = 0.7

    belief = [0.25, 0.25, 0.25, 0.25]
    prior_set = False
    prior = np.asarray([0, 0, 0.5, 0.5])
    if prior_command == "B":
        next_prior = [0, 0, 0, 1]
        BETA = 0.005
    elif prior_command != "A" and prior_command != "C":
        print("[*] Please input a valid prior ('A'. 'B', or 'C')")
        sys.exit()

    print('[*] Ready for a teleoperation...')

    # Set up the initial robotInit.txt file
    send2hololens(G, belief, s_home, False, gradient)

    # Set up timers
    lastsend = time.time() - sendfreq
    lastsave = time.time() - sendfreq
    start_time = time.time()
    task_start_time = time.time()
    start_timer = time.time()
    gui_update_timer = time.time()
    motion_start = random.uniform(1, 5)
    set_prior_time = motion_start + random.uniform(4, 8)
    prior_timer = time.time()
    prior_timer_start = False
    time_controlled = 0

    dist = 1

    # Data variable for saving to pickle file
    data = []

    window = tk.Tk()
    window.title("User Study GUI")
    text = tk.Label(text="The below table represents the belief as a percentage for each goal.")
    text.pack()
    b_text = ["Top Shelf, Edge:          ", "Bottom Shelf, Edge:    ", "Top Shelf, Inside:        ", "Bottom Shelf, Inside:  "]
    b_vars = [tk.StringVar(), tk.StringVar(), tk.StringVar(), tk.StringVar()]
    b_labels = [tk.Label(window, textvariable = b_var) for b_var in b_vars]
    [b_var.set(f"{b_t}{b:.3f}") for b_t, b_var, b in zip(b_text, b_vars, belief)]
    [b_label.pack() for b_label in b_labels]
    crit_var = tk.StringVar()
    crit_label = tk.Label(window, textvariable = crit_var)
    crit_label.pack()

    while True:
        window.update()
        if time.time() - gui_update_timer < 3:
            [b_var.set(f"{b_t}{b:.3f}") for b_t, b_var, b in zip(b_text, b_vars, belief)]
            gui_update_timer = time.time()
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
        if prior_command == "B" and not prior_timer_start and not np.array_equal(a_h, np.asarray([0]*3)):
            prior_timer_start = True
            prior_timer = time.time()
        if prior_command == "B" and prior_timer_start and not np.array_equal(a_h, np.asarray([0]*3)):
            time_controlled += time.time() - prior_timer
            prior_timer = time.time()
        if mode and start_mode and (time.time() - start_time > 1):
            start_mode = False
            start_time = time.time()
            start_timer = time.time()
            print("[*] Started")
            pickle.dump(True, open("game_start.pkl", "wb"))
        if grasp and (time.time() - start_time > 1):
            gripper_closed = not gripper_closed
            start_time = time.time()
            utils.send2gripper(conn_gripper)

        if stop or dist < 0.03:
            pickle.dump(False, open("game_start.pkl", "wb"))
            utils.end()
            pickle.dump(data, open(f"users/user{participant}/task3/data_method_{method}_prior_{prior_command}.pkl", "wb"))
            haptic.close(hapticconn)
            return_home(conn, home)
            print("[*] Done!")
            return True

        # this is where we compute the belief
        belief = [b * np.exp(-BETA * utils.cost_to_go(s, 0.4*a_h, g)) / np.exp(-BETA * utils.cost_to_go(s, 0*a_h, g)) for g, b in zip(G, belief)]
        belief /= np.sum(belief)
        if not start_mode and not prior_set and (time.time() - start_timer > set_prior_time):
            print(start_timer, prior_set)
            prior_set = True
            belief = prior
        if prior_command == "B" and prior_timer_start and time_controlled > 1.5:
            belief = np.asarray(next_prior)
        elif belief[1] == 0 and prior_command == "C" and prior_timer_start and (time.time() - prior_timer > 24):
            belief[1] += 0.75

        # print(belief)  # This is the robot's current confidence

        # Individual and blended actions
        a_star = [g - s for g in G]
        a_star = [action_scale * a / np.linalg.norm(a) if np.linalg.norm(a) > action_scale else a for a in a_star]
        a_r = np.sum(a_star * belief[:, None], axis = 0)
        # human inputs converted to dx, dy, dz velocities in the end-effector space
        alpha = 0.4
        a = (1-alpha) * action_scale * a_h + alpha * np.asarray(a_r)
        a = np.pad(np.asarray(a), (0, 3), 'constant')

        # Critical States
        C = sum([b*(utils.cost_to_go(s, a_r, goal_x) - utils.cost_to_go(s, a_star_x, goal_x)) for b, a_star_x, goal_x in zip(belief, a_star, G)])
        id = np.identity(3)
        U_set = [[-1*action_scale*id[:, i], action_scale*id[:, i]] for i in range(3)]
        I_set = [utils.info_gain(BETA, U, s, G, belief) for U in U_set]
        # print(C, np.argmax(I_set))

        if C > 0.011:
            if np.argmax(I_set) == 2:
                crit_var.set("Critical State Z!")
                if not z_triggered and haptics_on:
                    print("Critical State Z")
                    haptic.haptic_command(hapticconn, 'vertical', 3, 1)
                    z_triggered = True
            if np.argmax(I_set) == 1:
                crit_var.set("Critical State Y!")
                if not y_triggered and haptics_on:
                    print("Critical State Y")
                    haptic.haptic_command(hapticconn, 'horizontal', 3, 1)
                    y_triggered = True
        else:
            crit_var.set("")

        if start_mode or time.time() - start_timer < motion_start:
            a = [0]*6
        elif not prior_timer_start and prior_command != "B":
            prior_timer_start = True
            prior_timer = time.time()

        if (time.time() - lastsend) > sendfreq:
            # print("[*] Sending Updated Coordinates and Beliefs")
            send2hololens(G, belief, s, True, None)
            lastsend = time.time()

        # convert this command to joint space
        qdot = utils.xdot2qdot(a, state)
        # send our final command to robot
        utils.send2robot(conn, qdot)

        dist = np.linalg.norm(s[:3] - G[3][:3])

        # every so many second save data
        if (time.time() - lastsave) > sendfreq and not start_mode:
            data.append([time.time() - task_start_time, state, s, G, a_h, a_star, a_r, belief, C, z_triggered, y_triggered])
            lastsave = time.time()


if __name__ == "__main__":
    main()
