import numpy as np
import sys
import time
import tkinter as tk
from return_home import return_home
import teleop_utils as utils
import hapticcode.haptic_control as haptic
np.set_printoptions(suppress=True)

"""
 * a minimal script for teleoperating the robot using a joystick
 * Three goals are present and the system operates similarly to the tasks
 * The user should be able to acclimate to the methods using this
 * Dylan Losey, September 2020
 * James Mullen, April 2021

 * To run:
 [1] in one terminal:
    navigate to ~/panda-ws/essentials
    run python3 teleop.py
 [2] in a second terminal:
    navigate to ~/libfranka/build
    run ./collab/velocity_control
"""


# hard coded three goal positions
goal1 = np.asarray([0.63, -0.0525, 0.12])
goal2 = np.asarray([0.55, 0.37, 0.077])
goal3 = np.asarray([0.08, 0.4, 0.1])
G = [goal1, goal2, goal3]
sendfreq = 0.1
home = np.asarray([0.000297636, -0.785294, -0.000218009, -2.3567, 0.000397658, 1.57042, 0.785205])


def send2hololens(goals, belief, coord_curr, initialized, latch_point):
    # print(belief)
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

    method = sys.argv[1]
    haptics_on = method == "B" or method == "D"

    PORT_robot = 8080
    action_scale = 0.05
    interface = utils.Joystick()
    print('[*] Connecting to haptic device...')
    hapticconn = haptic.initialize()
    x_triggered = False
    y_triggered = False

    print('[*] Connecting to low-level controller...')

    conn = utils.connect2robot(PORT_robot)  # connect to other computer

    # readState -> give you a dictionary with things like joint values, velocity, torque
    state = utils.readState(conn)
    # joint2pose -> forward kinematics: convert the joint position to the xyz position of the end-effector
    s_home = np.asarray(utils.joint2pose(state["q"]))

    belief = [0.33, 0.33, 0.33]

    BETA = 0.05
    start_mode = True
    gradient = 0.8

    print('[*] Ready for a teleoperation...')

    # Set up the initial robotInit.txt file
    send2hololens(G, belief, s_home, False, gradient)

    # Set up timers
    lastsend = time.time() - sendfreq
    start_time = time.time()
    haptic_timer = time.time()

    window = tk.Tk()
    window.title("User Study GUI")
    text = tk.Label(text="The below table represents the belief as a percentage for each goal.")
    text.pack()
    b_text = ["Cup:     ", "Plate:   ", "Pad:     "]
    b_vars = [tk.StringVar(), tk.StringVar(), tk.StringVar()]
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
        if mode and (time.time() - start_time > 1):
            start_mode = not start_mode
            start_time = time.time()

        if stop:
            utils.end()
            haptic.close(hapticconn)
            return_home(conn, home)
            print("[*] Done!")
            return True

        # this is where we compute the belief
        belief = [b * np.exp(-BETA * utils.cost_to_go(s, 0.5*a_h, g)) / np.exp(-BETA * utils.cost_to_go(s, 0*a_h, g)) for g, b in zip(G, belief)]
        belief /= np.sum(belief)
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

        if C > 0.035:
            if np.argmax(I_set) == 0:
                crit_var.set("Critical State X!")
                if not x_triggered and haptics_on:
                    print("Critical State X")
                    haptic.haptic_command(hapticconn, 'vertical', 3, 1)
                    x_triggered = True
                    haptic_timer = time.time()
            if C > 0.4 and np.argmax(I_set) == 1 and time.time() - haptic_timer > 5:
                crit_var.set("Critical State Y!")
                if not y_triggered and haptics_on:
                    print("Critical State Y")
                    haptic.haptic_command(hapticconn, 'horizontal', 3, 1)
                    y_triggered = True
        else:
            crit_var.set("")

        if start_mode:
            a = [0]*6

        if (time.time() - lastsend) > sendfreq:
            # print("[*] Sending Updated Coordinates and Beliefs")
            send2hololens(G, belief, s, True, None)
            lastsend = time.time()

        # convert this command to joint space
        qdot = utils.xdot2qdot(a, state)
        # send our final command to robot
        utils.send2robot(conn, qdot)


if __name__ == "__main__":
    main()
