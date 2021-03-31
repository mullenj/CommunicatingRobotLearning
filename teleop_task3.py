import numpy as np
import os
import sys
import time
import subprocess
import signal
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
sendfreq = 0.1


'''
This function allows us to send all of the necessary information to the hololens.
This information changes based off of what state the program is in, initialized
or in progress.
'''
def send2hololens(goals, belief, coord_curr, initialized, latch_point):
    # print(belief)
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
    coord_home = np.asarray(utils.joint2pose(state["q"]))
    # print(coord_home)
    belief_dir = sys.argv[1]
    belief_depth = sys.argv[2]
    if belief_dir == 'top' and belief_depth == "edge":
        belief = np.asarray([0.65, 0.15, 0.15, 0.15])
    elif belief_dir == 'bot' and belief_depth == "edge":
        belief = np.asarray([0.15, 0.65, 0.15, 0.15])
    elif belief_dir == 'top' and belief_depth == "in":
        belief = np.asarray([0.15, 0.15, 0.65, 0.15])
    elif belief_dir == 'bot' and belief_depth == "in":
        belief = np.asarray([0.15, 0.15, 0.15, 0.65])
    else:
        print("[*] Please input a valid prior ('top' or 'bot')")
        sys.exit()

    BETA = 0.1
    start_mode = True
    gripper_closed = False
    gradient = 0.95

    print('[*] Ready for a teleoperation...')

    # Set up the initial robotInit.txt file
    send2hololens(G, belief, coord_home, False, gradient)
    # Start the Web Server
    print('[*] Starting Server')
    server = subprocess.Popen(["python3", "server.py"])
    print('[*] Server Ready')
    lastsend = time.time() - sendfreq
    start_time = time.time()

    while True:
        # read the current state of the robot + the xyz position
        state = utils.readState(conn)
        s = np.asarray(utils.joint2pose(state["q"]))
        # print(s)

        # get the humans joystick input
        a_h, mode, grasp, stop = interface.input()
        a_h[1] = -a_h[1]
        a_h[2] = -a_h[2]
        a_h = np.asarray(a_h)
        if mode and (time.time() - start_time > 1):
            start_mode = not start_mode
            start_time = time.time()
        if grasp and (time.time() - start_time > 1):
            gripper_closed = not gripper_closed
            start_time = time.time()
            utils.send2gripper(conn_gripper)

        if stop:
            os.killpg(os.getpgid(server.pid), signal.SIGTERM)
            haptic.close(hapticconn)
            # Run this command if the server doesnt stop correctly to find process you need to kill: lsof -i :8010
            return_home(conn, home)
            print("[*] Done!")
            return True

        # this is where we compute the belief
        belief = [b * np.exp(-BETA * utils.cost_to_go(s, a_h, g)) / np.exp(-BETA * utils.cost_to_go(s, 0*a_h, g)) for g, b in zip(G, belief)]
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
        print(C, np.argmax(I_set))

        if C > 0.011:
            if np.argmax(I_set) == 2:
                print("Critical State Z")
                if not z_triggered:
                    haptic.haptic_command(hapticconn, 'vertical', 3, 1)
                    z_triggered = True
            if np.argmax(I_set) == 1:
                print("Critical State Y")
                if not y_triggered:
                    haptic.haptic_command(hapticconn, 'horizontal', 3, 1)
                    y_triggered = True

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

        # every so many iteration (every so many second 0.1)
        # data = [].append([joint positon, belief, input, ....])
        # when they press start , pickle.save(data)


if __name__ == "__main__":
    main()
