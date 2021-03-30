import numpy as np
import os
import time
import subprocess
import signal
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
goal1 = np.asarray([0.627, -0.459, 0.629, 1.59, 0.766, 0.058])  # Top Shelf flat
goal2 = np.asarray([0.634, -0.457, 0.318, 1.59, 0.766, 0.058])  # Bottom Shelf flat
goal3 = np.asarray([0.627, -0.459, 0.629, 1.59, -0.795, 0.027])  # Top Shelf sideways
goal4 = np.asarray([0.634, -0.457, 0.318, 1.59, -0.795, 0.027])  # Bottom Shelf sideways
goals = [goal1, goal2, goal3, goal4]
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

    PORT_robot = 8080
    PORT_gripper = 8081
    action_scale = 0.05
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
    coord_home = np.asarray(utils.joint2posewrot(state["q"]))
    # print(coord_home)
    belief = np.asarray([0.25, 0.25, 0.25, 0.25])
    BETA = 12
    translation_mode = True
    gripper_closed = False
    latch_point = 0.75

    print('[*] Ready for a teleoperation...')

    # Set up the initial robotInit.txt file
    send2hololens(goals, belief, coord_home, False, latch_point)
    # Start the Web Server
    print('[*] Starting Server')
    server = subprocess.Popen(["python3", "server.py"])
    print('[*] Server Ready')
    lastsend = time.time() - sendfreq
    start_time = time.time()

    while True:
        # read the current state of the robot + the xyz position
        state = utils.readState(conn)
        coord_curr = np.asarray(utils.joint2posewrot(state["q"]))
        # print(xyz_curr, rot_curr)
        # print(xyz_curr) THis is where the robot currently is

        # get the humans joystick input
        z, mode, grasp, stop = interface.input()
        if mode and (time.time() - start_time > 1):
            translation_mode = not translation_mode
            start_time = time.time()
        if grasp and (time.time() - start_time > 1):
            gripper_closed = not gripper_closed
            start_time = time.time()
            utils.send2gripper(conn_gripper)

        if stop:
            os.killpg(os.getpgid(server.pid), signal.SIGTERM)
            # Run this command if the server doesnt stop correctly to find process you need to kill: lsof -i :8010
            return_home(conn, home)
            print("[*] Done!")
            return True

        # this is where we compute the belief
        # belief = probability that the human wants each goal
        # belief = [confidence in goal 1, confidence in goal 2]
        dist_start = np.linalg.norm(coord_curr - coord_home)
        dist_goals = [np.linalg.norm(goal_coord - coord_curr) for goal_coord in goals]
        dist_goals_star = [np.linalg.norm(goal_coord - coord_home) for goal_coord in goals]
        belief = [np.exp(-BETA * (dist_start + dist_goal)) / np.exp(-BETA * dist_goal_star) for dist_goal, dist_goal_star in zip(dist_goals, dist_goals_star)]
        belief /= np.sum(belief)
        # print(belief)  # This is the robot's current confidence

        # Individual and blended actions
        a_star = [np.clip(goal - coord_curr, -0.05, 0.05) for goal in goals]
        a_star = [action_scale * a / np.linalg.norm(a) if np.linalg.norm(a) > action_scale else a for a in a_star]
        action = np.sum(a_star * belief[:, None], axis = 0)
        if np.linalg.norm(action) > action_scale:
            action = action_scale * action / np.linalg.norm(action)
        # human inputs converted to dx, dy, dz velocities in the end-effector space
        # xdot = [0]*6
        alpha = 1
        a = (1-alpha) * action_scale * np.pad(np.asarray(z), (0, 3), 'constant') + alpha * np.asarray(action)
        a[4] = a[4] / 0.15
        # Critical States
        C = sum([b*(utils.cost_to_go(coord_curr, action, goal_x) - utils.cost_to_go(coord_curr, a_star_x, goal_x)) for b, a_star_x, goal_x in zip(belief, a_star, goals)])

        id = np.identity(6)
        Quest = [[-0.1*id[:, i], 0.1*id[:, i]] for i in range(6)]
        Ix = [utils.info_gain(Quest_x, coord_curr, goals, belief) for Quest_x in Quest]
        print(C, np.argmax(Ix))

        # Naive implementation of Haptics
        if C > 0.017:
            if np.argmax(Ix) == 2:
                print("Critical State Z")
                if not z_triggered:
                    haptic.haptic_command(hapticconn, 'vertical', 3, 1)
                    z_triggered = True
            elif np.argmax(Ix) > 2:
                print("Critical State Rotation")
                if not rot_triggered:
                    haptic.haptic_command(hapticconn, 'circular', 3, 1)
                    rot_triggered = True

        # if translation_mode:
        #     if max(belief) < 0.3:
        #         xdot[0] = action_scale * z[0]
        #         xdot[1] = action_scale * -z[1]
        #         xdot[2] = action_scale * -z[2]
        #         xdot[4] = action_scale * -3 * z[1]
        #     elif max(belief) < latch_point:
        #         scalar = (max(belief) - 0.3)/(0.45*action_scale)
        #         xdot[0] = action_scale * (z[0] + action[0]*scalar)
        #         xdot[1] = action_scale * (-z[1] + action[1]*scalar)
        #         xdot[2] = action_scale * (-z[2] + action[2]*scalar)
        #         xdot[3] = action_scale * (action[3]*scalar)
        #         xdot[4] = action_scale * (z[1] * -3 + action[4]*scalar)
        #         xdot[5] = action_scale * (action[5]*scalar)
        #     else:
        #         xdot[0] = action[0]
        #         xdot[1] = action[1]
        #         xdot[2] = action[2]
        #         xdot[3] = 4 * action[3]
        #         xdot[4] = 4 * action[4]
        #         xdot[5] = 4 * action[5]
        # else:
        #     xdot[3] = 2 * action_scale * z[0]
        #     xdot[4] = 2 * action_scale * z[1]
        #     xdot[5] = 2 * action_scale * z[2]

        if (time.time() - lastsend) > sendfreq:
            # print("[*] Sending Updated Coordinates and Beliefs")
            send2hololens(goals, belief, coord_curr, True, None)
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
