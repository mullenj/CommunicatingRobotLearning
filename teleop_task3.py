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
 * a minimal script for teleoperating the robot using a joystick
 * Dylan Losey, September 2020

 * To run:
 [1] in one terminal:
    navigate to ~/panda-ws/essentials
    run python3 teleop.py
 [2] in a second terminal:
    navigate to ~/libfranka/build
    run ./collab/velocity_control
"""
home = np.asarray([0.709588, -0.446052, 0.020361, -2.536814, -1.168517, 0.98433, -0.128633])  # real home
goal1 = np.asarray([0.627, -0.459, 0.629])  # Top Shelf flat
goal2 = np.asarray([0.634, -0.457, 0.318])  # Bottom Shelf flat
# goal3 = np.asarray([0.627, -0.459, 0.629, 1.59, -0.795, 0.027])  # Top Shelf sideways
# goal4 = np.asarray([0.634, -0.457, 0.318, 1.59, -0.795, 0.027])  # Bottom Shelf sideways
goals = [goal1, goal2] # , goal3, goal4]
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
    haptic_triggered = False

    print('[*] Connecting to low-level controller...')

    conn = utils.connect2robot(PORT_robot)  # connect to other computer
    conn_gripper = utils.connect2robot(PORT_gripper)

    # readState -> give you a dictionary with things like joint values, velocity, torque
    state = utils.readState(conn)
    # joint2pose -> forward kinematics: convert the joint position to the xyz position of the end-effector
    coord_home = np.asarray(utils.joint2pose(state["q"]))
    # print(coord_home)
    belief_dir = sys.argv[1]
    if belief_dir == 'top':
        belief = np.asarray([0.52, 0.48])
    elif belief_dir == 'bot':
        belief = np.asarray([0.48, 0.52])
    else:
        print("[*] Please input a valid prior ('top' or 'bot')")
        sys.exit()

    prior = np.copy(belief)
    BETA = 3
    start_mode = True
    gripper_closed = False
    latch_point = 0.95

    print('[*] Ready for a teleoperation...')

    # Set up the initial robotInit.txt file
    send2hololens(goals, belief, coord_home, False, latch_point)
    # Start the Web Server
    print('[*] Starting Server')
    server = subprocess.Popen(["python3", "server.py"])
    print('[*] Server Ready')
    lastsend = time.time() - sendfreq
    start_time = time.time()
    belief_time = time.time()

    while True:
        # read the current state of the robot + the xyz position
        state = utils.readState(conn)
        coord_curr = np.asarray(utils.joint2pose(state["q"]))
        # print(xyz_curr, rot_curr)
        # print(xyz_curr) THis is where the robot currently is

        # get the humans joystick input
        z, mode, grasp, stop = interface.input()
        if mode and (time.time() - start_time > 1):
            start_mode = not start_mode
            start_time = time.time()
            belief_time = time.time()
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
        # belief = probability that the human wants each goal
        # belief = [confidence in goal 1, confidence in goal 2]
        dist_start = np.linalg.norm(coord_curr - coord_home)
        # Could the below be done outside of comprehensions in numpy itself?
        dist_goals = [np.linalg.norm(goal_coord - coord_curr) for goal_coord in goals]
        dist_goals_star = [np.linalg.norm(goal_coord - coord_home) for goal_coord in goals]
        belief = [np.exp(-BETA * (dist_start + dist_goal)) / np.exp(-BETA * dist_goal_star) for dist_goal, dist_goal_star in zip(dist_goals, dist_goals_star)]
        belief = belief*np.asarray(prior)
        belief /= np.sum(belief)
        if not start_mode and (time.time() - belief_time > 1.0):
            # print("in update")
            prior = np.copy(belief)
            belief_time = time.time()
        # print(belief)  # This is the robot's current confidence

        xdot_g = [np.clip(goal - coord_curr, -0.05, 0.05) for goal in goals]
        # action_difference = np.abs(xdot_g1 - xdot_g2) # WHAT TO DO HERE??? not used elsewhere

        # Critial States implementation where cost is described as dist(s, g)
        # where s is the state and g is the goal location
        s_prime = xdot_g + coord_curr

        expected_cost = [sum(np.linalg.norm(goals - s_prime_x, axis = 1)*belief)
                         for s_prime_x in s_prime]
        if abs(expected_cost[0]-expected_cost[1]) > 0.009:
            print("Critical State")
            print(abs(expected_cost[0]-expected_cost[1]))
            print(belief)
            if not haptic_triggered:
                haptic.haptic_command(hapticconn, 'all_on', 2, 1)
                haptic_triggered = True

        # human inputs converted to dx, dy, dz velocities in the end-effector space
        xdot = [0]*6
        which_goal = np.argmax(belief)
        if max(belief) < latch_point:
            xdot[0] = action_scale * z[0] + xdot_g[which_goal][0] * 0.5
            xdot[1] = action_scale * -z[1] + xdot_g[which_goal][1] * 0.5
            xdot[2] = action_scale * -z[2] + xdot_g[which_goal][2] * 0.5
        else:
            xdot[0] = xdot_g[which_goal][0]
            xdot[1] = xdot_g[which_goal][1]
            xdot[2] = xdot_g[which_goal][2]

        if start_mode:
            xdot = [0]*6

        if (time.time() - lastsend) > sendfreq:
            # print("[*] Sending Updated Coordinates and Beliefs")
            send2hololens(goals, belief, coord_curr, True, None)
            lastsend = time.time()

        # convert this command to joint space
        qdot = utils.xdot2qdot(xdot, state)
        # send our final command to robot
        utils.send2robot(conn, qdot)

        # every so many iteration (every so many second 0.1)
        # data = [].append([joint positon, belief, input, ....])
        # when they press start , pickle.save(data)


if __name__ == "__main__":
    main()
