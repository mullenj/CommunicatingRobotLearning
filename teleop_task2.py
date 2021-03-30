import numpy as np
import pickle
import os
import time
import subprocess
import signal
from return_home import return_home
import teleop_utils as utils
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
sendfreq = 0.1
trajectory_files = ["task2_path0", "task2_path1", "task2_path2", "task2_path3"]
home = np.asarray([0.000297636, -0.785294, -0.000218009, -2.3567, 0.000397658, 1.57042, 0.785205])


'''
This function allows us to send all of the necessary information to the hololens.
This information changes based off of what state the program is in, initialized
or in progress.
'''
def send2hololens(belief, coord_curr, initialized, waypoints, latch_point):
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

    PORT_robot = 8080
    PORT_gripper = 8081
    action_scale = 0.05
    interface = utils.Joystick()

    print('[*] Connecting to low-level controller...')

    conn = utils.connect2robot(PORT_robot) # connect to other computer
    conn_gripper = utils.connect2robot(PORT_gripper)

    # readState -> give you a dictionary with things like joint values, velocity, torque
    state = utils.readState(conn)
    # joint2pose -> forward kinematics: convert the joint position to the xyz position of the end-effector
    coord_home = np.asarray(utils.joint2pose(state["q"]))
    # print(coord_home)

    # Prepare Trajectories by loading them into the trajectory class and getting an array of points along the trajectory to compare against
    playback_time = 20.0
    proportional_gain = 5.0
    waypoints = [pickle.load(open(traj + ".pkl", "rb")) for traj in trajectory_files]
    trajectories = [utils.Trajectory(waypoint, playback_time) for waypoint in waypoints]
    trajectories = [np.asarray([utils.joint2pose(traj.get(time_i)) for time_i in list(np.linspace(0, playback_time, int(playback_time * 100 + 1)))]) for traj in trajectories]
    belief = np.asarray([0.25, 0.25, 0.25, 0.25])
    BETA = 12
    start_mode = True
    gripper_closed = False
    latch_point = 0.55

    print('[*] Ready for a teleoperation...')

    # Set up the initial robotInit.txt file
    send2hololens(belief, coord_home, False, waypoints, latch_point)
    # Start the Web Server
    print('[*] Starting Server')
    server = subprocess.Popen(["python3", "server.py"])
    print('[*] Server Ready')
    lastsend = time.time() - sendfreq
    start_time = time.time()

    while True:
        # read the current state of the robot + the xyz position
        state = utils.readState(conn)
        coord_curr = np.asarray(utils.joint2pose(state["q"]))

        # get the humans joystick input
        z, mode, grasp, stop = interface.input()
        if grasp and (time.time() - start_time > 1):
            gripper_closed = not gripper_closed
            start_time = time.time()
            utils.send2gripper(conn_gripper)
        if mode and (time.time() - start_time > 1):
            start_mode = not start_mode
            start_time = time.time()

        # Stop if button pressed or if going to hit the cup thing
        if stop or (coord_curr[0] > 0.45 and coord_curr[0] < 0.65 and coord_curr[1] > -0.02 and coord_curr[1] < 0.27 and coord_curr[2] < 0.26):
            os.killpg(os.getpgid(server.pid), signal.SIGTERM)
            # Run this command if the server doesnt stop correctly to find process you need to kill: lsof -i :8010
            return_home(conn, home)
            print("[*] Done!")
            return True

        # this is where we compute the belief
        # belief = probability that the human wants each goal
        # belief = [confidence in goal 1, confidence in goal 2]
        # Get the minimum distance to a trajectory in cartesian space, and set the belief
        dist_trajectories = [min(np.linalg.norm(trajectory - coord_curr, axis = 1)) for trajectory in trajectories]
        belief_denominator = sum(np.exp(-BETA * np.asarray(dist_trajectories)))
        belief = [np.exp(-BETA * dist_trajectory) / belief_denominator for dist_trajectory in dist_trajectories]
        belief /= np.sum(belief)
        # print(belief) # This is the robot's current confidence

        # get the next location to navigate towards
        def next_loc(trajectory, xyz):
            try:
                next_loc = trajectory[np.argmin(np.linalg.norm(trajectory - xyz, axis = 1)) + 10]
                return proportional_gain * (next_loc - xyz)
            except:
                print("Exception")
                return (0, 0, 0)
            finally:
                if (coord_curr[2] < 0.075):
                    return (0, 0, 0)

        a_star = [next_loc(trajectory, coord_curr) for trajectory in trajectories]
        a_star = [action_scale * a / np.linalg.norm(a) if np.linalg.norm(a) > action_scale else a for a in a_star]
        action = np.sum(a_star * belief[:, None], axis = 0)
        # action = a_star[np.argmax(belief)]
        if np.linalg.norm(action) > action_scale:
            action = action_scale * action / np.linalg.norm(action)
        if start_mode:
            action = (0, 0, 0)

        # Critical States (How to do this with trajectories as the goals)
        # One way: instead of feading goal feed closest point to s + a
        G = [min(np.linalg.norm(trajectory - (coord_curr + action), axis = 1)) for trajectory in trajectories]
        C = sum([b*(utils.cost_to_go(coord_curr, action, goal_x) - utils.cost_to_go(coord_curr, a_star_x, goal_x)) for b, a_star_x, goal_x in zip(belief, a_star, G)])
        print(C)
        # id = np.identity(3)
        # Quest = [[-0.1*id[:, i], 0.1*id[:, i]] for i in range(3)]
        # Ix = [utils.info_gain(Quest_x, coord_curr, goals, belief) for Quest_x in Quest]

        # human inputs converted to dx, dy, dz velocities in the end-effector space
        # xdot = [0]*6
        alpha = 1
        z[1] = -z[1]
        z[2] = -z[2]
        a = (1-alpha) * action_scale * np.asarray(z) + alpha * np.asarray(action)

        # Navigate
        # xdot = [0]*6
        # if max(belief) < 0.3:
        #     xdot[0] = action_scale * z[0] + 0.5 * action[0]
        #     xdot[1] = action_scale * -z[1] + 0.5 * action[1]
        #     xdot[2] = action_scale * -z[2] + 0.5 * action[2]
        # elif max(belief) < latch_point:
        #     scalar = 1 + (max(belief) - 0.3)/(0.25)
        #     xdot[0] = action_scale * z[0] + action[0] * scalar
        #     xdot[1] = action_scale * -z[1] + action[1] * scalar
        #     xdot[2] = action_scale * -z[2] + action[2] * scalar
        # else:
        #     xdot[0] = 2 * action[0]
        #     xdot[1] = 2 * action[1]
        #     xdot[2] = 2 * action[2]

        if (time.time() - lastsend) > sendfreq:
            # print("[*] Sending Updated Coordinates and Beliefs")
            send2hololens(belief, coord_curr, True, None, None)
            lastsend = time.time()

        # convert this command to joint space
        qdot = utils.xdot2qdot(np.pad(np.asarray(a), (0, 3), 'constant'), state)
        # send our final command to robot
        utils.send2robot(conn, qdot)

        # every so many iteration (every so many second 0.1)
        # data = [].append([joint positon, belief, input, ....])
        # when they press start , pickle.save(data)


if __name__ == "__main__":
    main()
