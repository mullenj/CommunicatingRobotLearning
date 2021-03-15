import socket
import time
import numpy as np
import math
import pickle
import pygame
import sys
import os
from datetime import datetime, timedelta
import subprocess
import signal
from return_home import return_home
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
# hard coded three goal positions
# goal1 = np.asarray([0.627, -0.459, 0.629]) # Top Shelf
# goal2 = np.asarray([0.634, -0.457, 0.318]) # Bottom Shelf
# goals = [goal1, goal2]
goal1 = np.asarray([0.627, -0.459, 0.629, 1.59, 0.766, 0.058]) # Top Shelf flat
goal2 = np.asarray([0.634, -0.457, 0.318, 1.59, 0.766, 0.058]) # Bottom Shelf flat
goal3 = np.asarray([0.627, -0.459, 0.629, 1.59, -0.795, 0.027]) # Top Shelf sideways
goal4 = np.asarray([0.634, -0.457, 0.318, 1.59, -0.795, 0.027]) # Bottom Shelf sideways
goals = [goal1, goal2, goal3, goal4]
sendfreq = timedelta(seconds=0.1)



class Joystick(object):

    def __init__(self):
        pygame.init()
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        self.deadband = 0.1

    def input(self):
        pygame.event.get()
        dx = self.gamepad.get_axis(0)
        dy = self.gamepad.get_axis(1)
        dz = self.gamepad.get_axis(4)
        if abs(dx) < self.deadband:
            dx = 0.0
        if abs(dy) < self.deadband:
            dy = 0.0
        if abs(dz) < self.deadband:
            dz = 0.0
        A_pressed = self.gamepad.get_button(0)
        START_pressed = self.gamepad.get_button(7)
        return [dx, dy, dz], A_pressed, START_pressed


def connect2robot(PORT):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('172.16.0.3', PORT))
    s.listen()
    conn, addr = s.accept()
    return conn

def send2robot(conn, qdot, limit=1.0):
    qdot = np.asarray(qdot)
    scale = np.linalg.norm(qdot)
    if scale > limit:
        qdot = np.asarray([qdot[i] * limit/scale for i in range(7)])
    send_msg = np.array2string(qdot, precision=5, separator=',',suppress_small=True)[1:-1]
    send_msg = "s," + send_msg + ","
    conn.send(send_msg.encode())

def listen2robot(conn):
    state_length = 7 + 7 + 7 + 42
    message = str(conn.recv(2048))[2:-2]
    state_str = list(message.split(","))
    for idx in range(len(state_str)):
        if state_str[idx] == "s":
            state_str = state_str[idx+1:idx+1+state_length]
            break
    try:
        state_vector = [float(item) for item in state_str]
    except ValueError:
        return None
    if len(state_vector) is not state_length:
        return None
    state_vector = np.asarray(state_vector)
    state = {}
    state["q"] = state_vector[0:7]
    state["dq"] = state_vector[7:14]
    state["tau"] = state_vector[14:21]
    state["J"] = state_vector[21:].reshape((7,6)).T
    return state

def readState(conn):
    while True:
        state = listen2robot(conn)
        if state is not None:
            break
    return state

def xdot2qdot(xdot, state):
    J_pinv = np.linalg.pinv(state["J"])
    return J_pinv @ np.asarray(xdot)

def joint2pose(q):
    def RotX(q):
        return np.array([[1, 0, 0, 0], [0, np.cos(q), -np.sin(q), 0], [0, np.sin(q), np.cos(q), 0], [0, 0, 0, 1]])
    def RotZ(q):
        return np.array([[np.cos(q), -np.sin(q), 0, 0], [np.sin(q), np.cos(q), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    def TransX(q, x, y, z):
        return np.array([[1, 0, 0, x], [0, np.cos(q), -np.sin(q), y], [0, np.sin(q), np.cos(q), z], [0, 0, 0, 1]])
    def TransZ(q, x, y, z):
        return np.array([[np.cos(q), -np.sin(q), 0, x], [np.sin(q), np.cos(q), 0, y], [0, 0, 1, z], [0, 0, 0, 1]])
    H1 = TransZ(q[0], 0, 0, 0.333)
    H2 = np.dot(RotX(-np.pi/2), RotZ(q[1]))
    H3 = np.dot(TransX(np.pi/2, 0, -0.316, 0), RotZ(q[2]))
    H4 = np.dot(TransX(np.pi/2, 0.0825, 0, 0), RotZ(q[3]))
    H5 = np.dot(TransX(-np.pi/2, -0.0825, 0.384, 0), RotZ(q[4]))
    H6 = np.dot(RotX(np.pi/2), RotZ(q[5]))
    H7 = np.dot(TransX(np.pi/2, 0.088, 0, 0), RotZ(q[6]))
    H_panda_hand = TransZ(-np.pi/4, 0, 0, 0.2105)
    H = np.linalg.multi_dot([H1, H2, H3, H4, H5, H6, H7, H_panda_hand])
    H_no_hand = np.linalg.multi_dot([H1, H2, H3, H4, H5, H6, H7])
    return np.concatenate((H[:,3][:3], rotationMatrixToEulerAngles(H_no_hand[:3,:3])), axis = None)

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])


def send2hololens(goals, belief, coord_curr, initialized):
    #print(belief)
    #print(xyz_curr)
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
            f.write(f"{coord_curr[0]}\t{coord_curr[1]}\t{coord_curr[2]}\t{coord_curr[3]}\t{coord_curr[4]}\t{coord_curr[5]}\tCurrent Location\n")
            for count, (goal, belief_x) in enumerate(zip(goals, belief)):
                if count < len(belief) - 1:
                    f.write(f"{goal[0]}\t{goal[1]}\t{goal[2]}\t{goal[3]}\t{goal[4]}\t{goal[5]}\tGoal {count + 1}\t{belief_x}\n")
                else:
                    f.write(f"{goal[0]}\t{goal[1]}\t{goal[2]}\t{goal[3]}\t{goal[4]}\t{goal[5]}\tGoal {count + 1}\t{belief_x}")


def main():

    PORT_robot = 8080
    action_scale = 0.05
    interface = Joystick()

    print('[*] Connecting to low-level controller...')

    conn = connect2robot(PORT_robot) #connect to other computer

    # readState -> give you a dictionary with things like joint values, velocity, torque
    state = readState(conn)
    # joint2pose -> forward kinematics: convert the joint position to the xyz position of the end-effector
    coord_home = np.asarray(joint2pose(state["q"]))
    # print(coord_home)
    belief = np.asarray([0.25, 0.25, 0.25, 0.25])
    BETA = 12
    translation_mode = True

    print('[*] Ready for a teleoperation...')

    #Set up the initial robotInit.txt file
    send2hololens(goals, belief, coord_home, False)
    #Start the Web Server
    print('[*] Starting Server')
    #server = subprocess.Popen(["python3", "server.py"])
    print('[*] Server Ready')
    lastsend = datetime.now() - sendfreq
    start_time = datetime.now()

    while True:


        # read the current state of the robot + the xyz position
        state = readState(conn)
        coord_curr = np.asarray(joint2pose(state["q"]))
        # print(xyz_curr, rot_curr)
        # print(xyz_curr) THis is where the robot currently is

        # get the humans joystick input
        z, mode, stop = interface.input()
        if mode and (datetime.now() - start_time > timedelta(seconds=0.2)):
            translation_mode = not translation_mode
            start_time = datetime.now()

        if stop:
            # os.killpg(os.getpgid(server.pid), signal.SIGTERM)
            #Run this command if the server doesnt stop correctly to find process you need to kill: lsof -i :8080
            return_home(conn)
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
        print(belief) # This is the robot's current confidence

        xdot_g = [np.clip(goal - coord_curr, -0.05, 0.05) for goal in goals]
        # action_difference = np.abs(xdot_g1 - xdot_g2) # WHAT TO DO HERE??? not used elsewhere

        # human inputs converted to dx, dy, dz velocities in the end-effector space

        xdot = [0]*6

        if translation_mode:
            if max(belief) < 0.3:
                xdot[0] = action_scale * z[0]
                xdot[1] = action_scale * -z[1]
                xdot[2] = action_scale * -z[2]
                xdot[4] = action_scale * -3 * z[1]
            elif max(belief) < 0.65:
                which_goal = np.argmax(belief)
                scalar = (max(belief) - 0.3)/(0.35*action_scale)
                xdot[0] = action_scale * (z[0] + xdot_g[which_goal][0]*scalar)
                xdot[1] = action_scale * (-z[1] + xdot_g[which_goal][1]*scalar)
                xdot[2] = action_scale * (-z[2] + xdot_g[which_goal][2]*scalar)
                xdot[3] = action_scale * (xdot_g[which_goal][3]*scalar)
                xdot[4] = action_scale * (z[1] * -3 + xdot_g[which_goal][4]*scalar)
                xdot[5] = action_scale * (xdot_g[which_goal][5]*scalar)
            else:
                which_goal = np.argmax(belief)
                xdot[0] = xdot_g[which_goal][0]
                xdot[1] = xdot_g[which_goal][1]
                xdot[2] = xdot_g[which_goal][2]
                xdot[3] = xdot_g[which_goal][3]
                xdot[4] = xdot_g[which_goal][4]
                xdot[5] = xdot_g[which_goal][5]
        else:
            xdot[3] = 2 * action_scale * z[0]
            xdot[4] = 2 * action_scale * z[1]
            xdot[5] = 2 * action_scale * z[2]

        if (datetime.now() - lastsend) > sendfreq:
            #print("[*] Sending Updated Coordinates and Beliefs")
            send2hololens(goals, belief, coord_curr, True)
            lastsend = datetime.now()

        # convert this command to joint space
        qdot = xdot2qdot(xdot, state)
        # send our final command to robot
        send2robot(conn, qdot)

        #every so many iteration (every so many second 0.1)
        #data = [].append([joint positon, belief, input, ....])
        #when they press start , pickle.save(data)


if __name__ == "__main__":
    main()
