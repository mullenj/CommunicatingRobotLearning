import socket
import time
import numpy as np
import pickle
import pygame
import sys


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


# hard coded two goal positions
goal1 = np.asarray([0.5, -0.5, 0.2])
goal2 = np.asarray([0.5, +0.5, 0.2])
goal3 = np.asarray([0.7, +0.0, 0.2])


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
    return H[:,3][:3]


def main():

    PORT_robot = 8080
    action_scale = 0.05
    interface = Joystick()

    print('[*] Connecting to low-level controller...')

    conn = connect2robot(PORT_robot) #connect to other computer

    # readState -> give you a dictionary with things like joint values, velocity, torque
    state = readState(conn)
    # joint2pose -> forward kinematics: convert the joint position to the xyz position of the end-effector
    xyz_home = joint2pose(state["q"])
    belief = np.asarray([0.5, 0.5])
    BETA = 2.5

    print('[*] Ready for a teleoperation...')

    while True:


        # read the current state of the robot + the xyz position
        state = readState(conn)
        xyz_curr = joint2pose(state["q"])
        # print(xyz_curr) THis is where the robot currently is

        # get the humans joystick input
        z, grasp, stop = interface.input()
        if stop:
            print("[*] Done!")
            return True

        # this is where we compute the belief
        # belief = probability that the human wants each goal
        # belief = [confidence in goal 1, confidence in goal 2]
        dist_start = np.linalg.norm(xyz_curr - xyz_home)
        dist_goal1 = np.linalg.norm(goal1 - xyz_curr)
        dist_goal2 = np.linalg.norm(goal2 - xyz_curr)
        dist_goal1_star = np.linalg.norm(goal1 - xyz_home)
        dist_goal2_star = np.linalg.norm(goal2 - xyz_home)
        belief[0] = np.exp(-BETA * (dist_start + dist_goal1)) / np.exp(-BETA * dist_goal1_star)
        belief[1] = np.exp(-BETA * (dist_start + dist_goal2)) / np.exp(-BETA * dist_goal2_star)
        belief /= np.sum(belief)
        # print(belief) THis is the robot's current confidence

        xdot_g1 = np.clip(goal1 - xyz_curr, -0.05, 0.05)
        xdot_g2 = np.clip(goal2 - xyz_curr, -0.05, 0.05)
        action_difference = np.abs(xdot_g1 - xdot_g2)

        # human inputs converted to dx, dy, dz velocities in the end-effector space
        xdot = [0]*6
        xdot[0] = action_scale * z[0]
        xdot[1] = action_scale * -z[1]
        xdot[2] = action_scale * -z[2]

        # if belief[0] > 0.7:
        #     xdot[0] = xdot_g1[0]
        #     xdot[1] = xdot_g1[1]
        #     xdot[2] = xdot_g1[2]

        # convert this command to joint space
        qdot = xdot2qdot(xdot, state)
        # send our final command to robot
        send2robot(conn, qdot)


if __name__ == "__main__":
    main()
