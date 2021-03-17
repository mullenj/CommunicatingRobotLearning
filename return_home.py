import socket
import time
import numpy as np
import pickle
import pygame
import sys
import os
from datetime import datetime, timedelta
import subprocess
import signal

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

#home = np.asarray([0.000297636, -0.785294, -0.000218009, -2.3567, 0.000397658, 1.57042, 0.785205]) # Original Home with effector rotated 90 deg around z axis
#home = np.asarray([-0.272437, -0.523389, -0.339722, -2.608481,  1.12807,   1.153394, -1.352741]) # Facing away from user
home = np.asarray([ 0.709588, -0.446052,  0.020361, -2.536814, -1.168517,  0.98433,  -0.128633]) # real home
#home = np.asarray([0.136338, 0.26118, 0.157175, -1.80338, -0.00288155, 2.07467, 0.987893]) # for testing random trajectories

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
    #print(H)
    #print(H[:,3][:3])
    return H[:,3][:3]

def return_home(conn, home):
    print('[*] Returning to Home!')
    action_scale = 0.05

    total_time = 35.0
    start_time = time.time()
    dist = 1
    elapsed_time = time.time() - start_time

    while dist > 0.02 and elapsed_time < total_time:
        state = np.asarray(readState(conn)['q'].tolist())
        qdot = np.clip(home - state, -0.2, 0.2)
        send2robot(conn, qdot)
        dist = np.linalg.norm(state - home)
        elapsed_time = time.time() - start_time
    print('[*] Returned to Home!')

def main():

    PORT_robot = 8080
    action_scale = 0.05

    print('[*] Connecting to low-level controller...')

    conn = connect2robot(PORT_robot) #connect to other computer

    total_time = 35.0
    start_time = time.time()
    dist = 1
    elapsed_time = time.time() - start_time

    while dist > 0.02 and elapsed_time < total_time:
        state = np.asarray(readState(conn)['q'].tolist())
        qdot = np.clip(home - state, -0.2, 0.2)
        send2robot(conn, qdot)
        dist = np.linalg.norm(state - home)
        elapsed_time = time.time() - start_time
    print('[*] Done!')


if __name__ == "__main__":
    main()
