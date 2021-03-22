import socket
import time
import numpy as np
import pickle
import pygame
import sys
from scipy.interpolate import interp1d


"""
 * a minimal script for replaying trajectories or demonstrations
 * Dylan Losey, September 2020

 * To run:
 [1] in one terminal:
    navigate to ~/panda-ws/essentials
    run python3 playback_demo.py [your demo name]
 [2] in a second terminal:
    navigate to ~/libfranka/build
    run ./collab/velocity_control
"""


class Trajectory(object):

    def __init__(self, xi, T):
        """ create cublic interpolators between waypoints """
        self.xi = np.asarray(xi)
        self.T = T
        self.n_waypoints = xi.shape[0]
        timesteps = np.linspace(0, self.T, self.n_waypoints)
        self.f1 = interp1d(timesteps, self.xi[:,0], kind='cubic')
        self.f2 = interp1d(timesteps, self.xi[:,1], kind='cubic')
        self.f3 = interp1d(timesteps, self.xi[:,2], kind='cubic')
        self.f4 = interp1d(timesteps, self.xi[:,3], kind='cubic')
        self.f5 = interp1d(timesteps, self.xi[:,4], kind='cubic')
        self.f6 = interp1d(timesteps, self.xi[:,5], kind='cubic')
        self.f7 = interp1d(timesteps, self.xi[:,6], kind='cubic')

    def get(self, t):
        """ get interpolated position """
        if t < 0:
            q = [self.f1(0), self.f2(0), self.f3(0), self.f4(0), self.f5(0), self.f6(0), self.f7(0)]
        elif t < self.T:
            q = [self.f1(t), self.f2(t), self.f3(t), self.f4(t), self.f5(t), self.f6(t), self.f7(t)]
        else:
            q = [self.f1(self.T), self.f2(self.T), self.f3(self.T), self.f4(self.T), self.f5(self.T), self.f6(self.T), self.f7(self.T)]
        return np.asarray(q)


class Joystick(object):

    def __init__(self):
        pygame.init()
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        self.deadband = 0.1

    def input(self):
        pygame.event.get()
        z = self.gamepad.get_axis(0)
        if abs(z) < self.deadband:
            z = 0.0
        A_pressed = self.gamepad.get_button(0)
        START_pressed = self.gamepad.get_button(7)
        return [z], A_pressed, START_pressed


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


def main():

    demo_name = sys.argv[1]
    playback_time = 20.0
    proportional_gain = 5.0
    PORT = 8080

    print('[*] Connecting to low-level controller...')
    conn = connect2robot(PORT)
    interface = Joystick()

    print('[*] Reading trajectory I need to follow...')
    xi = pickle.load( open( demo_name + ".pkl", "rb" ) )
    traj = Trajectory(xi, playback_time)

    print('[*] Ready for replay, Press [A]...')
    timepressstart = time.time()
    curr_time = 0.0
    while True:
        z, start, stop = interface.input()
        if start:
            timepressstart = time.time()
            break

    print('[*] Replaying pickle file...')
    while True:

        state = readState(conn)

        z, start, stop = interface.input()
        if stop:
            return True

        q_des = traj.get(curr_time)
        action = proportional_gain * (q_des - state["q"])

        send2robot(conn, action)
        curr_time = time.time() - timepressstart
        if curr_time > playback_time + 1.0:
            print('[*] Finished replaying pickle file!')
            return True


if __name__ == "__main__":
    main()
