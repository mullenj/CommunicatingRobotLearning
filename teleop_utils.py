import socket
import numpy as np
import math
import pygame
from scipy.interpolate import interp1d

"""
 * a script with utilities for teleoperating the robot using a joystick.
 * Functions are to be imported to teleop_task*.py scripts
 * Dylan Losey, September 2020
"""
class Trajectory(object):

    def __init__(self, xi, T):
        """ create cublic interpolators between waypoints """
        self.xi = np.asarray(xi)
        self.T = T
        self.n_waypoints = xi.shape[0]
        timesteps = np.linspace(0, self.T, self.n_waypoints)
        self.f1 = interp1d(timesteps, self.xi[:, 0], kind='cubic')
        self.f2 = interp1d(timesteps, self.xi[:, 1], kind='cubic')
        self.f3 = interp1d(timesteps, self.xi[:, 2], kind='cubic')
        self.f4 = interp1d(timesteps, self.xi[:, 3], kind='cubic')
        self.f5 = interp1d(timesteps, self.xi[:, 4], kind='cubic')
        self.f6 = interp1d(timesteps, self.xi[:, 5], kind='cubic')
        self.f7 = interp1d(timesteps, self.xi[:, 6], kind='cubic')

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
        B_pressed = self.gamepad.get_button(1)
        START_pressed = self.gamepad.get_button(7)
        return [dx, dy, dz], A_pressed, B_pressed, START_pressed

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
        qdot = np.asarray([qdot[i] * limit / scale for i in range(7)])
    send_msg = np.array2string(qdot, precision=5, separator=',', suppress_small=True)[1:-1]
    send_msg = "s," + send_msg + ","
    conn.send(send_msg.encode())


def send2gripper(conn):
    send_msg = "s"
    conn.send(send_msg.encode())


def listen2robot(conn):
    state_length = 7 + 7 + 7 + 42
    message = str(conn.recv(2048))[2:-2]
    state_str = list(message.split(","))
    for idx in range(len(state_str)):
        if state_str[idx] == "s":
            state_str = state_str[idx + 1:idx + 1 + state_length]
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
    state["J"] = state_vector[21:].reshape((7, 6)).T
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

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    Id = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(Id - shouldBeIdentity)
    return n < 1e-6

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
    # H_no_hand = np.linalg.multi_dot([H1, H2, H3, H4, H5, H6, H7])
    return H[:, 3][:3]

def joint2posewrot(q):
    def RotX(q):
        return np.array([[1, 0, 0, 0], [0, np.cos(q), -np.sin(q), 0], [0, np.sin(q), np.cos(q), 0], [0, 0, 0, 1]])

    def RotZ(q):
        return np.array([[np.cos(q), -np.sin(q), 0, 0], [np.sin(q), np.cos(q), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    def TransX(q, x, y, z):
        return np.array([[1, 0, 0, x], [0, np.cos(q), -np.sin(q), y], [0, np.sin(q), np.cos(q), z], [0, 0, 0, 1]])

    def TransZ(q, x, y, z):
        return np.array([[np.cos(q), -np.sin(q), 0, x], [np.sin(q), np.cos(q), 0, y], [0, 0, 1, z], [0, 0, 0, 1]])
    H1 = TransZ(q[0], 0, 0, 0.333)
    H2 = np.dot(RotX(-np.pi / 2), RotZ(q[1]))
    H3 = np.dot(TransX(np.pi / 2, 0, -0.316, 0), RotZ(q[2]))
    H4 = np.dot(TransX(np.pi / 2, 0.0825, 0, 0), RotZ(q[3]))
    H5 = np.dot(TransX(-np.pi / 2, -0.0825, 0.384, 0), RotZ(q[4]))
    H6 = np.dot(RotX(np.pi / 2), RotZ(q[5]))
    H7 = np.dot(TransX(np.pi / 2, 0.088, 0, 0), RotZ(q[6]))
    H_panda_hand = TransZ(-np.pi / 4, 0, 0, 0.2105)
    H = np.linalg.multi_dot([H1, H2, H3, H4, H5, H6, H7, H_panda_hand])
    H_no_hand = np.linalg.multi_dot([H1, H2, H3, H4, H5, H6, H7])
    return np.concatenate((H[:, 3][:3], rotationMatrixToEulerAngles(H_no_hand[:3, :3])), axis=None)

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def dist(x, y):
    return np.linalg.norm(y - x)

def cost_to_go(s, a, g):
    return dist(s + a, g)

def boltzmann(q, Quest, s, g):
    beta_h = 10
    p = np.exp(-beta_h * cost_to_go(s, q, g))
    p1 = np.exp(-beta_h * cost_to_go(s, Quest[0], g))
    p2 = np.exp(-beta_h * cost_to_go(s, Quest[1], g))
    return p / (p1 + p2)

def info_gain(Quest, s, G, b):
    Qinfo = 0
    for q in Quest:
        Z = 0
        for x, g in enumerate(G):
            Z += b[x] * boltzmann(q, Quest, s, g)
        for x, g in enumerate(G):
            Phuman = boltzmann(q, Quest, s, g)
            Qinfo += b[x] * Phuman * np.log2(Phuman / Z)
    return Qinfo
