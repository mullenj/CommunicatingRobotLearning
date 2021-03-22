import socket
import time
import numpy as np
import pickle
import pygame
import sys


"""
 * a minimal script for recording kinesthetic demonstrations
 * Dylan Losey, September 2020

 * To run:
 [1] in one terminal:
    navigate to ~/panda-ws/essentials
    run python3 record_demo.py test.pkl
 [2] in a second terminal:
    navigate to ~/libfranka/build
    run ./collab/read_state
"""


def connect2robot(PORT):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('172.16.0.3', PORT))
    s.listen()
    conn, addr = s.accept()
    return conn

def listen2robot(conn):
    state_length = 7 + 7 + 7
    message = str(conn.recv(1024))[2:-2]
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
    return state

def readState(conn):
    while True:
        state = listen2robot(conn)
        if state is not None:
            break
    return state

class Joystick(object):

    def __init__(self):
        pygame.init()
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()

    def input(self):
        pygame.event.get()
        A_pressed = self.gamepad.get_button(0)
        B_pressed = self.gamepad.get_button(1)
        START_pressed = self.gamepad.get_button(7)
        return A_pressed, B_pressed, START_pressed


def main():

    filename = sys.argv[1]
    steptime = 0.1;
    PORT = 8080

    print('[*] Connecting to low-level controller...')
    conn = connect2robot(PORT)
    interface = Joystick()
    demonstration = []
    record = False

    print('[*] Ready for a demonstration...')

    while True:

        state = readState(conn)
        start, pause, stop = interface.input()

        if stop:
            pickle.dump( demonstration, open( filename, "wb" ) )
            print(demonstration)
            print("[*] Done!")
            print("[*] I recorded this many datapoints: ", len(demonstration))
            return True
        if start and not record:
            record = True
            start_time = time.time()
            print('[*] Recording the demonstration...')
        if pause and record:
            record = False
            print('[*] Pausing the demonstration...')

        curr_time = time.time()
        if record and curr_time - start_time >= steptime:
            demonstration.append(state["q"].tolist())
            start_time = curr_time


if __name__ == "__main__":
    main()
