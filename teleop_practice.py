import teleop_utils as utils
import numpy as np
import time
"""
 * a minimal script for teleoperating the robot using a joystick
 * Dylan Losey, September 2020
 * James Mullen, April 2021

 * To run:
 [1] in one terminal:
    navigate to ~/panda-ws/essentials
    run python3 teleop.py
 [2] in a second terminal:
    navigate to ~/libfranka/build
    run ./collab/velocity_control
"""

def main():

    PORT_robot = 8080
    action_scale = 0.05
    interface = utils.Joystick()

    print('[*] Connecting to low-level controller...')

    conn = utils.connect2robot(PORT_robot) # connect to other computer

    # readState -> give you a dictionary with things like joint values, velocity, torque
    state = utils.readState(conn)

    translation_mode = True
    start_time = time.time()

    print('[*] Ready for a teleoperation...')
    while True:

        # read the current state of the robot + the xyz position
        state = utils.readState(conn)

        # get the humans joystick input
        z, mode, grasp, stop = interface.input()
        if mode and (time.time() - start_time > 1):
            translation_mode = not translation_mode
            start_time = time.time()
        if stop:
            print("[*] Done!")
            return True

        a_h = [0]*3
        a_h[0] = z[1]
        a_h[1] = z[0]
        a_h[2] = -z[2]
        a_h = np.asarray(a_h)
        a_h = np.pad(a_h, (0, 3), 'constant')
        if not translation_mode:
            a_h = np.pad(np.asarray(z), (3, 0), 'constant')
            a_h[3] = 0
            a_h[5] = 0

        a_h = action_scale * a_h

        # convert this command to joint space
        qdot = utils.xdot2qdot(a_h, state)
        # send our final command to robot
        utils.send2robot(conn, qdot)


if __name__ == "__main__":
    main()
