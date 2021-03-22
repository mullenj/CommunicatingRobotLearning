import socket
import time
import numpy as np
import pickle
import pygame
import sys
from scipy.interpolate import interp1d

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

def main():

    demo_name = 'task2_path1'
    playback_time = 20.0
    proportional_gain = 5.0


    print('[*] Reading trajectory I need to follow...')
    xi = pickle.load( open( demo_name + ".pkl", "rb" ) )
    traj = Trajectory(xi, playback_time)

    trajectory = np.asarray([traj.get(time)for time in list(np.linspace(0, playback_time, int(playback_time * 10 + 1)))])
    test_point = np.asarray([0.136338, 0.26118, 0.157175, -1.80338, -0.00288155, 2.07467, 0.987893])
    #print(trajectory)

    print(min(np.linalg.norm(trajectory - test_point, axis = 1)))


if __name__ == "__main__":
    main()
