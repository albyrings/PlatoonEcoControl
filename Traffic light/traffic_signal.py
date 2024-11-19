import numpy as np
from sympy import *
import networkx as nx
import copy
from networkx.algorithms.shortest_paths.weighted import dijkstra_path
import scipy

class TrafficSignal:
    def __init__(self, n_intersections=5, T_cycle=30, tf=200, teta=None, offset=3, v_min=5, v_max=14, distance=None):
        self.n_intersections = n_intersections
        self.T_cycle = T_cycle
        self.T_green = 10#T_cycle / 2.77
        self.tf = tf
        self.teta = teta if teta is not None else [13, 3, 28, 15, 5]
        self.offset = offset
        self.v_min = v_min
        self.v_max = v_max
        self.distance = distance if distance is not None else [0, 300, 600, 900, 1200, 1550, 2000]



        # Generate time vector
        self.time = np.arange(self.offset, self.tf + self.offset, self.offset)

        # Initialize the signal matrix
        self.s = np.zeros((self.n_intersections, self.time.shape[0]))

        # Generate the signal matrix
        self._generate_signal_matrix()






    def _generate_signal_matrix(self):
        for i in range(self.n_intersections):
            if self.teta[i] < (self.T_cycle - self.T_green):
                for z in range(self._n_cycles() + 1):
                    for t in self.time:
                        if ((t - self.teta[i]) > (z * self.T_cycle) and (t - self.teta[i]) <= (z * self.T_cycle + self.T_green)):
                            tt = np.where(np.abs(self.time - t) < self.offset * 0.1)[0]
                            self.s[i, tt] = 1
            else:
                for z in range(self._n_cycles() + 1):
                    for t in self.time:
                        if t <= (self.teta[i] - (self.T_cycle - self.T_green)) and z == 0:
                            tt = np.where(np.abs(self.time - t) < self.offset * 0.1)[0]
                            self.s[i, tt] = 1
                        elif ((t - self.teta[i]) > (z * self.T_cycle) and (t - self.teta[i]) <= (z * self.T_cycle + self.T_green)):
                            tt = np.where(np.abs(self.time - t) < self.offset * 0.1)[0]
                            self.s[i, tt] = 1

        self._finalize_signal_matrix()

    def _n_cycles(self):
        return int(np.ceil(self.tf / self.T_cycle))

    def _finalize_signal_matrix(self):
        ss = np.zeros(self.time.shape[0])
        ss[0] = 1
        sss = np.zeros((7, self.time.shape[0]))
        sss[0] = ss
        for k in range(self.s.shape[0]):
            sss[k + 1] = self.s[k]

        ss2 = np.zeros(self.time.shape[0])
        ss2[-1] = 1
        sss[6] = ss2
        self.s = sss

    def get_signal_matrix(self):
        return self.s

    def get_parameters(self):
        return self.n_intersections, self.T_cycle, self.T_green, self.tf, self.teta, self.offset, self.v_min, self.v_max, self.distance, self.time



