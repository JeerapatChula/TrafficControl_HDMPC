from scipy.optimize import minimize
from scipy.optimize import Bounds
from numpy import linalg as LA
import numpy as np
import warnings

warnings.filterwarnings("ignore", message="numpy.dtype size changed")
warnings.filterwarnings("ignore", message="numpy.ufunc size changed")


class Road():

    def __init__(self, name, road_configs, is_incoming, available_lanes, incoming_roads):
        self.name = name
        self.is_incoming = is_incoming
        self.road_configs = road_configs
        self.available_lanes = available_lanes
        self.incoming_roads = incoming_roads
        self.num_directions = len(road_configs["dst_roads"]) if is_incoming else 1
        self.x_hat = np.zeros((self.num_directions, 1))
        self.x_hat_mean = np.zeros_like(self.x_hat)
        self.x_hat_mean_tmp = np.zeros_like(self.x_hat)
        self.num_means = 0

        self.arrived = np.ones((self.num_directions, 1))
        self.departed = np.ones((self.num_directions, 1))
        self.num_phases = len(available_lanes)

        self.veh_passed = 0
        self.veh_entered = 0

        self.matrix_c = np.ones((1, self.num_directions))
        self.create_matrix_b()
        self.create_matrix_p()
        # self.create_matrix_k()

    def create_matrix_b(self):

        self.matrix_b = np.zeros((self.num_directions, self.num_phases))

        for (items, ind) in zip(self.available_lanes, range(len(self.available_lanes))):
            for (src, dst) in items:
                if self.is_incoming and src == self.name:
                    dir = self.road_configs["dst_roads"].index(dst)
                    prob = self.incoming_roads[src]["splitting_flows"][dir]
                    self.matrix_b[dir, ind] = -self.incoming_roads[src]["weighting"][dir]*self.incoming_roads[src]["flow_rate"]
                    # self.matrix_b[dir, ind] = -MaxFlowRate
                elif dst == self.name:
                    # if dst == "R32":
                    #     input((self.incoming_roads[src]["dst_roads"], src))
                    dir = self.incoming_roads[src]["dst_roads"].index(dst)
                    prob = self.incoming_roads[src]["splitting_flows"][dir]
                    self.matrix_b[0, ind] += self.incoming_roads[src]["weighting"][dir]*self.incoming_roads[src]["flow_rate"]
                    # self.matrix_b[0, ind] += MaxFlowRate

    def create_matrix_k(self):
        self.matrix_k = np.eye(self.num_directions)/(self.road_configs["num_lane_in"]*self.road_configs["length"])

    def create_matrix_p(self):
        self.matrix_p = np.ones((self.num_directions, 1))
        if self.is_incoming:
            self.flow_rate = self.road_configs["flow_rate"]
            for (p, x) in zip(self.road_configs["splitting_flows"], range(self.num_directions)):
                self.matrix_p[x] = p

    def pre_update(self, arrived, departed, desired_y):
        departed = np.array(departed)
        self.arrived += self.matrix_p*arrived
        self.departed += departed.T
        self.veh_passed += departed if self.is_incoming else arrived
        self.veh_entered += arrived
        self.x_hat = self.get_estimated_state(desired_y) if self.is_incoming else desired_y
        self.x_hat_mean_tmp += self.x_hat
        self.num_means += 1

    def func(self, x):
        E = x.reshape((-1, 1)) - self.x_hat + self.arrived - self.departed
        return (LA.norm(E, 2)**2)/2

    def func_dar(self, x):
        return x

    def get_estimated_state(self, desired_y):
        initial = np.zeros((self.num_directions, 1))
        bounds = Bounds([0]*self.num_directions, [np.inf]*self.num_directions)

        eq_cons = {'type': 'eq',
                    'fun' : lambda x: np.squeeze(np.matmul(self.matrix_c, x.reshape((-1, 1))) - desired_y),
                    'jac' : lambda x: self.matrix_c}

        res = minimize(self.func, initial, method='SLSQP', jac=self.func_dar, constraints=[eq_cons], options={'disp': False}, bounds=bounds)
        return res.x.reshape((-1, 1))

    def update_state(self):
        self.x_hat_mean = self.x_hat_mean_tmp/self.num_means
        self.x_hat_mean_tmp = np.zeros_like(self.x_hat)
        self.num_means = 0
        self.arrived = np.zeros_like(self.arrived)
        self.departed = np.zeros_like(self.departed)
