from scipy.optimize import minimize
from scipy.optimize import Bounds
from scipy.linalg import block_diag
from numpy import linalg as LA
from libs.etc.files import File
import numpy as np
import warnings
import traci

warnings.filterwarnings("ignore", message="numpy.dtype size changed")
warnings.filterwarnings("ignore", message="numpy.ufunc size changed")


class Model():
    def __init__(self, **kwargs):
        self.mat_a = None
        self.mat_a_eq = np.zeros((0, 0))

        roads = kwargs["roads"]
        incoming_roads = kwargs["incoming_roads"]
        outcome_roads = kwargs["outcome_roads"]
        available_lane = kwargs["available_lane"]

        for (incoming_road, incoming_road_ind) in zip(incoming_roads, range(len(incoming_roads))):
            matrix_a_tmp = np.zeros((len(outcome_roads), len(roads[incoming_road]["dst"])))
            for (dst, dir_ind) in zip(roads[incoming_road]["dst"], range(len(roads[incoming_road]["dst"]))):
                ind = outcome_roads.index(dst)
                matrix_a_tmp[ind, dir_ind] = 1 if (incoming_road, dst) in available_lane else 0
            self.mat_a = matrix_a_tmp if self.mat_a is None else np.hstack((self.mat_a, matrix_a_tmp))

        mat_a_eq_2 = None
        for incoming_road in incoming_roads:
            tmp = np.zeros((1, len(roads[incoming_road]["dst"])))
            mat_a_eq_2_tmp = None
            for (dst, ind) in zip(roads[incoming_road]["dst"], range(len(roads[incoming_road]["dst"]))):
                tmp[0, ind] = 1 if (incoming_road, dst) in available_lane else 0

                tmp2 = np.zeros((1, len(roads[incoming_road]["dst"])))
                if (incoming_road, dst) not in available_lane:
                    tmp2[0, ind] = 1
                    mat_a_eq_2_tmp = tmp2 if mat_a_eq_2_tmp is None else np.vstack((mat_a_eq_2_tmp, tmp2))
                    # self.vec_b_eq = np.vstack((self.vec_b_eq, 0))

            if mat_a_eq_2_tmp is not None:
                mat_a_eq_2 = mat_a_eq_2_tmp if mat_a_eq_2 is None else block_diag(mat_a_eq_2, mat_a_eq_2_tmp)
            else:
                size_h = 0 if mat_a_eq_2 is None else mat_a_eq_2.shape[0]
                mat_a_eq_2_tmp = np.zeros((size_h, len(roads[incoming_road]["dst"])))
                mat_a_eq_2 = mat_a_eq_2_tmp if mat_a_eq_2 is None else np.hstack((mat_a_eq_2, mat_a_eq_2_tmp))
            self.mat_a_eq = block_diag(self.mat_a_eq, tmp)


class SplittingFlow():

    def __init__(self, intersection, intersection_configs):
        self.name = intersection
        self.incoming_roads = list(intersection_configs['incoming_roads'].keys())
        self.outcome_roads = list(intersection_configs['outcome_roads'].keys())
        self.num_phase = len(intersection_configs["available_lanes"])
        self.p_index = []
        self.total_phase_time = [0]*self.num_phase
        self.available_lanes = intersection_configs["available_lanes"]

        self.model = [None]*self.num_phase
        self.src_phases = []
        for phase in range(self.num_phase):
            tmp = [x[0] for x in intersection_configs["available_lanes"][phase]]
            self.src_phases.append(set(tmp))

        self.roads = {}
        self.roads_flow_pahse = {}
        self.flow_log = {}
        self.time_log = []

        for road in self.incoming_roads:
            self.roads_flow_pahse[road] = []
            self.flow_log[road] = []
            for i in range(self.num_phase):
                self.roads_flow_pahse[road].append([0]*len(intersection_configs['incoming_roads'][road]["dst_roads"]))

            self.roads[road] = {}
            self.roads[road]["dst"] = intersection_configs['incoming_roads'][road]["dst_roads"]
            self.roads[road]["total_departed"] = [0]*len(self.roads[road]["dst"])
            self.roads[road]["actual_total_departed"] = [0]*len(self.roads[road]["dst"])

            for i in range(len(self.roads[road]["dst"])):
                self.p_index.append(road)
        self.last_departed = {}

        for road in self.outcome_roads:
            self.roads_flow_pahse[road] = []
            for i in range(self.num_phase):
                self.roads_flow_pahse[road].append([0]*1)

        for (road, i) in zip(self.incoming_roads, range(len(self.incoming_roads))):
            self.last_departed[road] = [0]*len(self.roads[road]["dst"])

        self.model = []
        for self.available_lane in self.available_lanes:
            self.model.append(Model(roads=self.roads, incoming_roads=self.incoming_roads, outcome_roads=self.outcome_roads, available_lane=self.available_lane))


    def completed_phase(self, actaul_departed, departed, arrived, last_phase, duration):

        self.total_phase_time[last_phase] += duration
        self.vec_w_in = np.array(arrived).reshape((-1, 1))
        self.mat_a = self.model[last_phase].mat_a
        self.mat_a_eq = self.model[last_phase].mat_a_eq
        self.vec_b_eq = np.array(departed).reshape((-1, 1))

        for road, x in zip(self.outcome_roads, arrived):
            self.roads_flow_pahse[road][last_phase][0] += x

        ind_zero = np.where(np.sum(self.mat_a_eq, 1) == 0)
        self.mat_a_eq = np.delete(self.mat_a_eq, ind_zero, 0)
        self.vec_b_eq = np.delete(self.vec_b_eq, ind_zero, 0)

        flow = self.get_flow()
        self.last_departed = {}
        self.time_log.append(int(traci.simulation.getTime()))

        for (road, i) in zip(self.incoming_roads, range(len(self.incoming_roads))):
            ind = np.isin(self.p_index, road)
            flow_road = flow[ind].tolist()
            self.last_departed[road] = [0]*len(self.roads[road]["dst"])
            self.flow_log[road].append(flow_road)

            for x in range(len(self.roads[road]["dst"])):
                self.roads[road]["total_departed"][x] += flow_road[x]
                self.roads[road]["actual_total_departed"][x] += actaul_departed[i][x]
                self.last_departed[road][x] = flow_road[x]
                self.roads_flow_pahse[road][last_phase][x] += flow_road[x]

    def func(self, x):
        E = np.matmul(self.mat_a, x.reshape((-1, 1))) - self.vec_w_in
        E = (LA.norm(E, 2)**2)/2
        return E

    def func_dar(self, x):
        E_dar = np.matmul(self.mat_a, x.reshape((-1, 1))) - self.vec_w_in
        E_dar = np.matmul(E_dar.T, self.mat_a)
        return np.squeeze(np.asarray(E_dar))

    def get_flow(self):

        initial = np.zeros(len(self.p_index))

        eq_cons = {'type': 'eq',
                    'fun' : lambda x: np.squeeze(np.matmul(self.mat_a_eq, x.reshape((-1, 1))) - self.vec_b_eq),
                    'jac' : lambda x: self.mat_a_eq}

        LowerBound = np.array([0] * len(self.p_index))
        UpperBound = np.array([np.inf] * len(self.p_index))
        bounds = Bounds(LowerBound, UpperBound)

        res = minimize(self.func, initial, method='SLSQP', constraints=[eq_cons], options={'disp': False}, bounds=bounds)
        return np.rint(res.x)

    def get_summarize(self):
        tmp = {}
        for road in self.incoming_roads:
            data = "actual_total_departed"
            # data = "total_departed"
            total = float(sum(self.roads[road][data]))
            total = 1.0 if total == 0 else total
            tmp[road] = [float(x)/total for x in self.roads[road][data]]
        return tmp

    def get_matrix_b(self):
        matrix_b = None

        for road in self.incoming_roads:
            num_directions = len(self.roads_flow_pahse[road][0])
            tmp = np.zeros((num_directions, self.num_phase))
            for p in range(self.num_phase):
                tmp[:, p] = -np.array(self.roads_flow_pahse[road][p]).T / self.total_phase_time[p] if self.total_phase_time[p] > 0 else np.zeros_like(tmp[:, p])
            matrix_b = tmp if matrix_b is None else np.vstack((matrix_b, tmp))

        for road in self.outcome_roads:
            tmp = np.zeros((1, self.num_phase))
            for p in range(self.num_phase):
                tmp[0, p] = self.roads_flow_pahse[road][p][0]/ self.total_phase_time[p] if self.total_phase_time[p] > 0 else 0
            matrix_b = np.vstack((matrix_b, tmp))

        return matrix_b

    def save(self, output_path):
        filename = "{}/modules/splitting_flow/{}.mat".format(output_path, self.name)
        File.save_mat(filename, {'flow_log': self.flow_log, 'time_log': self.time_log})
