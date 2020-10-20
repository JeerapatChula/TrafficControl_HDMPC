from .agreements import Agreement
from .roads import Road
from ai.intersection_flow_model import IntersectionFlowModel
from scipy.optimize import minimize
from scipy.optimize import Bounds
from scipy.linalg import block_diag
from numpy import linalg as LA
from libs.etc.files import File
import numpy as np
import warnings

warnings.filterwarnings("ignore", message="numpy.dtype size changed")
warnings.filterwarnings("ignore", message="numpy.ufunc size changed")


class Model():

    def __init__(self, intersection, intersection_configs, model_incoming_roads, model_outcome_roads, controller_config):
        self.name = intersection
        self.incoming_roads = list(intersection_configs["incoming_roads"].keys())
        self.outcome_roads = list(intersection_configs["outcome_roads"].keys())
        self.cycle_length = intersection_configs["cycle_length"]
        self.default_signals = intersection_configs["default_signals"]
        self.fixed_phase = intersection_configs["fixed_phase"]
        self.maximum_signals = intersection_configs["maximum_signals"]
        self.cycle_length = intersection_configs["cycle_length"]
        self.num_phase = len(intersection_configs["default_signals"])

        self.num_incoming_roads = len(self.incoming_roads)
        self.num_outcome_roads = len(self.outcome_roads)

        self.model_incoming_roads = model_incoming_roads
        self.model_outcome_roads = model_outcome_roads

        self.gamma = controller_config["gamma"]
        # self.gamma = intersection_configs["gamma"]
        self.np = controller_config["options"]["np"]

        self.state = self.get_state()
        self.mean_state = self.get_mean_state()
        self.matrix_c = self.get_matrix_c()
        # self.matrix_k = self.get_matrix_k()
        # self.matrix_kk = self.get_matrix_kk()
        self.matrix_b = self.get_matrix_b()
        self.matrix_p = self.get_matrix_p()
        self.matrix_g = self.get_matrix_g()
        self.matrix_h = self.get_matrix_h()
        self.matrix_q = self.get_matrix_q()
        self.veh_passed = self.get_veh_passed()
        self.create_con()
        self.ref = np.zeros((self.num_incoming_roads + self.num_outcome_roads, 1))

        # self.KG = np.matmul(self.matrix_kk, self.matrix_g)
        # self.KH = np.matmul(self.matrix_kk, self.matrix_h)

        self.state_log = []
        self.mean_state_log = []
        self.mean_output_log = []
        self.ref_log = []
        self.predicted_flow_log = []
        self.control_signal_log = []


        # input([self.incoming_roads, self.outcome_roads])


    def get_state(self):
        state = None

        for road in self.model_incoming_roads:
            state = road.x_hat if state is None else np.vstack((state, road.x_hat))

        for road in self.model_outcome_roads:
            state = np.vstack((state, road.x_hat))

        return state

    def get_total_demand(self):
        total_demand = 0

        for road in self.model_incoming_roads:
            total_demand += sum(road.x_hat_mean)

        return total_demand

    def get_mean_state(self):
        mean_state = None

        for road in self.model_incoming_roads:
            mean_state = road.x_hat_mean if mean_state is None else np.vstack((mean_state, road.x_hat_mean))

        for road in self.model_outcome_roads:
            mean_state = np.vstack((mean_state, road.x_hat_mean))

        return mean_state

    def get_matrix_c(self):
        matrix_c = None

        for road in self.model_incoming_roads:
            matrix_c = road.matrix_c if matrix_c is None else block_diag(matrix_c, road.matrix_c)

        for road in self.model_outcome_roads:
            matrix_c = block_diag(matrix_c, road.matrix_c)

        return matrix_c

    def get_matrix_k(self):
        matrix_k = None

        for road in self.model_incoming_roads:
            matrix_k = road.matrix_k if matrix_k is None else block_diag(matrix_k, road.matrix_k)

        for road in self.model_outcome_roads:
            matrix_k = block_diag(matrix_k, road.matrix_k)

        return matrix_k

    def get_matrix_b(self):
        matrix_b = None

        for road in self.model_incoming_roads:
            matrix_b = road.matrix_b if matrix_b is None else np.vstack((matrix_b, road.matrix_b))

        for road in self.model_outcome_roads:
            matrix_b = np.vstack((matrix_b, road.matrix_b))

        return matrix_b

    def get_matrix_p(self):
        matrix_p = None

        for road in self.model_incoming_roads:
            matrix_p = road.matrix_p if matrix_p is None else block_diag(matrix_p, road.matrix_p)

        for road in self.model_outcome_roads:
            matrix_p = block_diag(matrix_p, road.matrix_p)

        return matrix_p

    def get_matrix_g(self):
        return np.kron(np.ones((self.np, 1)), np.eye(len(self.state)))

    def get_matrix_kk(self):
        return np.kron(np.eye(self.np), self.matrix_k)

    def get_matrix_h(self):
        tmp = self.matrix_b
        size_tmp = tmp.shape

        size_h = (size_tmp[0]*self.np, size_tmp[1]*self.np)
        matrix = np.zeros(size_h)

        for i in range(0, self.np):
            vid = np.arange(i*size_tmp[0], (i + 1)*size_tmp[0])
            for y in range(0, i + 1):
                hid = np.arange(y*size_tmp[1], (y + 1)*size_tmp[1])
                for h in range(len(hid)):
                    matrix[vid, hid[h]] = np.squeeze(tmp[:, h])
        return matrix

    def get_matrix_q(self):
        # This function is used to builds the matrix Q for MPC in eq(58).
        CB = np.matmul(self.matrix_c, np.absolute(self.matrix_b))
        return np.kron(np.ones((1, self.np)), CB)/(self.cycle_length*self.np)

    def get_veh_passed(self):
        veh_passed = None

        for road in self.model_incoming_roads:
            veh_passed = road.veh_passed if veh_passed is None else np.vstack((veh_passed, road.veh_passed))
            road.veh_passed = 0

        for road in self.model_outcome_roads:
            veh_passed = np.vstack((veh_passed, road.veh_passed))
            road.veh_passed = 0

        return veh_passed

    def get_veh_entered(self):
        veh_entered = None

        for road in self.model_incoming_roads:
            veh_entered = road.veh_entered if veh_entered is None else np.vstack((veh_entered, road.veh_entered))
            road.veh_entered = 0

        for road in self.model_outcome_roads:
            veh_entered = np.vstack((veh_entered, road.veh_entered))
            road.veh_entered = 0

        return veh_entered

    def get_num_veh(self):
        self.state = self.get_state()
        self.mean_state = self.get_mean_state()
        num_veh = np.matmul(self.matrix_c, self.state)
        num_mean_veh = np.matmul(self.matrix_c, self.mean_state)
        return (num_veh, num_mean_veh)

    def set_ref_flow(self, ref):
        for (x, ind) in zip(ref, range(self.num_incoming_roads + self.num_outcome_roads)):
            self.ref[ind] = x

    def put_predicted_flow(self, predicted_flow):
        predicted_flow = abs(predicted_flow)
        self.predicted_flow = np.zeros((self.num_incoming_roads + self.num_outcome_roads, 1))
        for x in range(len(predicted_flow)):
            self.predicted_flow[x] = predicted_flow[x] if x < self.num_incoming_roads else -predicted_flow[x]


    def func(self, u):
        q = np.matmul(self.matrix_q, u.reshape((-1, 1)))
        exogenous = np.matmul(self.matrix_p, self.predicted_flow*self.cycle_length)
        tmp = exogenous
        for i in range(1, self.np):
            tmp = np.vstack((tmp, exogenous*(i+1)))
        exogenous = tmp

        E1 = np.matmul(self.matrix_g, self.mean_state) + np.matmul(self.matrix_h, u.reshape((-1, 1))) + exogenous
        E2 = q - self.ref
        E1 = (LA.norm(E1, 2)**2)/2
        E2 = (LA.norm(E2, 2)**2)/2
        return E1 + self.gamma*E2

    def func_dar(self, u):
        q = np.matmul(self.matrix_q, u.reshape((-1, 1)))
        exogenous = np.matmul(self.matrix_p, self.predicted_flow*self.cycle_length)
        tmp = exogenous
        for i in range(1, self.np):
            tmp = np.vstack((tmp, exogenous*(i+1)))
        exogenous = tmp

        E1 = np.matmul(self.matrix_g, self.mean_state) + np.matmul(self.matrix_h, u.reshape((-1, 1))) + exogenous
        E1 = np.matmul(E1.T, self.matrix_h)
        E2 = q - self.ref
        E2 = np.matmul(E2.T, self.matrix_q,)
        return np.squeeze(np.asarray(E1 + self.gamma*E2))

    def create_con(self):
        self.matrix_con_eq = np.kron(np.eye(self.np), np.ones((1, self.num_phase)))
        self.vector_con_eq = np.ones((self.np, 1)) * self.cycle_length

        lower = [15]*self.num_phase
        upper = [np.inf]*self.num_phase
        upper = self.maximum_signals
        for x in self.fixed_phase:
            lower[x] = self.default_signals[x]
            upper[x] = self.default_signals[x]

        for i in range(self.np-1):
            for j in range(self.num_phase):
                lower.append(lower[j])
                upper.append(upper[j])

        self.control_bounds = Bounds(lower, upper)

    def get_control_signal(self):

        self.state_log.append(self.state.tolist())
        self.mean_state_log.append(self.mean_state.tolist())
        self.mean_output_log.append(np.matmul(self.matrix_c, self.mean_state))
        self.ref_log.append(self.ref.tolist())
        self.predicted_flow_log.append(self.predicted_flow.tolist())


        eq_cons = {'type': 'eq',
                    'fun' : lambda x: np.squeeze(np.matmul(self.matrix_con_eq, x.reshape((-1, 1))) - self.vector_con_eq),
                    'jac' : lambda x:self.matrix_con_eq}

        initial = np.zeros((self.num_phase*self.np, 1))

        res = minimize(self.func, initial, method='SLSQP', jac=self.func_dar, constraints=[eq_cons], options={'disp': False}, bounds=self.control_bounds)
        control_signal = (res.x[0:self.num_phase]).astype(int)
        self.control_signal_log.append(control_signal)
        return control_signal

    def save(self, name, output_path):
        filename = "{}/controllers/intersections/{}.mat".format(output_path, name)
        data = {
            "state_log": self.state_log,
            "mean_state_log": self.mean_state_log,
            "mean_output_log": self.mean_output_log,
            "ref_log": self.ref_log,
            "predicted_flow_log": self.predicted_flow_log,
            "control_signal_log": self.control_signal_log,
            }
        # File.save_mat(filename, data)


class Intersection():

    def __init__(self, intermediary, name, controller_config, network_config):
        self.name = name
        self.intersection_configs = network_config.intersections[name]
        self.my_area = Agreement.get_intersection_area(name, network_config)
        self.area_links = Agreement.get_area_link_list(self.my_area, network_config)
        self.links = Agreement.get_intersection_link_list(name, network_config)
        self.is_controlled = self.intersection_configs["controlled"]

        self.processed_signal = None

        self.incoming_roads = list(self.intersection_configs["incoming_roads"].keys())
        self.outcome_roads = list(self.intersection_configs["outcome_roads"].keys())

        self.socket = intermediary.client_register(name, "controller")
        self.socket.add_endpoint("upper_data_receiver", self.upper_data_receiver)
        self.socket.add_endpoint("phase_completed_signal", self.phase_completed_signal)
        self.socket.add_endpoint("cycle_completed_signal", self.cycle_completed_signal)
        self.socket.add_endpoint("get_total_demand", self.get_total_demand)

        data = {"cluster": "controller",
        "name": name,
        "phase_enpoint": "phase_completed_signal",
        "cycle_enpoint": "cycle_completed_signal",
        "controlled": self.is_controlled}

        self.socket.post(name, "module_intersection", "set_controlled", data)

        available_lanes = self.intersection_configs["available_lanes"]
        incoming_roads = self.intersection_configs["incoming_roads"]
        self.model_incoming_roads = [Road(road, self.intersection_configs["incoming_roads"][road], True, available_lanes, incoming_roads) for road in self.incoming_roads]
        self.model_outcome_roads = [Road(road, self.intersection_configs["outcome_roads"][road], False, available_lanes, incoming_roads) for road in self.outcome_roads]

        self.model = Model(name, self.intersection_configs, self.model_incoming_roads, self.model_outcome_roads, controller_config)
        self.old_signal = network_config.intersections[name]["default_signals"]
        self.intersection_flow_model = IntersectionFlowModel(self.name, self.intersection_configs)
        self.create_distribution_matrix()
        self.old_predicted_flow = np.zeros_like(self.model.veh_passed)

        filename = "models/np3/intersection/{}.mat".format(name)
        data = {
            "matrix_c": self.model.matrix_c,
            "matrix_b": self.model.matrix_b,
            "matrix_p": self.model.matrix_p,
            "matrix_g": self.model.matrix_g,
            "matrix_h": self.model.matrix_h,
            "matrix_q": self.model.matrix_q,
            "matrix_con_eq": self.model.matrix_con_eq,
            "vector_con_eq": self.model.vector_con_eq,
            "distribution_matrix": self.distribution_matrix,
            }
        File.save_mat(filename, data)

    def update(self):
        pass

    def save(self, output_path):
        self.intersection_flow_model.save(output_path)
        self.model.save(self.name, output_path)

    def send_data_upper(self, data):
        self.socket.post(self.my_area, "controller", "lower_data_receiver", {"signal": data, "intersection": self.name})

    def upper_data_receiver(self, data):
        self.raw_signal = data["raw_signal"]
        self.processed_signal = data["processed_signal"]
        self.raw_signal = np.matmul(self.distribution_matrix, self.raw_signal)
        self.processed_signal = np.matmul(self.distribution_matrix, self.processed_signal)
        # self.processed_signal = (float(self.intersection_configs["max_flow"])/3600.0)*(self.raw_signal/sum(self.raw_signal))
        self.model.set_ref_flow(self.processed_signal)

    def create_distribution_matrix(self):
        num_area_links = len(self.area_links)
        num_my_links = len(self.links)
        matrix = np.zeros((num_my_links, num_area_links))
        for (road, dst) in zip(self.links, range(num_my_links)):
            src = self.area_links.index(road)
            matrix[dst, src] = 1
        self.distribution_matrix = matrix

    def get_total_demand(self, data):
        return {"total_demand": self.model.get_total_demand()}

    def phase_completed_signal(self, data):

        incoming_roads = data["incoming_roads"]
        outcome_roads = data["outcome_roads"]

        for road in incoming_roads:
            ind = self.incoming_roads.index(road)
            arrived = data["incoming_roads"][road]["arrived"]
            departed = data["incoming_roads"][road]["departed"]
            num_vehicles = data["incoming_roads"][road]["num_vehicles"]
            self.model_incoming_roads[ind].pre_update(arrived, departed, num_vehicles)

        for road in outcome_roads:
            ind = self.outcome_roads.index(road)
            arrived = data["outcome_roads"][road]["arrived"]
            num_vehicles = data["outcome_roads"][road]["num_vehicles"]
            self.model_outcome_roads[ind].pre_update(arrived, 0, num_vehicles)

    def cycle_completed_signal(self, data):
        cycle_length = data["cycle_length"]
        incoming_roads = data["incoming_roads"]
        outcome_roads = data["outcome_roads"]
        outcome_roads = data["outcome_roads"]

        if not self.is_controlled:
            self.old_signal = data["old_signal"]

        for road in incoming_roads:
            ind = self.incoming_roads.index(road)
            arrived = data["incoming_roads"][road]["arrived"]
            departed = data["incoming_roads"][road]["departed"]
            num_vehicles = data["incoming_roads"][road]["num_vehicles"]
            self.model_incoming_roads[ind].pre_update(arrived, departed, num_vehicles)
            self.model_incoming_roads[ind].update_state()

        for road in outcome_roads:
            ind = self.outcome_roads.index(road)
            arrived = data["outcome_roads"][road]["arrived"]
            num_vehicles = data["outcome_roads"][road]["num_vehicles"]
            self.model_outcome_roads[ind].pre_update(arrived, 0, num_vehicles)
            self.model_outcome_roads[ind].update_state()

        veh_passed = self.model.get_veh_passed()
        veh_entered = self.model.get_veh_entered()

        self.send_data_upper(veh_passed)
        (num_veh, num_mean_veh) = self.model.get_num_veh()
        predicted_flow = np.zeros_like(self.model.veh_passed)
        new_flow = veh_entered/cycle_length
        if self.processed_signal is not None:
            self.intersection_flow_model.put_data(num_veh, num_mean_veh, self.raw_signal, self.processed_signal, self.old_signal, new_flow)
            predicted_flow = self.intersection_flow_model.predict()
        self.model.put_predicted_flow(predicted_flow)

        if self.is_controlled:
            control_signal = self.model.get_control_signal()
            self.old_signal = control_signal
        else:
            self.old_signal = data["old_signal"]
            control_signal = data["old_signal"]
        return {"control_signal": control_signal}
