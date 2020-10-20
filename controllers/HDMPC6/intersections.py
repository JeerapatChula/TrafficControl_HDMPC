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
        self.np = controller_config["options"]["np"]

        self.state = self.get_state()
        self.mean_state = self.get_mean_state()
        self.matrix_c = self.get_matrix_c()
        self.matrix_b = self.get_matrix_b() if "estimated_b" not in intersection_configs else intersection_configs["estimated_b"]
        self.matrix_p = self.get_matrix_p()
        self.matrix_g = self.get_matrix_g()
        self.matrix_h = self.get_matrix_h()
        self.matrix_q = self.get_matrix_q()
        self.veh_passed = self.get_veh_passed()
        self.create_con()
        self.ref = np.zeros((self.num_incoming_roads + self.num_outcome_roads, 1))

        self.state_log = []

        self.mean_state_log = []
        self.mean_output_log = []
        self.ref_log = []
        self.predicted_flow_log = []
        self.control_signal_log = []
        self.cost_1_log = []
        self.cost_2_log = []

        self.state_predict = {}
        for i in range(self.np):
            self.state_predict[f"np{i}"] = []


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

    def get_free_flow(self):
        free_flow = None

        for road in self.model_incoming_roads:
            free_flow = road.free_flow if free_flow is None else np.vstack((free_flow, road.free_flow))

        for road in self.model_outcome_roads:
            free_flow = np.vstack((free_flow, road.free_flow))

        return free_flow

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
        self.predicted_flow = np.zeros((self.num_incoming_roads + self.num_outcome_roads, 1))
        for x in range(len(predicted_flow)):
            self.predicted_flow[x, 0] = predicted_flow[x] if x < self.num_incoming_roads else -predicted_flow[x]

    def cost_1(self, u):
        exogenous = np.matmul(self.matrix_p, self.predicted_flow*self.cycle_length)
        tmp = exogenous
        for i in range(1, self.np):
            tmp = np.vstack((tmp, exogenous*(i+1)))
        exogenous = tmp
        E = np.matmul(self.matrix_g, self.mean_state)
        E += np.matmul(self.matrix_h, u.reshape((-1, 1)))
        E += exogenous
        return E, (LA.norm(E, 2)**2)/2

    def cost_2(self, u):
        q = np.matmul(self.matrix_q, u.reshape((-1, 1)))
        E = q - self.ref
        return E, (LA.norm(E, 2)**2)/2

    def func(self, u):
        E_1, E_1_n = self.cost_1(u)
        E_2, E_2_n = self.cost_2(u)
        return E_1_n + self.gamma*E_2_n

    def func_dar(self, u):
        E1, E1_n = self.cost_1(u)
        E1 = np.matmul(E1.T, self.matrix_h)
        E2, E2_n = self.cost_2(u)
        E2 = np.matmul(E2.T, self.matrix_q)
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

        self.upper_bounds = upper
        self.lower_bounds = lower
        self.control_bounds = Bounds(lower, upper)

    def get_control_signal(self):

        self.state_log.append(self.state.tolist())
        self.mean_state_log.append(self.mean_state.tolist())
        self.mean_output_log.append(np.matmul(self.matrix_c, self.mean_state))
        self.ref_log.append(self.ref.tolist())
        self.predicted_flow_log.append(self.predicted_flow.tolist())

        eq_cons = {'type': 'eq',
                    'fun' : lambda x: np.squeeze(np.matmul(self.matrix_con_eq, x.reshape((-1, 1))) - self.vector_con_eq),
                    'jac' : lambda x: self.matrix_con_eq}

        initial = np.zeros((self.num_phase*self.np, 1))

        res = minimize(self.func, initial, method='SLSQP', jac=self.func_dar, constraints=[eq_cons], options={'disp': False}, bounds=self.control_bounds)
        control_signal = (res.x[0:self.num_phase]).astype(int)
        self.control_signal_log.append(control_signal)
        E1, E1_n = self.cost_1(res.x)
        E2, E2_n = self.cost_2(res.x)
        self.cost_1_log.append(E1_n)
        self.cost_2_log.append(E2_n)

        E1 = (E1.T.tolist())[0]
        num_state = int(len(E1)/self.np)
        for i in range(self.np):
            temp = E1[num_state*i: (num_state*(i+1))]
            self.state_predict[f"np{i}"].append(temp)
            # self.state_predict[i].append(temp)

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
            "cost_1_log": self.cost_1_log,
            "cost_2_log": self.cost_2_log,
            "cycle_length": self.cycle_length,
            "matrix_g": self.matrix_g,
            "matrix_h": self.matrix_h,
            "matrix_q": self.matrix_q,
            "matrix_p": self.matrix_p,
            "matrix_con_eq": self.matrix_con_eq,
            "vector_con_eq": self.vector_con_eq,
            "upper_bounds": self.upper_bounds,
            "state_predict": self.state_predict,
            }
        File.save_mat(filename, data)


class Intersection():

    def __init__(self, intermediary, name, controller_config, network_config):
        self.name = name
        self.intersection_configs = network_config.intersections[name]
        self.network_roads = Agreement.get_network_road_list(network_config)
        self.roads = Agreement.get_intersection_road_list(name, network_config)
        self.is_controlled = self.intersection_configs["controlled"]
        self.network_rate = network_config.rates["network"]
        self.roads_configs = network_config.roads

        self.ref_signal = None

        self.incoming_roads = list(self.intersection_configs["incoming_roads"].keys())
        self.outcome_roads = list(self.intersection_configs["outcome_roads"].keys())

        self.create_aggregation_matrix()
        self.create_distribution_matrix()

        self.socket = intermediary.client_register(name, "controller")
        self.socket.add_endpoint("upper_data_receiver", self.upper_data_receiver)
        self.socket.add_endpoint("phase_completed_signal", self.phase_completed_signal)
        self.socket.add_endpoint("cycle_completed_signal", self.cycle_completed_signal)

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
        self.old_predicted_flow = np.zeros_like(self.model.veh_passed)


    def update(self):
        pass

    def send_data_upper(self):
        data = self.model.get_total_demand()
        self.socket.post("network", "controller", "lower_data_receiver", {"signal": data, "intersection": self.name})

    def upper_data_receiver(self, data):
        self.ref_signal = np.matmul(self.distribution_matrix, data["signal"])
        self.model.set_ref_flow(self.ref_signal)

    def create_aggregation_matrix(self):
        matrix = np.zeros((1, len(self.roads)))
        for x in range(len(self.incoming_roads)):
            matrix[0, x] = 1
        self.aggregation_matrix = matrix

    def create_distribution_matrix(self):
        num_network_roads = len(self.network_roads)
        num_my_links = len(self.roads)
        matrix = np.zeros((num_my_links, num_network_roads))

        for (road, dst) in zip(self.roads, range(num_my_links)):
            road_conf = self.roads_configs[road]
            num_src = len(list(set([x["source"] for x in road_conf])))
            src = self.network_roads.index(road)
            for route in road_conf:
                if route["source"] == self.name:
                    matrix[dst, src] = 1 if num_src == 1 else route["ratio"]

                if route["sink"] == self.name:
                    matrix[dst, src] = route["ratio"] if num_src == 1 else 1
        self.distribution_matrix = (1/self.network_rate)*matrix

    def phase_completed_signal(self, data):

        duration = data["duration"]
        incoming_roads = data["incoming_roads"]
        outcome_roads = data["outcome_roads"]

        for road in incoming_roads:
            ind = self.incoming_roads.index(road)
            arrived = data["incoming_roads"][road]["arrived"]
            departed = data["incoming_roads"][road]["departed"]
            num_vehicles = data["incoming_roads"][road]["num_vehicles"]
            free_flow = data["incoming_roads"][road]["free_flow"]
            actual_state = data["incoming_roads"][road]["actual_state"]
            # if road== 'R35':
            #     input(departed)
            self.model_incoming_roads[ind].pre_update(actual_state, arrived, departed, num_vehicles, free_flow, duration) # Revise here

        for road in outcome_roads:
            ind = self.outcome_roads.index(road)
            arrived = data["outcome_roads"][road]["arrived"]
            num_vehicles = data["outcome_roads"][road]["num_vehicles"]
            free_flow = data["outcome_roads"][road]["free_flow"]
            self.model_outcome_roads[ind].pre_update(0, arrived, 0, num_vehicles, free_flow, duration) # Revise here

    def cycle_completed_signal(self, data):
        duration = data["duration"]
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
            free_flow = data["incoming_roads"][road]["free_flow"]
            actual_state = data["incoming_roads"][road]["actual_state"]
            self.model_incoming_roads[ind].pre_update(actual_state, arrived, departed, num_vehicles, free_flow, duration) # Revise here
            self.model_incoming_roads[ind].update_state()

        for road in outcome_roads:
            ind = self.outcome_roads.index(road)
            arrived = data["outcome_roads"][road]["arrived"]
            num_vehicles = data["outcome_roads"][road]["num_vehicles"]
            free_flow = data["outcome_roads"][road]["free_flow"]
            self.model_outcome_roads[ind].pre_update(0, arrived, 0, num_vehicles, free_flow, duration) # Revise here
            self.model_outcome_roads[ind].update_state()

        veh_passed = self.model.get_veh_passed()
        veh_entered = self.model.get_veh_entered()
        free_flow = self.model.get_free_flow()

        self.send_data_upper()
        (num_veh, num_mean_veh) = self.model.get_num_veh()
        predicted_flow = np.zeros_like(self.model.veh_passed)
        new_flow = veh_entered/cycle_length


        if self.is_controlled:
            if self.ref_signal is not None:
                self.intersection_flow_model.put_data(num_veh, num_mean_veh, self.ref_signal, self.old_signal, free_flow)
                predicted_flow = self.intersection_flow_model.predict()
            self.model.put_predicted_flow(predicted_flow)

            control_signal = self.model.get_control_signal()
            self.old_signal = control_signal
        else:
            self.old_signal = data["old_signal"]
            control_signal = data["old_signal"]
        return {"control_signal": control_signal}

    def save(self, output_path):
        self.intersection_flow_model.save(output_path)
        self.model.save(self.name, output_path)

    def model_save(self):
        filename = "models/np3/intersection/{}.mat".format(self.name)
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
