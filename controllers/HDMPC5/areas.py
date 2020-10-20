from .agreements import Agreement
from scipy.optimize import minimize
from scipy.optimize import Bounds
from numpy import linalg as LA
from libs.etc.files import File
import numpy as np
import warnings

warnings.filterwarnings("ignore", message="numpy.dtype size changed")
warnings.filterwarnings("ignore", message="numpy.ufunc size changed")


class Model():

    def __init__(self, area, network_config, socket):
        self.areas_configs = network_config.areas[area]
        self.roads_configs = network_config.roads
        self.boundaries_conf = network_config.boundaries
        self.intersection_list = self.areas_configs["intersections"]
        self.network_rate = network_config.rates["network"]
        self.area_rate = network_config.rates["area"]
        self.socket = socket
        self.acceptable_flow_error = (self.area_rate/3600)*300

        self.boundary_list = Agreement.get_area_boundary_list(area, self.boundaries_conf)
        self.intersection_list = Agreement.get_area_intersection_list(area, network_config)
        self.network_links = Agreement.get_network_link_list(network_config)
        self.links = Agreement.get_area_link_list(area, network_config)
        self.roads = Agreement.get_area_road_list(area, network_config)
        self.nodes = Agreement.get_area_node_list(area, network_config)

        self.num_boundaries = len(self.boundary_list)
        self.num_intersections = len(self.intersection_list)
        self.num_network_links = len(self.network_links)
        self.num_links = len(self.links)
        self.num_roads = len(self.roads)
        self.num_nodes = len(self.nodes)

        self.create_graph()
        self.create_matrix_d(network_config)
        self.create_con_eq(network_config)
        self.create_con_ineq(network_config)
        self.create_boundary()

        self.state = np.zeros((2*self.num_boundaries + self.num_intersections, 1))
        self.demand_rate = np.zeros((2*self.num_boundaries, 1))
        self.ref = np.zeros((self.num_links, 1))

        self.set_state(network_config.boundaries)
        self.state_log = []
        self.control_signal_log = []
        self.cost_log = []

    def create_graph(self):
        graph = np.zeros((self.num_nodes, self.num_roads))

        for (road, ind) in zip(self.roads, range(self.num_roads)):
            road_conf = self.roads_configs[road]
            num_src = len(list(set([x["source"] for x in road_conf])))

            for route in road_conf:
                src = "{}_out".format(route["source"]) if route["source"][0] == 'B' else route["source"]
                dst = "{}_in".format(route["sink"]) if route["sink"][0] == 'B' else route["sink"]

                if src in self.nodes:
                    src_ind = self.nodes.index(src)
                    graph[src_ind, ind] = -1 if num_src == 1 else -route["ratio"]

                if dst in self.nodes:
                    dst_ind = self.nodes.index(dst)
                    graph[dst_ind, ind] = route["ratio"] if num_src == 1 else 1

        self.graph = graph

    def create_matrix_d(self, network_config):
        matrix = np.zeros((2*self.num_boundaries + self.num_intersections, 2*self.num_boundaries))
        for x in range(2*self.num_boundaries):
            matrix[x, x] = 1
        self.matrix_d = matrix

    def set_state(self, data):
        self.state = data

    def set_ref(self, data):
        self.ref = data
        self.vector_con_eq = data

    def set_demand_rate(self, data):
        self.demand_rate = data/3600

    def create_con_eq(self, network_config):
        self.matrix_con_eq = np.zeros((self.num_links, self.num_roads))
        for (link, ind) in zip(self.links, range(self.num_links)):
            for road in network_config.links[link]["roads"]:
                rind = self.roads.index(road)
                self.matrix_con_eq[ind, rind] = 1

    def create_con_ineq(self, network_config):
        matrix_con_ineq_1 = np.zeros((self.num_intersections, self.num_roads))
        vector_con_ineq_1 = np.zeros((self.num_intersections, 1))
        matrix_con_ineq_2 = None
        total_num_roads_out = 0

        for (intersection, ind) in zip(self.intersection_list, range(self.num_intersections)):
            jind = self.nodes.index(intersection)
            vector_con_ineq_1[ind, 0] = (self.area_rate/3600)*network_config.intersections[intersection]["max_flow"]
            in_roads = network_config.intersections[intersection]["incoming_roads"]
            out_roads = network_config.intersections[intersection]["outcome_roads"]
            for road in in_roads:
                rind = self.roads.index(road)
                matrix_con_ineq_1[ind, rind] = self.graph[jind, rind]

            total_num_roads_out += len(out_roads)
            for road in out_roads:
                tmp_ineq_2 = np.zeros((1, self.num_roads))
                tmp_ineq_2[0, self.roads.index(road)] = 1
                for in_road in in_roads:
                    if road in in_roads[in_road]["dst_roads"]:
                        rind = self.roads.index(in_road)
                        int_out = in_roads[in_road]["dst_roads"].index(road)
                        # print("{} {} {} {} : {} {}".format(in_road, int_out, jind, rind, len(in_roads[in_road]["splitting_flows"]), self.graph.shape))
                        tmp_ineq_2[0, rind] = -in_roads[in_road]["splitting_flows"][int_out]*self.graph[jind, rind]
                matrix_con_ineq_2 = tmp_ineq_2 if matrix_con_ineq_2 is None else np.vstack((matrix_con_ineq_2, tmp_ineq_2))

        vector_con_ineq_2 = np.ones((total_num_roads_out, 1))*self.acceptable_flow_error
        # matrix_con_ineq_2 = np.vstack((matrix_con_ineq_2, matrix_con_ineq_2))
        # vector_con_ineq_2 = np.vstack((vector_con_ineq_2, -vector_con_ineq_2))

        # self.matrix_con_ineq = np.vstack((matrix_con_ineq_1, matrix_con_ineq_2))
        # self.vector_con_ineq = np.vstack((vector_con_ineq_1, vector_con_ineq_2))

        self.matrix_con_ineq = matrix_con_ineq_1
        self.vector_con_ineq = vector_con_ineq_1

        self.matrix_con_ineq_2 = matrix_con_ineq_2
        self.vector_con_ineq_2 = vector_con_ineq_2

    def create_boundary(self):
        lb = [0]*self.num_roads
        ub = [250]*self.num_roads
        self.bounds = Bounds(lb, ub)

    def cost(self, u):
        E = self.state.reshape((-1, 1)) + np.matmul(self.graph, u.reshape((-1, 1))) + self.area_rate*np.matmul(self.matrix_d, self.demand_rate)
        return E, (LA.norm(E, 2)**2)/2

    def cost_2(self, u):
        E = np.matmul(self.matrix_con_ineq_2, u.reshape((-1, 1))) - self.vector_con_ineq_2
        return E, (LA.norm(E, 2)**2)/2

    def func(self, u):
        E, E_n = self.cost(u)
        E2, E2_n = self.cost_2(u)
        return E_n + 100*E2_n

    def func_dar(self, u):
        E, E_n = self.cost(u)
        E2, E2_n = self.cost_2(u)
        E_dar = np.matmul(self.graph.T, E)
        E2_dar = np.matmul(self.matrix_con_ineq_2.T, E2)
        return np.squeeze(np.asarray(E_dar + 100*E2_dar))

    def get_control_signal(self):
        initial = np.zeros((self.num_roads, 1))

        eq_cons = {'type': 'eq',
                    'fun': lambda x: np.squeeze(np.matmul(self.matrix_con_eq, x.reshape((-1, 1))) - self.vector_con_eq),
                    'jac': lambda x: self.matrix_con_eq}

        ineq_cons = {'type': 'ineq',
                    'fun': lambda x: np.squeeze(np.matmul(self.matrix_con_ineq, x.reshape((-1, 1))) - self.vector_con_ineq),
                    'jac': lambda x: self.matrix_con_ineq}

        res = minimize(self.func, initial, method='SLSQP', jac=self.func_dar, constraints=[eq_cons, ineq_cons], options={'disp': False}, bounds=self.bounds)
        control_signal = res.x.reshape((-1, 1))
        self.control_signal_log.append(control_signal)
        self.cost_log.append(self.func(control_signal))
        self.state_log.append(self.state.tolist())
        # input(control_signal)
        return control_signal

    def save(self, name, output_path):
        filename = "{}/controllers/area/{}.mat".format(output_path, name)
        data = {
            "state_log": self.state_log,
            "control_signal_log": self.control_signal_log,
            "cost_log": self.cost_log,
            }
        File.save_mat(filename, data)


class Area():

    def __init__(self, intermediary, name, controller_config, network_config):

        self.name = name
        self.configs = network_config.areas[name]
        self.boundaries = network_config.boundaries
        self.intersections_config = network_config.intersections

        self.network_rate = network_config.rates["network"]
        self.area_rate = network_config.rates["area"]

        self.boundary_list = Agreement.get_area_boundary_list(name, self.boundaries)
        self.intersection_list = Agreement.get_area_intersection_list(name, network_config)
        self.network_links = Agreement.get_network_link_list(network_config)
        self.links = Agreement.get_area_link_list(name, network_config)

        self.num_boundaries = len(self.boundary_list)
        self.num_intersections = len(self.intersection_list)
        self.num_network_links = len(self.network_links)
        self.num_links = len(self.links)

        self.socket = intermediary.client_register(name, "controller")
        self.socket.add_endpoint("upper_data_receiver", self.upper_data_receiver)
        self.socket.add_endpoint("lower_data_receiver", self.lower_data_receiver)

        self.model = Model(name, network_config, self.socket)
        self.state = np.zeros((2*self.num_boundaries + self.num_intersections, 1))
        self.demand_rate = np.zeros((2*self.num_boundaries, 1))
        self.create_aggregation_matrix(network_config)
        self.create_distribution_matrix(network_config)

    def update(self):
        self.get_boundary_demand()
        self.model.set_state(self.state)
        self.model.set_demand_rate(self.demand_rate)
        control_signal = self.model.get_control_signal()
        self.send_data_lower(control_signal)
        self.send_data_upper(np.matmul(self.aggregation_matrix, self.model.state))

    def send_data_lower(self, data):
        for intersection in self.intersection_list:
            self.socket.post(intersection, "controller", "upper_data_receiver", {"signal": data})

    def send_data_upper(self, data):
        self.socket.post("network_layer", "controller", "lower_data_receiver", {"signal": data, "area": self.name})

    def get_boundary_demand(self):
        for (boundary, bind) in zip(self.boundary_list, range(self.num_boundaries)):
            data = self.socket.post(boundary, "module_boundary", "get_data", {})
            self.state[2*bind, 0] = -data["sink"]
            self.state[(2*bind) + 1, 0] = data["source"]
            self.demand_rate[2*bind, 0] = -data["departure_rate"]
            self.demand_rate[(2*bind) + 1, 0] = data["arrival_rate"]

    def create_aggregation_matrix(self, network_config):
        matrix = np.ones((1, 2*self.num_boundaries + self.num_intersections))
        for x in range(2*self.num_boundaries):
            matrix[0, x] = 0
        self.aggregation_matrix = matrix

    def create_distribution_matrix(self, network_config):
        matrix = np.zeros((self.num_links, self.num_network_links))
        for (link, ind) in zip(self.links, range(self.num_links)):
            src = self.network_links.index(link)
            if self.name == network_config.links[link]["source_node"]:
                matrix[ind, src] = 1
            else:
                matrix[ind, src] = network_config.links[link]["ratio"]
        self.distribution_matrix = (self.area_rate/self.network_rate)*matrix

    def upper_data_receiver(self, data):
        signal = data["signal"]
        self.model.set_ref(np.matmul(self.distribution_matrix, signal))

    def lower_data_receiver(self, data):
        intersection = data["intersection"]
        signal = data["signal"]
        self.state[2*self.num_boundaries + self.intersection_list.index(intersection), 0] = signal

    def save(self, output_path):
        self.model.save(self.name, output_path)

    def model_save(self):
        filename = "models/np3/area/{}.mat".format(self.name)
        data = {
            "state": self.model.state,
            "graph": self.model.graph,
            "matrix_con_eq": self.model.matrix_con_eq,
            "vector_con_eq": self.model.vector_con_eq,
            "matrix_con_ineq": self.model.matrix_con_ineq,
            "vector_con_ineq": self.model.vector_con_ineq,
            "distribution_matrix": self.distribution_matrix,
            "aggregation_matrix": self.aggregation_matrix,
            "reduce_matrix": self.reduce_matrix,
            }
        File.save_mat(filename, data)
