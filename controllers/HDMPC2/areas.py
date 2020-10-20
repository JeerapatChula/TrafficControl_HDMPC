from .agreements import Agreement
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

    def __init__(self, area, network_config):
        self.areas_configs = network_config.areas[area]
        self.intersection_list = self.areas_configs["intersections"]

        self.links = Agreement.get_area_link_list(area, network_config)
        self.nodes = Agreement.get_area_node_list(area, network_config)
        self.adjacend_areas = Agreement.get_adjacend_areas(area, network_config)
        self.num_links = len(self.links)
        self.num_nodes = len(self.nodes)
        self.num_adjacend_areas = len(self.adjacend_areas)
        self.create_graph(network_config)
        self.create_constraint(network_config)

        self.state = np.zeros((self.num_nodes, 1))
        self.state_log = []
        self.input_log = []
        self.control_signal_log = []

    def update(self, signal):
        self.state += np.matmul(self.graph, signal)
        self.state_log.append(self.state.tolist())
        self.input_log.append(signal)

    def create_graph(self, network_config):
        graph = np.zeros((self.num_nodes, self.num_links))

        for intersection in self.intersection_list:
            incoming_roads = network_config.intersections[intersection]["incoming_roads"]
            outcome_roads = network_config.intersections[intersection]["outcome_roads"]

            link_index = None
            src_index = None
            dst_index = None

            for road in incoming_roads:
                link_index = self.links.index(road)
                src_index = self.nodes.index(incoming_roads[road]["boundary"])
                dst_index = self.nodes.index(intersection)
                graph[src_index, link_index] = -1
                graph[dst_index, link_index] = 1

            for road in outcome_roads:
                link_index = self.links.index(road)
                src_index = self.nodes.index(intersection)
                dst_index = self.nodes.index(outcome_roads[road]["boundary"])
                graph[src_index, link_index] = -1
                graph[dst_index, link_index] = 1

        self.graph = graph

    def set_state(self, boundaries):
        for boundary in boundaries:
            if boundary in self.nodes:
                ind = self.nodes.index(boundary)
                self.state[ind, 0] = boundaries[boundary]["source"] - boundaries[boundary]["sink"]

    def feed_upper_data(self, data):
        for x in range(self.num_adjacend_areas):
            self.state[x, 0] = 0
        for x in self.intersection_list:
            ind = self.nodes.index(x)
            self.state[ind, 0] = 0
        self.state += data

    def create_constraint(self, network_config):
        matrix_con_eq = None
        vector_con_eq = np.zeros((len(self.intersection_list), 1))
        matrix_con_ineq = None
        vector_con_ineq = np.zeros((len(self.intersection_list), 1))
        num_links = len(self.links)

        for (intersection, x) in zip(self.intersection_list, range(len(self.intersection_list))):
            max_flow = network_config.intersections[intersection]["max_flow"]
            in_roads = network_config.intersections[intersection]["incoming_roads"]
            out_roads = network_config.intersections[intersection]["outcome_roads"]

            matrix_ineq = np.zeros((1, num_links))
            matrix_eq = np.zeros((1, num_links))
            vector_con_ineq[x, 0] = max_flow

            for road in in_roads:
                ind = self.links.index(road)
                matrix_ineq[0, ind] = 1
                matrix_eq[0, ind] = 1

            for road in out_roads:
                ind = self.links.index(road)
                matrix_eq[0, ind] = -1

            matrix_con_eq = matrix_eq if matrix_con_eq is None else np.vstack((matrix_con_eq, matrix_eq))
            matrix_con_ineq = matrix_ineq if matrix_con_ineq is None else np.vstack((matrix_con_ineq, matrix_ineq))

        # input(matrix_con_ineq)
        # input(matrix_con_eq)
        # input(vector_con_ineq)
        # input(vector_con_eq)

        self.matrix_con_eq = matrix_con_eq
        self.vector_con_eq = vector_con_eq
        self.matrix_con_ineq = matrix_con_ineq
        self.vector_con_ineq = vector_con_ineq

    def func(self, x):
        E = np.matmul(self.graph, x.reshape((-1, 1))) + self.state.reshape((-1, 1))
        return (LA.norm(E, 2)**2)/2

    def func_dar(self, x):
        E_dar = np.matmul(self.graph, x.reshape((-1, 1))) + self.state.reshape((-1, 1))
        E_dar = np.matmul(self.graph.T, E_dar)
        return np.squeeze(np.asarray(E_dar))

    def get_control_signal(self):
        initial = np.zeros((self.num_links, 1))
        bounds = Bounds([0]*self.num_links, [np.inf]*self.num_links)

        eq_cons = {'type': 'eq',
                    'fun' : lambda x: np.squeeze(np.matmul(self.matrix_con_eq, x.reshape((-1, 1))) - self.vector_con_eq),
                    'jac' : lambda x: self.matrix_con_eq}

        ineq_cons = {'type': 'ineq',
                    'fun' : lambda x: np.squeeze(np.matmul(self.matrix_con_ineq, x.reshape((-1, 1))) - self.vector_con_ineq),
                    'jac' : lambda x: self.matrix_con_ineq}

        res = minimize(self.func, initial, method='SLSQP', jac=self.func_dar, constraints=[eq_cons, ineq_cons], options={'disp': False}, bounds=bounds)
        control_signal = res.x.reshape((-1, 1))
        self.control_signal_log.append(control_signal)
        # input(control_signal)
        return control_signal

    def save(self, name, output_path):
        filename = "{}/controllers/area/{}.mat".format(output_path, name)
        data = {
            "state_log": self.state_log,
            "input_log": self.input_log,
            "control_signal_log": self.control_signal_log,
            }
        File.save_mat(filename, data)


class Area():

    def __init__(self, intermediary, name, controller_config, network_config):

        self.name = name
        self.configs = network_config.areas[name]
        self.boundaries = network_config.boundaries
        self.area_road = Agreement.get_area_link_list(self.name, network_config)
        self.intersections_config = network_config.intersections

        self.model = Model(name, network_config)
        self.model.set_state(self.boundaries)
        self.create_aggregation_matrix(network_config)
        self.create_reduce_matrix(network_config)
        self.create_distribution_matrix(network_config)
        self.input = np.zeros((len(self.area_road), 1))

        self.socket = intermediary.client_register(name, "controller")
        self.socket.add_endpoint("upper_data_receiver", self.upper_data_receiver)
        self.socket.add_endpoint("lower_data_receiver", self.lower_data_receiver)

    def update(self):
        # input(self.input)
        self.model.update(self.input)
        control_signal = self.model.get_control_signal()
        self.send_data_lower(control_signal)
        self.input = np.zeros_like(self.input)

    def save(self, output_path):
        self.model.save(self.name, output_path)

    def send_data_lower(self, data):
        # FreeFlowSpeed = 40.0 # km/h
        # MaxDensity = 142.0 # veh/1km
        # MaxFlowRate = (FreeFlowSpeed*(MaxDensity/4.0))/3600.0 # veh/sec

        max_flows = min([self.intersections_config[x]["max_flow"] for x in self.configs["intersections"]])

        processed_signal = data/3600.0
        for intersection in self.configs["intersections"]:
            self.socket.post(intersection, "controller", "upper_data_receiver", {"raw_signal": data, "processed_signal": processed_signal})

    def send_data_upper(self, data):
        self.socket.post("network_layer", "controller", "lower_data_receiver", {"signal": data, "area": self.name})

    def create_aggregation_matrix(self, network_config):
        intersections_links = []
        self.intersections_links_name = []
        intersections_links_is_incomming = []
        for intersection in self.configs["intersections"]:
            links = Agreement.get_intersection_link_list(intersection, network_config)
            links_is_incomming = Agreement.get_intersection_links_is_incomming(intersection, network_config)
            for (link, is_incomming) in zip(links, links_is_incomming):
                intersections_links.append(link)
                self.intersections_links_name.append(intersection)
                intersections_links_is_incomming.append(is_incomming)

        num_my_roads = len(self.area_road)
        num_intersections_links = len(intersections_links)
        matrix = np.zeros((num_my_roads, num_intersections_links))

        for (link, dst) in zip(self.area_road, range(len(self.area_road))):
            if intersections_links.count(link) == 1:
                src = intersections_links.index(link)
                matrix[dst, src] = 1

        for (link, src) in zip(intersections_links, range(len(intersections_links))):
            if not intersections_links_is_incomming[src]:
                dst = self.area_road.index(link)
                matrix[dst, src] = 1
        self.aggregation_matrix = matrix

    def create_reduce_matrix(self, network_config):
        area_outcome_links_list = Agreement.get_area_outcome_links_list(self.name, network_config)
        # area_outcome_roads_list = Agreement.get_area_outcome_roads_list(self.name, network_config)
        matrix = np.zeros((len(area_outcome_links_list), len(self.area_road)))
        for (link, dst) in zip(area_outcome_links_list, range(len(area_outcome_links_list))):
            for road in network_config.links[link]["roads"]:
                src = self.area_road.index(road)
                matrix[dst, src] = 1
        self.reduce_matrix = matrix

    def create_distribution_matrix(self, network_config):
        network_links = Agreement.get_network_link_list(network_config)
        area_nodes = Agreement.get_area_node_list(self.name, network_config)
        self.num_network_link = len(network_links)
        self.num_area_nodes = len(area_nodes)
        matrix = np.zeros((self.num_area_nodes, self.num_network_link))
        for (link, src) in zip(network_links, range(len(network_links))):
            if network_config.links[link]["source_node"] in area_nodes:
                dst = area_nodes.index(network_config.links[link]["source_node"])
                matrix[dst, src] = 1
            elif network_config.links[link]["sink_node"] in area_nodes:
                dst = area_nodes.index(network_config.links[link]["sink_node"])
                matrix[dst, src] = -1
        self.distribution_matrix = matrix

    def upper_data_receiver(self, data):
        signal = data["signal"]
        tmp = np.matmul(self.distribution_matrix, signal)
        self.model.feed_upper_data(tmp)

    def lower_data_receiver(self, data):
        intersection = data["intersection"]
        signal = data["signal"]
        ind = np.isin(self.intersections_links_name, intersection)
        ind = np.where(ind == True)
        tmp = np.zeros((len(self.intersections_links_name), 1))
        for (x, y) in zip(ind[0], signal):
            tmp[x, 0] = y

        received_signal = np.matmul(self.aggregation_matrix, tmp)
        self.input += received_signal
        reduced_signal = np.matmul(self.reduce_matrix, received_signal)
        self.send_data_upper(reduced_signal)
