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
        self.intersection_list = self.areas_configs["intersections"]
        self.socket = socket
        self.links = Agreement.get_area_link_list(area, network_config)
        self.nodes = Agreement.get_area_node_list(area, network_config)
        self.adjacend_areas = Agreement.get_adjacend_areas(area, network_config)
        self.num_links = len(self.links)
        self.num_nodes = len(self.nodes)
        self.num_adjacend_areas = len(self.adjacend_areas)
        self.create_graph(network_config)
        self.create_constraint(network_config)
        self.create_boundary(network_config)

        self.state = np.zeros((self.num_nodes, 1))
        self.set_state(network_config.boundaries)
        self.state_log = []
        self.input_log = []
        self.control_signal_log = []

        # input(self.links)
        # input(self.nodes)

    def update(self, signal):
        self.state += np.matmul(self.graph, signal)

        for intersection in self.intersection_list:
            data = self.socket.post(intersection, "controller", "get_total_demand", {})
            ind = self.nodes.index(intersection)
            self.state[ind, 0] = data["total_demand"]

        self.state_log.append(self.state.tolist())
        self.input_log.append(signal)

    def create_graph(self, network_config):
        graph = np.zeros((self.num_nodes, self.num_links))

        areas = list(network_config.areas.keys())

        for intersection in self.intersection_list:
            incoming_roads = network_config.intersections[intersection]["incoming_roads"]
            outcome_roads = network_config.intersections[intersection]["outcome_roads"]

            link_index = None
            src_index = None
            dst_index = None

            for road in incoming_roads:
                boundary = incoming_roads[road]["boundary"]
                str = "{}_out".format(boundary) if boundary not in self.intersection_list else boundary

                link_index = self.links.index(road)
                src_index = self.nodes.index(str)
                dst_index = self.nodes.index(intersection)
                graph[src_index, link_index] = -1
                graph[dst_index, link_index] = 1

            for road in outcome_roads:
                boundary = outcome_roads[road]["boundary"]
                str = "{}_in".format(boundary) if boundary not in self.intersection_list else boundary

                link_index = self.links.index(road)
                src_index = self.nodes.index(intersection)
                dst_index = self.nodes.index(str)
                graph[src_index, link_index] = -1
                graph[dst_index, link_index] = 1

        self.graph = graph

    def set_state(self, boundaries):
        for boundary in boundaries:
            boundary_in = "{}_in".format(boundary)
            boundary_out = "{}_out".format(boundary)
            if boundary_in in self.nodes:
                ind_in = self.nodes.index(boundary_in)
                ind_out = self.nodes.index(boundary_out)
                self.state[ind_in, 0] = -boundaries[boundary]["sink"]*1.20
                self.state[ind_out, 0] = boundaries[boundary]["source"]*1.20

    def feed_upper_data(self, data):
        for x in range((self.num_adjacend_areas*2)):
            self.state[x, 0] = 0
        # for x in range(len(self.intersection_list)):
        #     self.state[-x, 0] = 0
        self.state += data

    def create_constraint(self, network_config):
        matrix_con_eq = None
        vector_con_eq = np.zeros((len(self.intersection_list), 1))
        matrix_con_ineq = None
        vector_con_ineq = np.zeros((len(self.intersection_list), 1))
        num_links = len(self.links)

        matrix_con_eq_2 = None
        num_road_out = 0

        matrix_con_ineq_2 = None
        vector_con_ineq_2 = None

        for (intersection, x) in zip(self.intersection_list, range(len(self.intersection_list))):
            max_flow = network_config.intersections[intersection]["max_flow"]
            in_roads = network_config.intersections[intersection]["incoming_roads"]
            out_roads = network_config.intersections[intersection]["outcome_roads"]

            matrix_ineq = np.zeros((1, num_links))
            vector_con_ineq[x, 0] = max_flow

            for road in in_roads:
                ind = self.links.index(road)
                matrix_ineq[0, ind] = 1

                bound = in_roads[road]["boundary"]
                if bound in network_config.boundaries.keys():
                    tmp_1 = np.zeros((1, num_links))
                    tmp_2 = np.zeros((1, 1))
                    tmp_1[0, ind] = 1
                    tmp_2[0, 0] = network_config.boundaries[bound]["source"]/(3)
                    matrix_con_ineq_2 = tmp_1 if matrix_con_ineq_2 is None else np.vstack((matrix_con_ineq_2, tmp_1))
                    vector_con_ineq_2 = tmp_2 if vector_con_ineq_2 is None else np.vstack((vector_con_ineq_2, tmp_2))

            for road in out_roads:
                ind = self.links.index(road)

                matrix_eq_2 = np.zeros((1, num_links))
                matrix_eq_2[0, ind] = -1
                num_road_out += 1
                for in_road in in_roads:
                    if road in in_roads[in_road]["dst_roads"]:
                        ind_in = self.links.index(in_road)
                        int_out = in_roads[in_road]["dst_roads"].index(road)
                        matrix_eq_2[0, ind_in] = in_roads[in_road]["splitting_flows"][int_out]

                matrix_con_eq_2 = matrix_eq_2 if matrix_con_eq_2 is None else np.vstack((matrix_con_eq_2, matrix_eq_2))
            matrix_con_ineq = matrix_ineq if matrix_con_ineq is None else np.vstack((matrix_con_ineq, matrix_ineq))

        self.matrix_con_eq = matrix_con_eq_2
        self.vector_con_eq = np.zeros((num_road_out, 1))
        self.matrix_con_ineq = matrix_con_ineq
        self.vector_con_ineq = vector_con_ineq


    def create_boundary(self, network_config):
        lb = [0]*self.num_links
        ub = [np.inf]*self.num_links

        for intersection in self.intersection_list:
            in_roads = network_config.intersections[intersection]["incoming_roads"]
            out_roads = network_config.intersections[intersection]["outcome_roads"]

            for road in in_roads:
                bound = in_roads[road]["boundary"]
                if bound in network_config.boundaries.keys():
                    ind = self.links.index(road)
                    ub[ind] = network_config.boundaries[bound]["source"]/3

            for road in out_roads:
                bound = out_roads[road]["boundary"]
                if bound in network_config.boundaries.keys():
                    ind = self.links.index(road)
                    ub[ind] = network_config.boundaries[bound]["sink"]/3

        self.bounds = Bounds(lb, ub)

    def func(self, x):
        E1 = np.matmul(self.graph, x.reshape((-1, 1))) + self.state.reshape((-1, 1))
        E1 = (LA.norm(E1, 2)**2)/2
        E2 = np.matmul(self.matrix_con_eq, x.reshape((-1, 1)))
        E2 = (LA.norm(E2, 2)**2)/2
        return E1 + 10*E2

    def func_dar(self, x):
        E1_dar = np.matmul(self.graph, x.reshape((-1, 1))) + self.state.reshape((-1, 1))
        E1_dar = np.matmul(self.graph.T, E1_dar)
        E2_dar = np.matmul(self.matrix_con_eq, x.reshape((-1, 1)))
        E2_dar = np.matmul(self.matrix_con_eq.T, E2_dar)
        return np.squeeze(np.asarray(E1_dar + 10*E2_dar))

    def get_control_signal(self):
        initial = np.zeros((self.num_links, 1))

        eq_cons = {'type': 'eq',
                    'fun' : lambda x: np.squeeze(np.matmul(self.matrix_con_eq, x.reshape((-1, 1))) - self.vector_con_eq),
                    'jac' : lambda x: self.matrix_con_eq}

        ineq_cons = {'type': 'ineq',
                    'fun' : lambda x: np.squeeze(np.matmul(self.matrix_con_ineq, x.reshape((-1, 1))) - self.vector_con_ineq),
                    'jac' : lambda x: self.matrix_con_ineq}

        res = minimize(self.func, initial, method='SLSQP', jac=self.func_dar, constraints=[ineq_cons], options={'disp': False}, bounds=self.bounds)
        control_signal = res.x.reshape((-1, 1))
        self.control_signal_log.append(control_signal)

        # for (link, x) in zip(self.links, range(len(self.links))):
        #     print("{} : {}".format(link, control_signal[x]))
        # input()
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


        self.socket = intermediary.client_register(name, "controller")
        self.socket.add_endpoint("upper_data_receiver", self.upper_data_receiver)
        self.socket.add_endpoint("lower_data_receiver", self.lower_data_receiver)

        self.model = Model(name, network_config, self.socket)
        self.create_aggregation_matrix(network_config)
        self.create_reduce_matrix(network_config)
        self.create_distribution_matrix(network_config)
        self.input = np.zeros((len(self.area_road), 1))

        filename = "models/np3/area/{}.mat".format(name)
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
        # File.save_mat(filename, data)

    def update(self):
        # input(self.input)
        self.model.update(self.input)
        control_signal = self.model.get_control_signal()
        self.send_data_lower(control_signal)
        self.input = np.zeros_like(self.input)

    def save(self, output_path):
        self.model.save(self.name, output_path)

    def send_data_lower(self, data):
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
            source_node = "{}_out".format(network_config.links[link]["source_node"])
            sink_node = "{}_in".format(network_config.links[link]["sink_node"])
            if source_node in area_nodes and network_config.links[link]["sink_node"] == self.name:
                dst = area_nodes.index(source_node)
                matrix[dst, src] = 1
            elif sink_node in area_nodes and network_config.links[link]["source_node"] == self.name:
                dst = area_nodes.index(sink_node)
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
