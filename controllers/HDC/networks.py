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

    def __init__(self, network_config):
        self.areas = network_config.areas
        self.links = network_config.links

        self.num_areas = len(self.areas)
        self.num_links = len(self.links)

        self.graph = np.zeros((self.num_areas, self.num_links))
        self.state = np.zeros((self.num_areas, 1))

        self.state_log = []
        self.input_log = []
        self.control_signal_log = []

        self.create_graph(network_config)

    def set_state(self, sate):
        for i in range(len(sate)):
            self.state[i] = sate[i]
        # self.state_log.append(self.state)

    def create_graph(self, network_config):
        areas = Agreement.get_network_node_list(network_config)
        links = Agreement.get_network_link_list(network_config)
        graph = np.zeros_like(self.graph)

        for link in self.links:
            link_ind = links.index(link)
            src = areas.index(self.links[link]["source_node"])
            dst = areas.index(self.links[link]["sink_node"])
            graph[src, link_ind] = -self.links[link]["ratio"]
            graph[dst, link_ind] = self.links[link]["ratio"]

        self.graph = graph

    def update(self, signal):
        self.state += np.matmul(self.graph, signal)
        self.state_log.append(self.state.tolist())

    def func(self, x):
        E = np.matmul(self.graph, x.reshape((-1, 1))) + self.state.reshape((-1, 1))
        return (LA.norm(E, 2)**2)/2

    def func_dar(self, x):
        E_dar = np.matmul(self.graph, x.reshape((-1, 1))) + self.state.reshape((-1, 1))
        E_dar = np.matmul(self.graph.T, E_dar)
        return np.squeeze(np.asarray(E_dar))

    def get_control_signal(self):
        initial = np.zeros((self.num_links,1))
        bounds = Bounds([0]*self.num_links, [np.inf]*self.num_links)
        res = minimize(self.func, initial, method='SLSQP', jac=self.func_dar, constraints=[], options={'disp': False}, bounds=bounds)
        control_signal = res.x.reshape((-1, 1))
        self.control_signal_log.append(control_signal)
        return control_signal

    def save(self, output_path):
        filename = "{}/controllers/network_layer.mat".format(output_path)
        data = {
            "state_log": self.state_log,
            "input_log": self.input_log,
            "control_signal_log": self.control_signal_log,
            }
        File.save_mat(filename, data)


class Network():

    def __init__(self, intermediary, controller_config, network_config):
        self.name = "network_layer"
        self.areas = network_config.areas
        self.areas_list = Agreement.get_network_node_list(network_config)
        self.links = network_config.links
        state_initial = [self.areas[area]["source"] - self.areas[area]["sink"] for area in self.areas]


        self.model = Model(network_config)
        self.model.set_state(state_initial)
        self.create_aggregation_matrix(network_config)

        self.input = np.zeros((len(self.links), 1))

        self.socket = intermediary.client_register(self.name, "controller")
        self.socket.add_endpoint("lower_data_receiver", self.lower_data_receiver)

        # control_signal = self.model.get_control_signal()
        # self.send_data_lower(control_signal)

    def update(self):
        self.model.update(self.input)
        control_signal = self.model.get_control_signal()
        self.send_data_lower(control_signal)
        self.input = np.zeros_like(self.input)

    def save(self, output_path):
        self.model.save(output_path)

    def create_aggregation_matrix(self, network_config):
        areas = self.areas_list
        links = Agreement.get_network_link_list(network_config)
        num_links = len(links)
        matrix = np.zeros((num_links, num_links))
        self.area_links_name = []
        all_outcome_links = []
        for area in areas:
            outcome_links = Agreement.get_area_outcome_links_list(area, network_config)
            for link in outcome_links:
                self.area_links_name.append(area)
                all_outcome_links.append(link)
        for (link, dst) in zip(links, range(num_links)):
            src = all_outcome_links.index(link)
            matrix[dst, src] = 1
        self.aggregation_matrix = matrix

    def send_data_lower(self, data):
        for area in self.areas_list:
            self.socket.post(area, "controller", "upper_data_receiver", {"signal": data})

    def lower_data_receiver(self, data):
        area = data["area"]
        signal = data["signal"]

        ind = np.isin(self.area_links_name, area)
        ind = np.where(ind == True)
        tmp = np.zeros((len(self.area_links_name), 1))
        for (x, y) in zip(ind[0], signal):
            tmp[x, 0] = y
        self.input += np.matmul(self.aggregation_matrix, tmp)
