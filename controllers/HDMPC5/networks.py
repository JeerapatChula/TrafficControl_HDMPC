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
        self.boundaries = network_config.boundaries
        self.area_nodes = Agreement.get_network_node_list(network_config)
        self.boundary_nodes = Agreement.get_network_boundary_list(network_config)
        self.link_list = Agreement.get_network_link_list(network_config)
        self.network_rate = network_config.rates["network"]

        self.num_areas = len(self.areas)
        self.num_boundaries = len(self.boundaries)
        self.num_links = len(self.links)

        self.state = np.zeros((self.num_areas, 1))
        self.demand = np.zeros((2*self.num_boundaries, 1))
        self.demand_rate = np.zeros((2*self.num_boundaries, 1))/3600

        self.area_demand = np.zeros((self.num_areas, 1))
        for (area, ind) in zip(self.area_nodes, range(self.num_areas)):
            self.area_demand[ind, 0] = self.areas[area]["demand_rate"]/3600
        # input(self.area_demand)
        self.state_log = []
        self.control_signal_log = []
        self.demand_log = []
        self.demand_rate_log = []
        self.cost_log = []

        self.create_graph()
        self.create_matrix_d()
        self.create_con_ineq()
        self.create_con_bound()

    def set_state(self, sate):
        self.state = sate

    def set_demand(self, demand, demand_rate):
        self.demand = demand
        self.demand_rate = demand_rate

    def create_matrix_d(self):
        matrix = np.zeros((self.num_areas, 2*self.num_boundaries))
        for (boundary, bind) in zip(self.boundary_nodes, range(self.num_boundaries)):
            aind = self.area_nodes.index(self.boundaries[boundary]["area"])
            matrix[aind, 2*bind] = -1
            matrix[aind, (2*bind) + 1] = 1

        self.matrix_d = matrix

    def create_graph(self):
        areas = self.area_nodes
        graph = np.zeros((self.num_areas, self.num_links))

        for (link, ind) in zip(self.link_list, range(self.num_links)):
            src = areas.index(self.links[link]["source_node"])
            dst = areas.index(self.links[link]["sink_node"])
            graph[src, ind] = -1
            graph[dst, ind] = self.links[link]["ratio"]

        self.graph = graph
        # input(graph)

    def cost(self, u):
        E = np.matmul(self.graph, u.reshape((-1, 1))) + self.state.reshape((-1, 1)) + self.network_rate*self.area_demand + np.matmul(self.matrix_d, self.demand + self.network_rate*self.demand_rate)
        return E, (LA.norm(E, 2)**2)/2

    def func(self, u):
        E, E_n = self.cost(u)
        return E_n

    def func_dar(self, u):
        E, E_n = self.cost(u)
        E_dar = np.matmul(self.graph.T, E)
        return np.squeeze(np.asarray(E_dar))

    # def create_con_ineq(self):
    #     self.matrix_con_ineq = np.zeros((self.num_areas, self.num_links))
    #     self.vector_con_ineq = np.zeros((self.num_areas, 1))
    #     for (link, ind) in zip(self.link_list, range(self.num_links)):
    #         area = self.links[link]["source_node"]
    #         src = self.area_nodes.index(self.links[link]["source_node"])
    #         dst = self.area_nodes.index(self.links[link]["sink_node"])
    #         boundaries = Agreement.get_area_boundary_list(area, self.boundaries)
    #         total_demand = 0
    #         for boundary in boundaries:
    #             bind = self.boundary_nodes.index(boundary)
    #             total_demand += self.demand[2*bind + 1, 0] + self.network_rate*self.demand_rate[2*bind + 1, 0]
    #         self.vector_con_ineq[src, 0] = self.state[src, 0] + total_demand
    #         self.matrix_con_ineq[src, ind] = 1
    #         self.matrix_con_ineq[dst, ind] = -self.links[link]["ratio"]

    def create_con_ineq(self):
        self.matrix_con_ineq = np.zeros((2*self.num_areas, self.num_links))
        self.vector_con_ineq = np.zeros((2*self.num_areas, 1))

        for (area, ind) in zip(self.area_nodes, range(self.num_areas)):

            boundaries = Agreement.get_area_boundary_list(area, self.boundaries)
            total_sink = 0
            total_source = 0
            for boundary in boundaries:
                bind = self.boundary_nodes.index(boundary)
                total_sink += self.demand[2*bind, 0] + self.network_rate*self.demand_rate[2*bind, 0]
                total_source += self.demand[2*bind + 1, 0] + self.network_rate*self.demand_rate[2*bind + 1, 0]
            self.vector_con_ineq[2*ind, 0] = abs(total_sink)
            self.vector_con_ineq[2*ind + 1, 0] = abs(total_source) + self.state[ind, 0]

        for (link, ind) in zip(self.link_list, range(self.num_links)):
            src_area = self.links[link]["source_node"]
            dst_area = self.links[link]["sink_node"]
            src_ind = self.area_nodes.index(src_area)
            dst_ind = self.area_nodes.index(dst_area)
            self.matrix_con_ineq[2*dst_ind, ind] = self.links[link]["ratio"]
            self.matrix_con_ineq[2*src_ind, ind] = -1
            self.matrix_con_ineq[2*dst_ind + 1, ind] = -self.links[link]["ratio"]
            self.matrix_con_ineq[2*src_ind + 1, ind] = 1

        self.matrix_con_ineq = self.matrix_con_ineq
        self.vector_con_ineq = self.vector_con_ineq


    def create_con_bound(self):
        lb = [50]*self.num_links
        ub = [np.inf]*self.num_links
        for (link, ind) in zip(self.links, range(self.num_links)):
            ub[ind] = (self.network_rate*self.links[link]["capacity"]/3600)*1.10
        self.bounds = Bounds(lb, ub)

    def get_control_signal(self):

        initial = np.zeros((self.num_links, 1))
        self.create_con_ineq()
        ineq_cons = {'type': 'ineq',
                    'fun' : lambda x: np.squeeze(np.matmul(self.matrix_con_ineq, x.reshape((-1, 1))) - self.vector_con_ineq),
                    'fun2' : lambda x: np.squeeze(np.matmul(self.matrix_con_ineq, x.reshape((-1, 1)))),
                    'jac' : lambda x: self.matrix_con_ineq}

        res = minimize(self.func, initial, method='SLSQP', jac=self.func_dar, constraints=[], options={'disp': False}, bounds=self.bounds)
        control_signal = res.x.reshape((-1, 1))
        self.control_signal_log.append(control_signal)
        self.cost_log.append(self.func(control_signal))
        self.state_log.append(self.state.tolist())
        self.demand_log.append(self.demand.tolist())
        self.demand_rate_log.append(self.demand_rate.tolist())
        # input(self.vector_con_ineq)
        # input(control_signal)
        return control_signal

    def save(self, output_path):
        filename = "{}/controllers/network_layer.mat".format(output_path)
        data = {
            "state_log": self.state_log,
            "control_signal_log": self.control_signal_log,
            "cost_log": self.cost_log,
            "demand_log": self.demand_log,
            "demand_rate_log": self.demand_rate_log,
            }
        File.save_mat(filename, data)


class Network():

    def __init__(self, intermediary, controller_config, network_config):
        self.name = "network_layer"
        self.areas = network_config.areas
        self.areas_list = Agreement.get_network_area_list(network_config)
        self.boundaries = Agreement.get_network_boundary_list(network_config)
        self.links = network_config.links

        self.state = np.zeros((len(self.areas), 1))
        self.model = Model(network_config)

        self.socket = intermediary.client_register(self.name, "controller")
        self.socket.add_endpoint("lower_data_receiver", self.lower_data_receiver)
        # self.model_save()

    def update(self):
        demand, demand_rate = self.get_boundary_demand()
        self.model.set_demand(demand, demand_rate)
        self.model.set_state(self.state)
        control_signal = self.model.get_control_signal()
        self.send_data_lower(control_signal)

    def get_boundary_demand(self):
        num_boundaries = len(self.boundaries)
        demand = np.zeros((2*num_boundaries, 1))
        demand_rate = np.zeros((2*num_boundaries, 1))
        for (boundary, bind) in zip(self.boundaries, range(num_boundaries)):
            data = self.socket.post(boundary, "module_boundary", "get_data", {})
            demand[2*bind, 0] = data["sink"]
            demand[(2*bind) + 1, 0] = data["source"]
            demand_rate[2*bind, 0] = data["departure_rate"]
            demand_rate[(2*bind) + 1, 0] = data["arrival_rate"]
        return demand, demand_rate

    def save(self, output_path):
        self.model.save(output_path)

    def send_data_lower(self, data):
        for area in self.areas_list:
            self.socket.post(area, "controller", "upper_data_receiver", {"signal": data})

    def lower_data_receiver(self, data):
        area = data["area"]
        signal = data["signal"]
        self.state[self.areas_list.index(area), 0] = signal

    def model_save(self):
        filename = "models/np3/network.mat"
        data = {
            "graph": self.model.graph,
            "matrix_d": self.model.matrix_d,
            "matrix_con_ineq": self.model.matrix_con_ineq,
            "vector_con_ineq": self.model.vector_con_ineq,
            }
        File.save_mat(filename, data)
