from libs.etc.files import File
import numpy as np
import warnings
import traci
import pdb


warnings.filterwarnings("ignore", message="numpy.dtype size changed")
warnings.filterwarnings("ignore", message="numpy.ufunc size changed")


class GroundTruthFlow():
    def __init__(self, road, intersection_configs, intermediary):
        self.num_dir = len(intersection_configs["incoming_roads"][road]['dst_roads'])
        self.state = [[0]*self.num_dir]
        self.flow = [[0]*self.num_dir]
        self.phase = [0]
        self.edges = intersection_configs["incoming_roads"][road]['edges']
        self.outcome_edge = [intersection_configs["outcome_roads"][dst_road]['edges'][0] for dst_road in intersection_configs["incoming_roads"][road]['dst_roads']]
        self.socket = intermediary.client_register(road, "module_ground")

    def count_next_direction_by_routes(self, routes):
        tmp = [0]*self.num_dir
        for route in routes:
            dir = self.get_next_direction(route)
            if dir != -1:
                tmp[dir] += 1
        return tmp

    def count_next_direction(self, veh_ids):
        data = self.socket.post("vehicle_route", "module_dummy", "get_vehicle_route", {"veh_ids": veh_ids})
        return self.count_next_direction_by_routes(data["routes"])

    def get_next_direction(self, route):
        for (edge, dir) in zip(self.outcome_edge, range(self.num_dir)):
            if edge in route:
                return dir
        return -1

    def update_veh_route(self):
        veh_ids = []
        for edge in self.edges:
            veh_ids += traci.edge.getLastStepVehicleIDs(edge)
        self.socket.post("vehicle_route", "module_dummy", "put_vehicle_id", {"veh_ids": veh_ids})

    def update_state(self):
        veh_ids = []
        for edge in self.edges:
            veh_ids += traci.edge.getLastStepVehicleIDs(edge)
        self.state.append(self.count_next_direction(veh_ids))

    def update_flow(self, passed_veh_ids):
        data = self.socket.post("vehicle_route", "module_dummy", "get_vehicle_route", {"veh_ids": passed_veh_ids})
        flow = self.count_next_direction_by_routes(data["routes"])
        self.flow.append(flow)

    def update(self, phase, routes):
        self.update_state()
        self.update_flow(routes)
        self.phase.append(phase)
