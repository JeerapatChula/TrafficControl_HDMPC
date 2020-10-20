from .detectors import Detector
from libs.etc.files import File
from .ground_truth_flows import GroundTruthFlow
import traci
import numpy as np


class Road():

    def __init__(self, intersection, name, configs, intermediary):
        self.intersection = intersection
        self.name = name
        self.time = 0
        self.last_update = 0
        self.edges = configs["edges"]
        self.num_lane_in = configs["num_lane_in"]
        self.num_lane_out = configs["num_lane_out"]
        self.intermediary = intermediary

        self.is_incoming = True if "detector_out" in configs else False

        if self.is_incoming:
            self.detector_out_dst_roads = configs["detector_out_dst_roads"]

        self.detector_in = [Detector(configs["detector_in"].format(x), intermediary) for x in range(self.num_lane_in)]
        self.detector_out = [Detector(configs["detector_out"].format(x), intermediary) for x in range(self.num_lane_out)] if self.is_incoming else []

        self.num_vehicles = []
        self.num_vehicles_in = np.zeros((1, self.num_lane_in))
        self.num_vehicles_out = np.zeros((1, self.num_lane_out))
        self.travel_time = []
        self.ground_truth = None

    def create_ground_truth(self, intersection_configs):
        self.ground_truth = GroundTruthFlow(self.name, intersection_configs, self.intermediary)

    def update_ground_truth(self, phase):
        passed_veh_ids = []
        # passed_veh_routes = []
        for detector in self.detector_out:
            passed_veh_ids += detector.passed_veh_ids
            # passed_veh_routes.append(detector.passed_veh_routes)
            detector.passed_veh_ids = []
            # detector.passed_veh_routes = []
        self.ground_truth.update(phase, passed_veh_ids)

    def get_actual_state(self):
        return self.ground_truth.state[-1]

    def get_last_actual_departed(self):
        return self.ground_truth.flow[-1]

    def update(self):
        if self.is_incoming:
            self.ground_truth.update_veh_route()
        [detector.update() for detector in self.detector_in]
        [detector.update() for detector in self.detector_out]
        self.travel_time.append(sum([traci.edge.getTraveltime(edge) for edge in self.edges])/len(self.edges))
        self.num_vehicles.append(sum([traci.edge.getLastStepVehicleNumber(edge) for edge in self.edges]))
        self.num_vehicles_in = np.vstack((self.num_vehicles_in, [detector.get_veh_passed(1) for detector in self.detector_in]))
        if self.is_incoming:
            self.num_vehicles_out = np.vstack((self.num_vehicles_out, [detector.get_veh_passed(1) for detector in self.detector_out]))


    def get_vehicles_in(self, duration):
        duration = len(self.num_vehicles_in) if duration == -1 else duration
        return self.num_vehicles_in[-int(duration):, :].sum(axis=0)

    def get_vehicles_out(self, duration):
        duration = len(self.num_vehicles_out) if duration == -1 else duration
        departed = self.num_vehicles_out[-int(duration):, :].sum(axis=0)

        # if self.name == "R32":
        #     input("{}: {}".format(duration, departed))
        return departed

    def get_vehicles(self):
        return self.num_vehicles[-1]

    def get_free_flow(self, duration):
        return sum([detector.get_free_flow(duration) for detector in self.detector_in])

    def get_occupancy_rate(self, duration):
        return [detector.get_occupancy_rate(duration) for detector in self.detector_out]

    def get_average_travel_time(self, duration):
        return sum(self.travel_time)/duration

    def save(self, output_path):
        filename = "{}/modules/intersections_{}/{}.mat".format(output_path, self.intersection, self.name)
        File.save_mat(filename, {'num_vehicles': self.num_vehicles, 'num_vehicles_in': self.num_vehicles_in, 'num_vehicles_out': self.num_vehicles_out})

        if self.ground_truth is not None:
            filename = "{}/ground_truth/intersections_{}/{}.mat".format(output_path, self.intersection, self.name)
            File.save_mat(filename, {'state': self.ground_truth.state, 'flow': self.ground_truth.flow})
