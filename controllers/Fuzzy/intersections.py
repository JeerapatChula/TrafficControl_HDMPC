from libs.etc.files import File
import numpy as np
import warnings

warnings.filterwarnings("ignore", message="numpy.dtype size changed")
warnings.filterwarnings("ignore", message="numpy.ufunc size changed")


class Intersection():

    def __init__(self, intermediary, name, network_config):
        self.name = name
        self.intersection_configs = network_config.intersections[name]
        self.is_controlled = self.intersection_configs["controlled"]
        self.num_phases = len(self.intersection_configs["default_signals"])
        self.phases = [Phase(phase, self.intersection_configs) for phase in range(self.num_phases)]

        self.socket = intermediary.client_register(name, "controller")
        self.socket.add_endpoint("phase_completed_signal", self.phase_completed_signal)
        self.socket.add_endpoint("cycle_completed_signal", self.cycle_completed_signal)

        data = {"cluster": "controller",
        "name": name,
        "phase_enpoint": "phase_completed_signal",
        "cycle_enpoint": "cycle_completed_signal",
        "controlled": self.is_controlled}

        self.socket.post(name, "module_intersection", "set_controlled", data)

    def update(self):
        pass

    def phase_completed_signal(self, data):
        # input(data)
        return None

    def cycle_completed_signal(self, data):
        road_occupancy_rate = {}
        for road in data['incoming_roads']:
            road_occupancy_rate[road] = {}
            road_occupancy_rate[road] = data['incoming_roads'][road]["occupancy_rate"]
        phase_occupancy_rate = self.get_phase_occupancy_rate(road_occupancy_rate)
        signal = [0]*self.num_phases
        for phase in range(self.num_phases):
            self.phases[phase].update_cycle(phase_occupancy_rate[phase])
            signal[phase] = self.phases[phase].g
        if sum(signal) > 200:
            signal = 200*(signal/sum(signal))
        # input(signal)
        return {"control_signal": signal}

    def get_phase_occupancy_rate(self, road_occupancy_rate):
        phase_occupancy_rate = [0] * self.num_phases
        for phase in range(self.num_phases):
            num_routes = len(self.intersection_configs["available_lanes"][phase])
            for route in self.intersection_configs["available_lanes"][phase]:
                dst = self.intersection_configs["incoming_roads"][route[0]]["dst_roads"].index(route[1])
                t = 0
                for sen in self.intersection_configs["incoming_roads"][route[0]]["detector_out_dst_roads"]:
                    if dst in sen:
                        phase_occupancy_rate[phase] += road_occupancy_rate[route[0]][t]
                        t += 1
            phase_occupancy_rate[phase] = phase_occupancy_rate[phase]/num_routes
        return phase_occupancy_rate



class Phase():
    def __init__(self, phase, intersection_configs):
        self.phase = phase
        self.DS = [0,0,0]                                        # Degree of saturation
        self.g = intersection_configs["default_signals"][phase]  # Allocated Green Time (sec.)
        self.n = 0                                               # Number of vehicles (pcu.)
        self.e = 0                                               # Actual space (sec.)
        self.t = 0                                               # Unavoidable space (sec.) = (3600/max.Flow)-(occupancy at max. flow/100)
        self.max_flow = 0

        self.OOC = 0                                             # Occupancy ratio
        self.dDs_mod = [0,0,0]                                   # Modified degree of saturation
        self.dDS = [0,0,0]                                       # degree of saturation

    def update_cycle(self, phase_occupancy_rate):

        self.DS = self.DS[1:] + [1 - phase_occupancy_rate]
        self.dDS = self.dDS[1:] + [phase_occupancy_rate - self.DS[1]]
        self.g = self.fuzzy_control()

    def fuzzy_control(self):
        DS = sum(self.DS)/3
        dDS = sum(self.dDS)/3

        DS_low, DS_medium, DS_high = self.fuzzy(DS, 0.3, 0.4, 0.5, 0.6)
        dDS_decrease, dDS_same, dDS_increase = self.fuzzy(dDS, 0.35, 0.45, 0.55, 0.65)

        CR_decrease_1 = min(DS_low, dDS_decrease)
        CR_decrease_2 = min(DS_low, dDS_same)
        CR_decrease_3 = min(DS_medium, dDS_decrease)
        CR_decrease = max(CR_decrease_1, CR_decrease_2, CR_decrease_3)

        CR_same_1 = min(DS_low, dDS_increase)
        CR_same_2 = min(DS_medium, dDS_same)
        CR_same_3 = min(DS_high, dDS_decrease)
        CR_same = max(CR_same_1, CR_same_2, CR_same_3)

        CR_increase_1 = min(DS_medium, dDS_increase)
        CR_increase_2 = min(DS_high, dDS_same)
        CR_increase_3 = min(DS_high, dDS_increase)
        CR_increase = max(CR_increase_1, CR_increase_2, CR_increase_3)
        CR = (CR_decrease*0.5 + CR_same + CR_increase*1.5)/(CR_decrease + CR_same + CR_increase)
        return CR*self.g


    def fuzzy(self, x, a, b, c, d):
        rst_1 = 0 if x > b else (1 if x < a else 1-((x-a)/(b-a)))
        rst_2 = 0 if (x < a or x > d) else ((x-a)/(b-a) if x < b else (1-((x-c)/(d-c)) if x > c else 1))
        rst_3 = 0 if x < c else (1 if x > d else (x-c)/(d-c))
        return rst_1, rst_2, rst_3
