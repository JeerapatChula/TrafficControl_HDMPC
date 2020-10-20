from .roads import Road
from .splitting_flow import SplittingFlow
# from .ground_truth_flows import GroundTruthFlow
from libs.etc.files import File
import traci


class Intersection():

    def __init__(self, intermediary, name, intersection_configs):
        self.name = name
        self.junction_id = intersection_configs["junction_id"]
        self.default_signals = intersection_configs["default_signals"]
        self.trafficlight_default = intersection_configs["trafficlight_default"]
        self.control_signals = self.default_signals
        self.old_signal = [0]*len(self.control_signals)

        self.log_phases = []
        self.log_duratoins = []
        self.log_cycle_duratoins = []

        self.time = traci.simulation.getTime()

        self.current_duration = 0
        self.previous_duration = 0
        self.current_phase = -1
        self.previous_phase = len(self.trafficlight_default)
        self.new_phase_triger = False
        self.new_cycle_triger = False
        self.is_controlled = False
        self.is_observed = False

        self.incoming_road_list = list(intersection_configs["incoming_roads"].keys())
        self.outcome_road_list = list(intersection_configs["outcome_roads"].keys())

        self.splitting_flow = SplittingFlow(name, intersection_configs)
        # self.ground_truth_flow = GroundTruthFlow(intersection_configs)
        self.incoming_roads = [Road(name, road, intersection_configs["incoming_roads"][road], intermediary) for road in intersection_configs["incoming_roads"]]
        self.outcome_roads = [Road(name, road, intersection_configs["outcome_roads"][road], intermediary) for road in intersection_configs["outcome_roads"]]
        [road.create_ground_truth(intersection_configs) for road in self.incoming_roads]

        self.socket = intermediary.client_register(name, "module_intersection")
        self.socket.add_endpoint("set_controlled", self.set_controlled)
        self.socket.add_endpoint("get_phase", self.get_phase)
        self.socket.add_endpoint("set_phase", self.set_phase)
        self.socket.add_endpoint("get_splitting_flow", self.get_splitting_flow)
        self.socket.add_endpoint("get_performce_data", self.get_performce_data)
        self.socket.add_endpoint("get_road_traffic_passed", self.get_road_traffic_passed)

    def update(self):
        [road.update() for road in self.incoming_roads]
        [road.update() for road in self.outcome_roads]
        self.update_phase()
        self.update_signal()
        self.update_datalog()

    def update_signal(self):
        if self.is_controlled and self.new_phase_triger:
            # traci.trafficlight.setPhaseDuration(self.junction_id, 10*self.current_phase)
            traci.trafficlight.setPhaseDuration(self.junction_id, self.control_signals[self.current_phase])

    def update_phase(self):
        self.current_phase, self.current_duration = self.interface_get_trafficlights()
        self.new_phase_triger = False
        self.new_cycle_triger = False

        if self.current_phase != self.previous_phase:
            self.new_phase_triger = True
            if self.previous_phase != len(self.trafficlight_default):
                new_time = traci.simulation.getTime()
                self.previous_duration = int(new_time - self.time)
                self.time = new_time
                self.old_signal[self.current_phase] = self.current_duration
                actaul_departed = [road.get_last_actual_departed() for road in self.incoming_roads]
                departed = [road.get_vehicles_out(self.previous_duration) for road in self.incoming_roads]
                arrived = [road.get_vehicles_in(self.previous_duration).sum() for road in self.outcome_roads]
                self.splitting_flow.completed_phase(actaul_departed, departed, arrived, self.previous_phase, self.previous_duration)
                [road.update_ground_truth(self.current_phase) for road in self.incoming_roads]
                # self.ground_truth_flow.update(self.current_phase)

            if self.current_phase < self.previous_phase:
                self.new_cycle_triger = True
                self.cycle_trigger()
                self.old_signal = [0]*len(self.control_signals)

        if self.previous_phase != len(self.trafficlight_default) and self.new_phase_triger and not self.new_cycle_triger:
            self.phase_trigger()

        self.previous_phase, self.previous_duration = self.current_phase, self.current_duration

    def update_datalog(self):

        if self.new_cycle_triger:
            self.log_cycle_duratoins.append([0]*len(self.trafficlight_default))

        if self.new_phase_triger:
            self.log_phases.append(self.current_phase)
            self.log_duratoins.append(self.current_duration)
            self.log_cycle_duratoins[-1][self.current_phase] = self.current_duration

    def interface_get_trafficlights(self):
        tmp = traci.trafficlight.getRedYellowGreenState(self.junction_id)
        phase = self.trafficlight_default.index(tmp) if tmp in self.trafficlight_default else 0
        return (phase, traci.trafficlight.getPhaseDuration(self.junction_id))

    def interface_set_trafficlights(self, phase, duration):
        traci.trafficlight.setRedYellowGreenState(self.junction_id, self.trafficlight_default[phase])
        traci.trafficlight.setPhaseDuration(self.junction_id, duration)

    def set_controlled(self, data):
        self.is_observed = True
        self.is_controlled = data["controlled"]
        self.controller_cluster = data["cluster"]
        self.controller_name = data["name"]
        self.controller_phase_enpoint = data["phase_enpoint"]
        self.controller_cycle_enpoint = data["cycle_enpoint"]
        if self.is_controlled:
            self.set_new_program()

    def set_new_program(self):
        phases = []
        for (duration, state) in zip(self.control_signals, self.trafficlight_default):
            phases.append(traci.trafficlight.Phase(duration, state))
        logic = traci.trafficlight.Logic("controlled_program", 0, 0, phases=phases)
        traci.trafficlight.setCompleteRedYellowGreenDefinition(self.junction_id, logic)

    def phase_trigger(self):
        if self.is_observed:
            data = self.get_phase_summarized_data()
            self.socket.post(self.controller_name, self.controller_cluster, self.controller_phase_enpoint, data)

    def cycle_trigger(self):
        if self.is_observed:
            data = self.get_phase_summarized_data()
            data["cycle_length"] = sum(self.control_signals)
            data["old_signal"] = sum(self.old_signal)
            feedback = self.socket.post(self.controller_name, self.controller_cluster, self.controller_cycle_enpoint, data)
            if self.is_controlled:
                for x in range(len(self.control_signals)):
                    self.control_signals[x] = feedback["control_signal"][x]

    def get_phase(self, data):
        return self.current_phase

    def set_phase(self, data):
        self.interface_set_trafficlights(data["phase"], data["duration"])
        return True

    def get_road_traffic_passed(self, data):
        road = data["road"]
        duration = data["duration"]
        if road in self.incoming_road_list:
            ind = self.incoming_road_list.index(road)
            data = self.incoming_roads[ind].get_vehicles_in(duration).sum()
        elif road in self.outcome_road_list:
            ind = self.outcome_road_list.index(road)
            data = self.outcome_roads[ind].get_vehicles_in(duration).sum()
        return data

    def get_splitting_flow(self, data):
        return self.splitting_flow.get_summarize()

    def get_phase_summarized_data(self):
        data = {"duration": self.previous_duration, "incoming_roads": {}, "outcome_roads": {}}
        for road in self.incoming_roads: # Revise here
            data["incoming_roads"][road.name] = {}
            data["incoming_roads"][road.name]["arrived"] = road.get_vehicles_in(self.previous_duration).sum()
            data["incoming_roads"][road.name]["departed"] = self.splitting_flow.last_departed[road.name]
            data["incoming_roads"][road.name]["free_flow"] = road.get_free_flow(sum(self.control_signals))
            data["incoming_roads"][road.name]["occupancy_rate"] = road.get_occupancy_rate(18000) # 5 minute = 5*60 seconds
            data["incoming_roads"][road.name]["num_vehicles"] = road.get_vehicles()
            data["incoming_roads"][road.name]["actual_state"] = road.get_actual_state()
            # input(data["incoming_roads"][road.name]["free_flow"])

        for road in self.outcome_roads:
            data["outcome_roads"][road.name] = {}
            data["outcome_roads"][road.name]["arrived"] = road.get_vehicles_in(self.previous_duration).sum()
            data["outcome_roads"][road.name]["free_flow"] = road.get_free_flow(sum(self.control_signals))
            data["outcome_roads"][road.name]["num_vehicles"] = road.get_vehicles()

        return data

    def get_performce_data(self, data):
        duration = data["duration"]
        results = {}
        results["splitting_flow"] = self.splitting_flow.get_summarize()
        results["average_flow"] = 3600*sum([road.get_vehicles_out(-1).sum() for road in self.incoming_roads])/duration
        results["average_travel_time"] = sum([road.get_average_travel_time(duration) for road in self.incoming_roads])/len(self.incoming_roads)
        return results

    def save(self, output_path):
        self.splitting_flow.save(output_path)
        [road.save(output_path) for road in self.incoming_roads]
        [road.save(output_path) for road in self.outcome_roads]
        filename = "{}/modules/intersections/{}.mat".format(output_path, self.name)
        File.save_mat(filename, {'log_phases': self.log_phases, 'log_duratoins': self.log_duratoins, 'log_cycle_duratoins': self.log_cycle_duratoins, 'matrix_b': self.splitting_flow.get_matrix_b()})
