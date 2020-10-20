from libs.etc.files import File
import traci


class Detector():

    def __init__(self, detector_name, intermediary):

        self.detector_name = detector_name
        self.last_veh_id = ''
        self.count_veh = []
        self.occupancy = []
        self.passed_veh_ids = []
        self.passed_veh_routes = []
        self.count_veh_total = 0
        self.total = 0
        self.socket = intermediary.client_register(detector_name, "module_detector")

    def update(self):
        new_veh_id = traci.inductionloop.getLastStepVehicleIDs(self.detector_name)
        new_veh_id = new_veh_id[0] if len(new_veh_id) != 0 else ''
        occupancy = 1 if len(new_veh_id) != 0 else 0
        new_veh = 0
        self.total = self.total + 1

        if len(new_veh_id) != 0 and new_veh_id != self.last_veh_id:
            self.rising_edge(new_veh_id)

        if len(self.last_veh_id) != 0 and new_veh_id != self.last_veh_id:
            new_veh = 1
            self.count_veh_total += 1
            self.falling_edge(self.last_veh_id)

        self.count_veh.append(new_veh)
        self.occupancy.append(occupancy)
        self.last_veh_id = new_veh_id
        # if new_veh_id == '311.6':
            # tmp1 = traci.inductionloop.getVehicleData(self.detector_name)
        # tmp2 = traci.inductionloop.getTimeSinceDetection(self.detector_name)
        # if len(new_veh_id) != 0 and tmp2 > 0.40:
            # print(int(float(new_veh_id)*100))
            # input((tmp1, tmp2))
        # self.last_veh_route = traci.vehicle.getRoute(new_veh_id) if len(new_veh_id) != 0 else None

    def rising_edge(self, veh_id):
        self.socket.post("vehicle_route", "module_dummy", "put_vehicle_id", {"veh_ids": [veh_id]})

    def falling_edge(self, veh_id):
        # route = self.last_veh_route
        # self.passed_veh_routes.append(route)
        self.passed_veh_ids.append(veh_id)

    def get_occupancy_rate(self, duration):
        n = min(self.total, int(duration))
        return sum(self.occupancy[-n:])/n

    def get_veh_passed(self, duration):
        n = min(self.total, int(duration))
        return sum(self.count_veh[-n:])

    def get_total_veh_passed(self):
        return self.count_veh_total

    def get_vehicles(self):
        return self.num_vehicles[-1]

    def get_free_flow(self, duration):
        n = min(self.total, int(duration))
        passed = sum(self.count_veh[-n:])
        occuped = sum(self.occupancy[-n:])
        return passed/(duration - occuped) if (duration - occuped) > 0 else 0

    def save(self, output_path):
        filename = "{}/modules/detectors/{}.mat".format(output_path, self.detector_name)
        File.save_mat(filename, {'count_veh': self.count_veh, 'count_veh_total':self.count_veh_total})

    def reset(self, data):
        self.last_veh_id = ''
        self.count_veh = []
        self.count_veh_total = 0
