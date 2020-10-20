from libs.etc.files import File


class Boundary():
    def __init__(self, intermediary, name, boundary_configs):
        self.name = name
        self.source = 0
        self.sink = 0
        self.arrival_rate = boundary_configs["arrival_rate"]/3600.0
        self.departure_rate = boundary_configs["departure_rate"]/3600.0
        self.road_in_conf = boundary_configs["road_in"]
        self.road_out_conf = boundary_configs["road_out"]

        self.source_log = []
        self.sink_log = []
        self.arrived_log = []
        self.departured_log = []

        self.socket = intermediary.client_register(name, "module_boundary")
        self.socket.add_endpoint("get_data", self.get_data)

    def update(self):
        self.put_flow()
        self.source += self.arrival_rate
        self.sink += self.departure_rate
        self.source_log.append(self.source)
        self.sink_log.append(self.sink)

    def put_flow(self):
        arrived = self.socket.post(self.road_in_conf["intersection"], "module_intersection", "get_road_traffic_passed", {"road": self.road_in_conf["name"], "duration": 15})
        departured = self.socket.post(self.road_out_conf["intersection"], "module_intersection", "get_road_traffic_passed", {"road": self.road_out_conf["name"], "duration": 15})
        self.source = max(self.source - self.road_in_conf["weigth"]*arrived, 0)
        self.sink = max(self.sink - self.road_out_conf["weigth"]*departured, 0)
        self.arrived_log.append(arrived)
        self.departured_log.append(departured)

    def get_data(self, data):
        data = {}
        data["source"] = self.source
        data["sink"] = self.sink
        data["arrival_rate"] = self.arrival_rate
        data["departure_rate"] = self.departure_rate
        return data

    def save(self, output_path):
        filename = "{}/modules/boundaries/{}.mat".format(output_path, self.name)
        File.save_mat(filename, {'source_log': self.source_log, 'sink_log':self.sink_log, 'arrived_log': self.arrived_log, 'departured_log': self.departured_log})
