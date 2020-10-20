from libs.etc.files import File


class PerformanceMeasurement():

    def __init__(self, intermediary, network_config, sim_length):
        self.sim_length = sim_length
        self.intersections = list(network_config.intersections.keys())
        self.socket = intermediary.client_register("performance_observer", "server")


    def summarize(self, simulation_options):
        self.total_average_flow = 0
        self.total_average_travel_time = 0
        path = "{}/{}/".format(simulation_options["scenario"], simulation_options["profile"]["name"])

        for intersection in self.intersections:
            results = self.socket.post(intersection, "module_intersection", "get_performce_data", {"duration": self.sim_length})
            self.put_data(simulation_options, "{}/{}/average_flow.json".format(path, intersection), results["average_flow"])
            self.put_data(simulation_options, "{}/{}/average_travel_time.json".format(path, intersection), results["average_travel_time"])
            self.put_data(simulation_options, "{}/{}/splitting_flow.json".format(path, intersection), results["splitting_flow"])
            self.total_average_flow += results["average_flow"]
            self.total_average_travel_time += results["average_travel_time"]

        print(f"\nFlow: {self.total_average_flow}")
        print(f"Travel time: {self.total_average_travel_time/len(self.intersections)}")
        self.put_data(simulation_options, "{}/total_average_flow.json".format(path), self.total_average_flow)
        self.put_data(simulation_options, "{}/total_average_travel_time.json".format(path), self.total_average_travel_time/len(self.intersections))

    def put_data(self, simulation_options, file_name, data):
        self.load(file_name)
        trial = "trial_{}".format(simulation_options["trial"])
        controller = simulation_options["controller"]

        if trial not in self.performances:
            self.performances[trial] = {}

        if controller not in self.performances[trial]:
            self.performances[trial][controller] = {}

        if "gamma" in simulation_options:
            gamma = "gamma_{}".format(simulation_options["gamma"])
            self.performances[trial][controller][gamma] = data
        else:
            self.performances[trial][controller] = data

        self.save(file_name)

    def load(self, file_name):
        filename = "results/{}".format(file_name)
        self.performances = File.load_json(filename)

    def save(self, file_name):
        filename = "results/{}".format(file_name)
        File.save_json(filename, self.performances)
