from libs.etc.files import File
from keras.models import load_model


class NetworkConfig():

    def __init__(self, scenario, profile):
        self.areas = File.load_json('scenarios/{}/{}/areas.json'.format(scenario, profile))
        self.boundaries = File.load_json('scenarios/{}/{}/boundaries.json'.format(scenario, profile))
        self.intersections = File.load_json('scenarios/{}/{}/intersections.json'.format(scenario, profile))
        self.links = File.load_json('scenarios/{}/{}/links.json'.format(scenario, profile))
        self.rates = File.load_json('scenarios/{}/{}/rates.json'.format(scenario, profile))
        self.roads = File.load_json('scenarios/{}/{}/roads.json'.format(scenario, profile))
        splitting_flow = File.load_json('scenarios/{}/{}/splitting_flow.json'.format(scenario, profile))
        
        for intersection in self.intersections:
            for road in self.intersections[intersection]["incoming_roads"]:
                self.intersections[intersection]["incoming_roads"][road]["splitting_flows"] = splitting_flow[intersection][road]

            if self.intersections[intersection]["flow_model"]:
                path = "scenarios/{}/{}/flow_models/{}.h5".format(scenario, profile, intersection)
                # print(path)
                model = load_model(path, compile=False)
                self.intersections[intersection]["model"] = model

            path = "scenarios/{}/{}/estimated_b/{}.mat".format(scenario, profile, intersection)
            if File.is_exist(path):
                tmp = File.load_mat(path)
                self.intersections[intersection]["estimated_b"] = tmp["matrix_b"]
