from libs.etc.files import File
from .model import Model


class IntersectionFlowModel():

    def __init__(self, intersection, intersection_configs):
        self.intersection = intersection
        self.is_modelled = intersection_configs["flow_model"]
        if self.is_modelled:
            self.model = intersection_configs["model"]
        self.num_veh = []
        self.num_veh_old = []
        self.mane_veh = []
        self.upper_data = []
        self.old_traffic_signal = []
        self.old_flow = []
        self.old_vector = []

    def put_model(self, model):
        self.model = model
        self.is_modelled = True

    def predict(self):
        if self.is_modelled:
            input_data = Model.get_input_vector(self.num_veh[-1], self.mane_veh[-1], self.upper_data[-1], self.old_traffic_signal[-1], self.old_flow[-1])
            # input_data, self.old_vector = Model.get_input_vector_series(self.num_veh[-1], self.mane_veh[-1], self.upper_data[-1], self.old_traffic_signal[-1], self.old_flow[-1], self.old_vector)
            pred = self.model.predict(input_data).T
            return abs(pred)
        else:
            return self.old_flow[-1]

    def put_data(self, num_veh, mane_veh, upper_data, old_traffic_signal, old_flow):
        if self.num_veh_old == []:
            self.num_veh_old.append((num_veh.T).tolist()[0])
        else:
            self.num_veh_old.append(self.num_veh[-1])
        self.num_veh.append((num_veh.T).tolist()[0])
        self.mane_veh.append((mane_veh.T).tolist()[0])
        self.upper_data.append(abs(upper_data.T).tolist()[0])
        self.old_traffic_signal.append(old_traffic_signal.tolist())
        self.old_flow.append(abs(old_flow.T).tolist()[0])

    def save(self, output_path):
        filename = "{}/ai/{}.mat".format(output_path, self.intersection)
        data = {
            "num_veh_old": self.num_veh_old,
            "num_veh": self.num_veh,
            "mane_veh": self.mane_veh,
            "upper_data": self.upper_data,
            "old_traffic_signal": self.old_traffic_signal,
            "old_flow": self.old_flow,
            }
        File.save_mat(filename, data)
