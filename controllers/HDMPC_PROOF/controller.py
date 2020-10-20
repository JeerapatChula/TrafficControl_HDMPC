from .areas import Area
from .intersections import Intersection
from .networks import Network


class Controller():

    def __init__(self, intermediary, controller_config, network_config):

        # roads_config = network_config.roads
        intersections_config = network_config.intersections
        areas_config = network_config.areas

        self.network = Network(intermediary, controller_config, network_config)
        self.areas = [Area(intermediary, x, controller_config, network_config) for x in areas_config]
        self.intersections = [Intersection(intermediary, x, controller_config, network_config) for x in intersections_config]

        self.network_rate = network_config.rates["network"]
        self.area_rate = network_config.rates["area"]
        self.intersection_rate = network_config.rates["intersection"]

        self.intersection_update_time = 1
        self.area_update_time = 1
        self.network_update_time = 1

    def update(self):

        self.network_update_time -= 1
        self.area_update_time -= 1
        self.intersection_update_time -= 1

        if self.network_update_time == 0:
            self.network_update_time = self.network_rate
            self.network.update()

        if self.area_update_time == 0:
            self.area_update_time = self.area_rate
            for area in self.areas:
                area.update()

        if self.intersection_update_time == 0:
            self.intersection_update_time = self.intersection_rate
            for intersection in self.intersections:
                intersection.update()

    def save(self, output_path):
        [intersection.save(output_path) for intersection in self.intersections]
        [area.save(output_path) for area in self.areas]
        self.network.save(output_path)
