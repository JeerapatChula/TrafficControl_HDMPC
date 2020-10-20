from .intersections import Intersection


class Controller():

    def __init__(self, intermediary, network_config):
        intersections_config = network_config.intersections
        self.intersections = [Intersection(intermediary, x, network_config) for x in intersections_config]

    def update(self):
        pass

    def save(self, output_path):
        pass
