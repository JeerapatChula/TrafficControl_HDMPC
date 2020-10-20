from modules.boundaries import Boundary
from modules.intersections import Intersection
from modules.vehicle_routes import VehicleRoute


class ModuleManager():
    def __init__(self, intermediary, network_configs):
        self.boundaries = self.load_boundaries(intermediary, network_configs.boundaries)
        self.intersections = self.load_intersections(intermediary, network_configs.intersections)
        VehicleRoute(intermediary, 100000)

        self.boundary_update_time = 15

    def update(self):
        self.boundary_update_time -= 1
        if self.boundary_update_time == 0:
            self.boundary_update_time = 15
            for x in self.boundaries:
                self.boundaries[x].update()

        for x in self.intersections:
            self.intersections[x].update()

    def load_boundaries(self, intermediary, boundaries):
        tmp = {}
        for boundary in boundaries:
            tmp[boundary] = Boundary(intermediary, boundary, boundaries[boundary])
        return tmp

    def load_intersections(self, intermediary, intersections):
        tmp = {}
        for intersection in intersections:
            tmp[intersection] = Intersection(intermediary, intersection, intersections[intersection])
        return tmp

    def save(self, output_path):
        for x in self.boundaries:
            self.boundaries[x].save(output_path)

        for x in self.intersections:
            self.intersections[x].save(output_path)
