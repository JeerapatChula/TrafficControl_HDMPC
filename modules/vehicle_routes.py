import traci


class VehicleRoute():
    def __init__(self, intermediary, max_vehicle_id):
        self.routes = [None]*max_vehicle_id
        self.socket = intermediary.client_register("vehicle_route", "module_dummy")
        self.socket.add_endpoint("get_vehicle_route", self.get_vehicle_route)
        self.socket.add_endpoint("put_vehicle_id", self.put_vehicle_id)

    def get_vehicle_route(self, data):
        routes = []
        for id in data["veh_ids"]:
            veh_ind = int(float(id)*100)
            if veh_ind > len(self.routes):
                input("id: {}".format(veh_ind))
            if self.routes[veh_ind] is None:
                self.routes[veh_ind] = traci.vehicle.getRoute(id)
            routes.append(self.routes[veh_ind])
        return {"routes": routes}

    def put_vehicle_id(self, data):
        for id in data["veh_ids"]:
            veh_ind = int(float(id)*100)
            if veh_ind > len(self.routes):
                input("id: {}".format(veh_ind))
            if self.routes[veh_ind] is None:
                self.routes[veh_ind] = traci.vehicle.getRoute(id)
        return None
