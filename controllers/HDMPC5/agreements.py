import copy


class Agreement():

    @staticmethod
    def get_network_area_list(network_config):
        return sorted(list(network_config.areas.keys()))

    @staticmethod
    def get_network_boundary_list(network_config):
        return sorted(list(network_config.boundaries.keys()))

    @staticmethod
    def get_network_node_list(network_config):
        return sorted(list(network_config.areas.keys()))

    @staticmethod
    def get_network_link_list(network_config):
        links = list(network_config.links.keys())
        return sorted(links)

    @staticmethod
    def get_adjacend_areas(area, network_config):
        return list(set(network_config.areas[area]["adjacents"]))

    @staticmethod
    def get_area_boundary_list(area, boundaries_conf):
        boundaries = []
        for boundary in list(boundaries_conf.keys()):
            if boundaries_conf[boundary]["area"] == area:
                boundaries.append(boundary)
        return boundaries

    @staticmethod
    def get_area_intersection_list(area, network_config):
        return sorted(network_config.areas[area]["intersections"])

    @staticmethod
    def get_area_node_list(area, network_config):
        areas_configs = copy.copy(network_config.areas[area])
        boundary_list = Agreement.get_area_boundary_list(area, network_config.boundaries)
        intersection_list = copy.copy(areas_configs["intersections"])
        nodes = intersection_list
        for boundary in boundary_list:
            nodes.append("{}_out".format(boundary))
            nodes.append("{}_in".format(boundary))
        nodes = sorted(list(set(nodes)))
        return nodes

    @staticmethod
    def get_area_road_list(area, network_config):
        areas_configs = copy.copy(network_config.areas[area])
        intersection_list = copy.copy(areas_configs["intersections"])
        links = []
        for intersection in intersection_list:
            incoming_roads = list(network_config.intersections[intersection]["incoming_roads"].keys())
            outcome_roads = list(network_config.intersections[intersection]["outcome_roads"].keys())
            links = list(set(links + incoming_roads + outcome_roads))
        return sorted(list(set(links)))

    @staticmethod
    def get_area_link_list(area, network_config):
        links = []
        for link in network_config.links:
            if area == network_config.links[link]["source_node"] or area == network_config.links[link]["sink_node"]:
                links.append(link)
        return sorted(list(set(links)))

    @staticmethod
    def get_area_outcome_roads_list(area, network_config):
        roads = []
        for link in network_config.links:
            if area == network_config.links[link]["source_node"]:
                roads = list(set(roads + network_config.links[link]["roads"]))
        return sorted(list(set(roads)))

    @staticmethod
    def get_area_outcome_links_list(area, network_config):
        links = []
        for link in network_config.links:
            if area == network_config.links[link]["source_node"]:
                links.append(link)
        return sorted(list(set(links)))

    @staticmethod
    def get_intersection_road_list(intersection, network_config):
        intersection_configs = copy.copy(network_config.intersections[intersection])
        incoming_roads = list(intersection_configs["incoming_roads"].keys())
        outcome_roads = list(intersection_configs["outcome_roads"].keys())
        links = incoming_roads
        for road in outcome_roads:
            links.append(road)
        return links

    @staticmethod
    def get_intersection_links_is_incomming(intersection, network_config):
        intersection_configs = copy.copy(network_config.intersections[intersection])
        incoming_roads = list(intersection_configs["incoming_roads"].keys())
        outcome_roads = list(intersection_configs["outcome_roads"].keys())
        links_is_incomming = [True]*len(incoming_roads)
        for road in outcome_roads:
            links_is_incomming.append(False)
        return links_is_incomming

    @staticmethod
    def get_intersection_area(intersection, network_config):
        for area in network_config.areas:
            if intersection in network_config.areas[area]["intersections"]:
                return area
        return None
