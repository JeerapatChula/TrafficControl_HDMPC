

class Agreement():

    @staticmethod
    def get_network_area_list(network_config):
        return sorted(list(network_config.areas.keys()))

    @staticmethod
    def get_network_node_list(network_config):
        nodes = []
        areas = sorted(list(network_config.areas.keys()))
        for area in areas:
            nodes.append("{}_in".format(area))
            nodes.append("{}_out".format(area))
        return nodes

    @staticmethod
    def get_network_link_list(network_config):
        links = list(network_config.links.keys())
        return sorted(links)

    @staticmethod
    def get_adjacend_areas(area, network_config):
        return list(set(network_config.areas[area]["adjacents"]))

    @staticmethod
    def get_area_node_list(area, network_config):
        areas_configs = network_config.areas[area]
        intersection_list = areas_configs["intersections"]
        nodes = list(intersection_list)
        areas = list(network_config.areas.keys())
        for intersection in intersection_list:
            incoming_roads = network_config.intersections[intersection]["incoming_roads"]
            outcome_roads = network_config.intersections[intersection]["outcome_roads"]
            for road in incoming_roads:
                boundary = incoming_roads[road]["boundary"]
                str = "{}_out".format(boundary) if boundary not in intersection_list else boundary
                nodes.append(str)
            for road in outcome_roads:
                boundary = outcome_roads[road]["boundary"]
                str = "{}_in".format(boundary) if boundary not in intersection_list else boundary
                nodes.append(str)
        return sorted(list(set(nodes)))

    @staticmethod
    def get_area_link_list(area, network_config):
        areas_configs = network_config.areas[area]
        intersection_list = areas_configs["intersections"]
        links = []
        for intersection in intersection_list:
            incoming_roads = list(network_config.intersections[intersection]["incoming_roads"].keys())
            outcome_roads = list(network_config.intersections[intersection]["outcome_roads"].keys())
            links = list(set(links + incoming_roads + outcome_roads))
        return sorted(list(set(links)))

    # @staticmethod
    # def get_area_link_list(area, network_config):
    #     areas_configs = network_config.areas[area]
    #     intersection_list = areas_configs["intersections"]
    #     links = []
    #     for intersection in intersection_list:
    #         incoming_roads = list(network_config.intersections[intersection]["incoming_roads"].keys())
    #         outcome_roads = list(network_config.intersections[intersection]["outcome_roads"].keys())
    #         for road in incoming_roads:
    #             links.append(road)
    #         for road in outcome_roads:
    #             links.append(road)
    #     return links

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
    def get_intersection_link_list(intersection, network_config):
        intersection_configs = network_config.intersections[intersection]
        incoming_roads = list(intersection_configs["incoming_roads"].keys())
        outcome_roads = list(intersection_configs["outcome_roads"].keys())
        links = incoming_roads
        for road in outcome_roads:
            links.append(road)
        return links

    @staticmethod
    def get_intersection_links_is_incomming(intersection, network_config):
        intersection_configs = network_config.intersections[intersection]
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
