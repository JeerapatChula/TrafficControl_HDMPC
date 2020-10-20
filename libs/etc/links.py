

class Link():

    @staticmethod
    def get_incoming_roads(node, roads):
        tmp = []
        for road in roads:
            if roads[road]["sink_node"] == node:
                tmp.append(road)
        return tmp

    @staticmethod
    def get_outcome_roads(node, roads):
        tmp = []
        for road in roads:
            if roads[road]["source_node"] == node:
                tmp.append(road)
        return tmp
