

class Client():

    def __init__(self, client_name, cluster, sever_post):
        self.client_name = client_name
        self.end_points = {}
        self.sever_post = sever_post

    def add_endpoint(self, endpoint_name, endpoint):
        self.end_points[endpoint_name] = endpoint

    def post(self, dst, cluster, endpoint_name, data):
        return self.sever_post(self.client_name, dst, cluster, endpoint_name, data)


class Intermediary():

    def __init__(self):
        self.clusters = {}

    def client_register(self, client_name, cluster):
        if cluster not in self.clusters:
            self.clusters[cluster] = {}

        if client_name not in self.clusters[cluster]:
            self.clusters[cluster][client_name] = Client(client_name, cluster, self.sever_post)

        return self.clusters[cluster][client_name]

    def sever_post(self, src, dst, cluster, endpoint_name, data):
        if cluster in self.clusters:
            if dst in self.clusters[cluster]:
                if endpoint_name in self.clusters[cluster][dst].end_points:
                    endpoint = self.clusters[cluster][dst].end_points[endpoint_name]
                    return endpoint(data)
        return None
