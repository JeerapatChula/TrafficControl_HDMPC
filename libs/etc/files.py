import json
import os
import scipy.io


class File():

    @staticmethod
    def save_mat(filename, data):
        path = os.path.dirname(filename)
        File.create_path(path)
        scipy.io.savemat(filename, mdict=data)

    @staticmethod
    def load_mat(filename):
        return scipy.io.loadmat(filename)

    @staticmethod
    def create_path(path):
        if not os.path.exists(path) and len(path) > 0:
            os.makedirs(path)

    @staticmethod
    def save_json(filename, data):
        path = os.path.dirname(filename)
        File.create_path(path)

        with open(filename, 'w') as outfile:
            str_ = json.dumps(data, indent=4, sort_keys=True)
            outfile.write(str_)

    @staticmethod
    def load_json(filename):
        true = True
        false = False
        if os.path.exists(filename):
            with open(filename, 'r') as myfile:
                data = myfile.read()
                return eval(data)
        return {}

    @staticmethod
    def is_exist(filename):
        return os.path.exists(filename)
