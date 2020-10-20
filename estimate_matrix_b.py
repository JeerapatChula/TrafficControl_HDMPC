#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
sys.path.append(os.path.abspath(os.path.join('..')))
import numpy as np
from libs.etc.files import File



if __name__ == "__main__":
    scenario = "sathorn_full_junctions_morning_100"
    profile = "b_1"

    controllers = ["HDMPC_Np1", "HDMPC_Np2", "HDMPC_Np3"]
    # intersection = "J{}".format(int(input("Intersection : ")))

    gamma_step = 3000
    gamma_start = 0
    gamma_stop = 12000

    start_trial = 1
    stop_trial = 5

    for i in range(10):
        intersection = "J{}".format(i+1)
        input_vector = None
        output_vector = None
        tmp_matrix_b = None
        N = 0
        for trial in range(start_trial, stop_trial+1):
            for controller in controllers:
                for gamma in range(gamma_start, gamma_stop+1, gamma_step):
                    path = "{}/sumo_outputs/trial_{}/{}/{}/{}/{}/modules/intersections/{}.mat".format(os.environ["HOME"], trial, scenario, profile, controller, gamma, intersection)
                    tmp = File.load_mat(path)
                    tmp_matrix_b = tmp_matrix_b + tmp["matrix_b"] if tmp_matrix_b is not None else tmp["matrix_b"]
                    N += 1
        tmp_matrix_b = tmp_matrix_b / N
        path = "estimated_model/{}/{}.mat".format(scenario, intersection)
        File.save_mat(path, {"matrix_b": tmp_matrix_b})
        # print(intersection)
