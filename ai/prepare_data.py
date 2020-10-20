#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
sys.path.append(os.path.abspath(os.path.join('..')))
from model import Model
import numpy as np
from libs.etc.files import File



if __name__ == "__main__":
    scenario = "sathorn_full_junctions_morning_100"
    profiles = {}
    case = ["case_1", "case_3"]
    profiles["J1"] = case
    profiles["J2"] = case
    profiles["J3"] = case
    profiles["J4"] = case
    profiles["J5"] = case
    profiles["J6"] = case
    profiles["J7"] = case
    profiles["J8"] = case
    profiles["J9"] = case
    profiles["J10"] = case

    controllers = ["HDMPC_Np1", "HDMPC_Np2", "HDMPC_Np3"]
    # intersection = "J{}".format(int(input("Intersection : ")))

    gamma_step = 1000
    gamma_start = 0
    gamma_stop = 20000

    start_trial = 1
    stop_trial = 5

    # for i in range(10):
        # intersection = "J{}".format(i+1)
        # input_vector = None
        # output_vector = None
        # for trial in range(start_trial, stop_trial+1):
            # for profile in profiles[intersection]:
                # for controller in controllers:
                    # for gamma in range(gamma_start, gamma_stop+1, gamma_step):
                        # path = "{}/sumo_outputs/trial_{}/{}/{}/{}/{}/ai/{}.mat".format(os.environ["HOME"], trial, scenario, profile, controller, gamma, intersection)
                        # tmp = File.load_mat(path)
                        # for i in range(1, len(tmp["old_flow"])):
                            # tmp_input = Model.get_input_vector(tmp["num_veh"][i-1], tmp["mane_veh"][i-1], tmp["upper_data"][i-1], tmp["old_traffic_signal"][i-1], tmp["old_flow"][i-1])
                            # tmp_output = np.array(tmp["old_flow"][i]).T

                            # input_vector = tmp_input if input_vector is None else np.vstack((input_vector, tmp_input))
                            # output_vector = tmp_output if output_vector is None else np.vstack((output_vector, tmp_output))
        # path = "data_sets/{}/{}.mat".format(scenario, intersection)
        # File.save_mat(path, {"input_vector": input_vector, "output_vector": output_vector})
        # print(intersection)

    limit = 4

    for i in range(10):
        intersection = "J{}".format(i+1)
        input_vector = []
        output_vector = []
        for trial in range(start_trial, stop_trial+1):
            for profile in profiles[intersection]:
                for controller in controllers:
                    for gamma in range(gamma_start, gamma_stop+1, gamma_step):
                        path = "{}/sumo_outputs/trial_{}/{}/{}/{}/{}/ai/{}.mat".format(os.environ["HOME"], trial, scenario, profile, controller, gamma, intersection)
                        tmp = File.load_mat(path)

                        input_vec = []
                        # output_vec = [0] * limit

                        old_vector = []

                        for i in range(1, len(tmp["old_flow"])):
                            tmp_input, old_vector = Model.get_input_vector_series(tmp["num_veh"][i-1], tmp["mane_veh"][i-1], tmp["upper_data"][i-1], tmp["old_traffic_signal"][i-1], tmp["old_flow"][i-1], old_vector)
                            tmp_output = np.array(tmp["old_flow"][i]).T
                            # output_vec.append(tmp_output)

                            # if len(output_vec) > limit:
                                # output_vec.pop(0)

                            input_vector.append(tmp_input)
                            output_vector.append(tmp_output)
        path = "data_sets/{}/{}.mat".format(scenario, intersection)
        File.save_mat(path, {"input_vector": input_vector, "output_vector": output_vector})
        print(intersection)
