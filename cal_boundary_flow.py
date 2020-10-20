#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function

import os
import numpy as np
from libs.etc.files import File


if __name__ == "__main__":
    scenario = "sathorn_full_junctions_morning_100"
    profile = "case_1"
    controllers = ["HDMPC_Np1", "HDMPC_Np2", "HDMPC_Np3"]
    boundaries = File.load_json(f'scenarios/{scenario}/{profile}/boundaries.json')

    gamma_start = 0
    gamma_stop = 20000
    gamma_step = 1000
    num_gamma = int(gamma_stop/gamma_step)
    num_trials = 5

    for boundary in boundaries:
        boundaries[boundary]["arrival_rate"] = 0
        boundaries[boundary]["departure_rate"] = 0
        arrival_rate = 0
        departure_rate = 0
        N = 0

        for controller in controllers:
            for (gamma, ind) in zip(range(gamma_start, gamma_stop+1, gamma_step), range(num_gamma)):
                for trial in range(num_trials):
                    N += 1
                    path = "{}/sumo_outputs/trial_{}/{}/{}/{}/{}/modules/boundaries/{}.mat".format(os.environ["HOME"], trial+1, scenario, profile, controller, gamma, boundary)
                    data = File.load_mat(path)
                    arrival_rate += np.sum(data["departured_log"]) / 3
                    departure_rate += np.sum(data["arrived_log"]) / 3

        boundaries[boundary]["arrival_rate"] = 1.25 * arrival_rate / N
        boundaries[boundary]["departure_rate"] = 1.25 * departure_rate / N
    File.save_json("summarized_results/averaged_output/boundaries.json", boundaries)
