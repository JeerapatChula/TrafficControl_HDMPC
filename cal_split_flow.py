#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function

import numpy as np
from libs.etc.files import File



if __name__ == "__main__":
    scenario = "sathorn_full_junctions_morning_100"
    profile = "case_5"
    controllers = ["HDMPC_Np1", "HDMPC_Np2", "HDMPC_Np3"]
    intersections = File.load_json(f'scenarios/{scenario}/{profile}/intersections.json')


    gamma_start = 0
    gamma_stop = 20000
    gamma_step = 1000
    num_gamma = int(gamma_stop/gamma_step)
    num_trials = 5

    summary = {}

    for intersection in intersections:
        summary[intersection] = {}
        for road in intersections[intersection]["incoming_roads"]:
            summary[intersection][road] = np.zeros((1, len(intersections[intersection]["incoming_roads"][road]["dst_roads"])))

    for intersection in intersections:
        tmp = File.load_json(f'results/{scenario}/{profile}/{intersection}/splitting_flow.json')
        for road in intersections[intersection]["incoming_roads"]:
            N = 0
            for controller in controllers:
                for (gamma, ind) in zip(range(gamma_start, gamma_stop+1, gamma_step), range(num_gamma)):
                    for trial in range(num_trials):
                        trial = trial + 1
                        data = tmp[f"trial_{trial}"][controller][f"gamma_{gamma}"]
                        summary[intersection][road] += np.array(data[road])
                        N += 1
            summary[intersection][road] = summary[intersection][road]/N
            tmp_sum = np.sum(summary[intersection][road])
            summary[intersection][road] = summary[intersection][road]/tmp_sum if tmp_sum > 0 else summary[intersection][road]
            summary[intersection][road] = summary[intersection][road][0].tolist()
    File.save_json("summarized_results/averaged_output/splitting_flow.json", summary)
