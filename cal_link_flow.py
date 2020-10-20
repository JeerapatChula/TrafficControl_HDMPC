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
    links = File.load_json(f'scenarios/{scenario}/{profile}/links.json')
    roads = File.load_json(f'scenarios/{scenario}/{profile}/roads.json')

    gamma_start = 0
    gamma_stop = 20000
    gamma_step = 1000
    num_gamma = int(gamma_stop/gamma_step)
    num_trials = 5

    for link in links:
        links[link]["ratio"] = 0
        source_flow = 0
        sink_flow = 0
        N = 0
        for road in links[link]["roads"]:
            source = roads[road][0]["source"]
            sink = roads[road][0]["sink"]
            for controller in controllers:
                for (gamma, ind) in zip(range(gamma_start, gamma_stop+1, gamma_step), range(num_gamma)):
                    for trial in range(num_trials):
                        N += 1
                        if source[0] == 'J':
                            path = "{}/sumo_outputs/trial_{}/{}/{}/{}/{}/modules/intersections_{}/{}.mat".format(os.environ["HOME"], trial+1, scenario, profile, controller, gamma, source, road)
                            data = File.load_mat(path)
                            source_flow += np.sum(np.sum(data["num_vehicles_in"]))
                        else:
                            path = "{}/sumo_outputs/trial_{}/{}/{}/{}/{}/modules/boundaries/{}.mat".format(os.environ["HOME"], trial+1, scenario, profile, controller, gamma, source)
                            data = File.load_mat(path)
                            source_flow += np.sum(data["departured_log"])

                        if sink[0] == 'J':
                            path = "{}/sumo_outputs/trial_{}/{}/{}/{}/{}/modules/intersections_{}/{}.mat".format(os.environ["HOME"], trial+1, scenario, profile, controller, gamma, sink, road)
                            data = File.load_mat(path)
                            sink_flow += np.sum(np.sum(data["num_vehicles_out"]))
                        else:
                            path = "{}/sumo_outputs/trial_{}/{}/{}/{}/{}/modules/boundaries/{}.mat".format(os.environ["HOME"], trial+1, scenario, profile, controller, gamma, sink)
                            data = File.load_mat(path)
                            sink_flow += np.sum(data["arrived_log"])
            source_flow = source_flow/N
            sink_flow = sink_flow/N
            links[link]["ratio"] = sink_flow/source_flow if source_flow != 0 else 1
    File.save_json("summarized_results/averaged_output/links.json", links)
