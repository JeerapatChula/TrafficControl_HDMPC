#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function

import numpy as np
from libs.etc.files import File


if __name__ == "__main__":
    scenario = "sathorn_full_junctions_morning_100"
    profile = "fixed_case"
    controllers = ["Fixed"]
    intersections = File.load_json('scenarios/{}/{}/intersections.json'.format(scenario, profile))

    num_gamma = 5
    num_trials = 1

    summary = {}

    for intersection in intersections:
        summary[intersection] = {}
        for road in intersections[intersection]["incoming_roads"]:
            summary[intersection][road] = np.zeros((1, len(intersections[intersection]["incoming_roads"][road]["dst_roads"])))

    for intersection in intersections:
        tmp = File.load_json('results/{}/{}/{}/splitting_flow.json'.format(scenario, profile, intersection))
        for road in intersections[intersection]["incoming_roads"]:
            N = 0
            for controller in controllers:
                data = tmp["trial_1"][controller]
                summary[intersection][road] += np.array(data[road])
                N += 1
            summary[intersection][road] = summary[intersection][road]/N
            tmp_sum = np.sum(summary[intersection][road])
            summary[intersection][road] = summary[intersection][road]/tmp_sum if tmp_sum > 0 else summary[intersection][road]
            summary[intersection][road] = summary[intersection][road][0].tolist()
    File.save_json("summarized_results/splitting_flow.json", summary)
