#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import numpy as np
from numpy import linalg as LA
import matplotlib.pyplot as plt
from libs.etc.files import File


class Summarize():

    @staticmethod
    def get_average_performance_fixed(num_trials, controller, data):
        tmp = 0
        for trial in range(1, num_trials+1):
            str = "trial_{}".format(trial)
            tmp += data[str][controller]
        return tmp/num_trials

    @staticmethod
    def get_average_performance_weighted_control(num_trials, gamma_start, gamma_stop, gamma_step, controller, data):
        num_gamma = (gamma_stop - gamma_start)/gamma_step + 1
        num_gamma = int(num_gamma)
        tmp = [0]*num_gamma
        for (gamma, ind) in zip(range(gamma_start, gamma_stop+1, gamma_step), range(num_gamma)):
            for trial in range(1, num_trials+1):
                str1 = "trial_{}".format(trial)
                str2 = "gamma_{}".format(gamma)
                tmp[ind] += data[str1][controller][str2]
            tmp[ind] = tmp[ind]/num_trials
        return tmp

    @staticmethod
    def summarize_fixed(num_trials, scenario, profile, controller, num_junctions):
        tmp_flow = {}
        tmp_travel = {}

        path = "results/{}/{}/".format(scenario, profile)
        data_flow = File.load_json("{}/total_average_flow.json".format(path))
        data_travel = File.load_json("{}/total_average_travel_time.json".format(path))
        tmp_flow["total"] = Summarize.get_average_performance_fixed(num_trials, controller, data_flow)
        tmp_travel["total"] = Summarize.get_average_performance_fixed(num_trials, controller, data_travel)

        tmp_flow["junctions"] = {}
        tmp_travel["junctions"] = {}
        for junction in range(1, num_junctions+1):
            intersection = "J{}".format(junction)
            file_name_flow = "{}/{}/average_flow.json".format(path, intersection)
            file_name_travel = "{}/{}/average_travel_time.json".format(path, intersection)
            tmp_flow["junctions"][intersection] = Summarize.get_average_performance_fixed(num_trials, controller, File.load_json(file_name_flow))
            tmp_travel["junctions"][intersection] = Summarize.get_average_performance_fixed(num_trials, controller, File.load_json(file_name_travel))
        return tmp_flow, tmp_travel

    @staticmethod
    def summarize_weighted_control(num_trials, scenario, profile, controller, gamma_start, gamma_stop, gamma_step, num_junctions):
        tmp_flow = {}
        tmp_travel = {}
        path = "results/{}/{}/".format(scenario, profile)
        data_flow = File.load_json("{}/total_average_flow.json".format(path))
        data_travel = File.load_json("{}/total_average_travel_time.json".format(path))
        tmp_flow["total"] = Summarize.get_average_performance_weighted_control(num_trials, gamma_start, gamma_stop, gamma_step, controller, data_flow)
        tmp_travel["total"] = Summarize.get_average_performance_weighted_control(num_trials, gamma_start, gamma_stop, gamma_step, controller, data_travel)

        tmp_flow["junctions"] = {}
        tmp_travel["junctions"] = {}
        for junction in range(1, num_junctions+1):
            intersection = "J{}".format(junction)
            file_name_flow = "{}/J{}/average_flow.json".format(path, junction)
            file_name_travel = "{}/J{}/average_travel_time.json".format(path, junction)
            tmp_flow["junctions"][intersection] = Summarize.get_average_performance_weighted_control(num_trials, gamma_start, gamma_stop, gamma_step, controller, File.load_json(file_name_flow))
            tmp_travel["junctions"][intersection] = Summarize.get_average_performance_weighted_control(num_trials, gamma_start, gamma_stop, gamma_step, controller, File.load_json(file_name_travel))
        return tmp_flow, tmp_travel

    @staticmethod
    def summarize_flow_rmse(num_trials, scenario, profile, controller, gamma_start, gamma_stop, gamma_step):
        output_path = "{}/sumo_outputs".format(os.environ["HOME"])
        num_gamma = (gamma_stop - gamma_start)/gamma_step + 1
        num_gamma = int(num_gamma)
        tmp_1 = [0]*num_gamma
        tmp_2 = [0]*num_gamma
        for (gamma, ind) in zip(range(gamma_start, gamma_stop+1, gamma_step), range(num_gamma)):
            arg_rmse_trial = 0
            arg_est_rmse_trial = 0
            for trial in range(1, num_trials+1):

                est_rmse_int_total = 0
                for intersection in range(1, 11):
                    path = '{}/trial_{}/{}/{}/{}/{}/controllers/intersections/J{}.mat'.format(output_path, trial, scenario, profile, controller, gamma, intersection)
                    model_path = 'models/HDMPC3_Np1/intersection/J{}.mat'.format(intersection)

                    data = File.load_mat(path)
                    model = File.load_mat(model_path)
                    control_signal_log = data["control_signal_log"]
                    ref_log = data["ref_log"]
                    matrix_q = model["matrix_q"]
                    N = control_signal_log.shape[0]
                    arg_rmse_int = 0
                    num_phase = control_signal_log.shape[1]
                    for t in range(N):
                        est_flow = np.matmul(matrix_q[:,0:num_phase], control_signal_log[t, :].T)
                        rmse = LA.norm(3600*(ref_log [t, :].T - est_flow))
                        arg_rmse_int += rmse
                    arg_rmse_int = arg_rmse_int/N
                    est_rmse_int_total += arg_rmse_int
                arg_est_rmse_trial += est_rmse_int_total

                arg_rmse_total = 0
                for area in range(1, 3+1):
                    path = '{}/trial_{}/{}/{}/{}/{}/controllers/area/A{}.mat'.format(output_path, trial, scenario, profile, controller, gamma, area)
                    data = File.load_mat(path)
                    input_log = data["input_log"]
                    control_signal_log = data["control_signal_log"]
                    N = input_log.shape[0]
                    arg_rmse_area = 0
                    for t in range(N):
                        arg_rmse_area += LA.norm(input_log[t, :] - control_signal_log[t-1, :], 2)
                    arg_rmse_area = arg_rmse_area/N
                    arg_rmse_total += arg_rmse_area
                arg_rmse_trial += arg_rmse_total

            arg_est_rmse_trial = arg_est_rmse_trial/num_trials
            arg_rmse_trial = arg_rmse_trial/num_trials
            tmp_1[ind] = arg_rmse_trial
            tmp_2[ind] = arg_est_rmse_trial
        return (tmp_1, tmp_2)


if __name__ == "__main__":
    scenario = "sathorn_full_junctions_morning_100"
    profiles = ["case_1", "case_2", "case_3", "case_4", "case_5"]
    profiles = ["case_5"]
    # profiles = ["case_1", "case_2", "case_3"]
    # profiles = ["A_1", "A_2", "A_3", "A_4"]
    legend = ["case_5"]
    controllers = ["HDMPC_Np1", "HDMPC_Np2", "HDMPC_Np3"]
    intersection = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9", "J10"]

    gamma_start = 0
    gamma_stop = 20000
    gamma_step = 1000

    num_trials = 5
    num_junctions = 10
    x_number = range(0, 20001, 1000)

    average_flow, average_travel_time = Summarize.summarize_fixed(num_trials, scenario, "fixed_case", "Fixed", num_junctions)
    path = "summarized_results/{}/fixed.mat".format(scenario)
    File.save_mat(path, {"average_flow": average_flow, "average_travel_time": average_travel_time})
    total_average_flow_fixed = average_flow["total"]
    total_average_travel_time_fixed = average_travel_time["total"]
    print("Manual control: flow={}, travel time={}".format(total_average_flow_fixed, total_average_travel_time_fixed))

    # average_flow, average_travel_time = Summarize.summarize_fixed(num_trials, scenario, "case_3", "Fuzzy", num_junctions)
    # path = "summarized_results/{}/fixed.mat".format(scenario)
    # File.save_mat(path, {"average_flow": average_flow, "average_travel_time": average_travel_time})
    # total_average_flow_fuzzy = average_flow["total"]
    # total_average_travel_time_fuzzy = average_travel_time["total"]
    # print("Fuzzy control: flow={}, travel time={}".format(total_average_flow_fuzzy, total_average_travel_time_fuzzy))

    # plt.clf()
    for (controller, c) in zip(controllers, range(len(controllers))):
        # plt.subplot(211+c)
        # plt.plot(x_number, np.ones((21, 1))*total_performance_fixed)
        # print(controller)
        for profile in profiles:
            average_flow, average_travel_time = Summarize.summarize_weighted_control(num_trials, scenario, profile, controller, gamma_start, gamma_stop, gamma_step, num_junctions)
            # flow_rmse, flow_est_rmse = Summarize.summarize_flow_rmse(num_trials, scenario, profile, controller, gamma_start, gamma_stop, gamma_step)
            path = "summarized_results/{}/{}/{}.mat".format(scenario, profile, controller)
            File.save_mat(path, {"average_flow": average_flow, "average_travel_time": average_travel_time})
            # File.save_mat(path, {"total_performance": total_performance, "intersection_performance": intersection_performance, "flow_rmse": flow_rmse, "flow_est_rmse": flow_est_rmse})
            y_flow = 100*(np.array(average_flow["total"])-float(total_average_flow_fixed))/float(total_average_flow_fixed)
            y_travel = 100*(np.array(average_travel_time["total"])-float(total_average_travel_time_fixed))/float(total_average_travel_time_fixed)
            # plt.plot(x_number, y)
            ind_max_flow = np.argmax(y_flow)
            ind_min_travel = np.argmin(y_travel)
            # print("{} => Flow:{} = {}% / Travel:{} = {}%".format(profile, ind_max_flow*gamma_step, y_flow[ind_max_flow], ind_max_flow*gamma_step, y_travel[ind_max_flow]))

            print("{}: flow={}, travel time={}".format(controller, average_flow["total"][ind_max_flow], average_travel_time["total"][ind_max_flow]))

        # plt.legend(legend, loc='upper left')
        # plt.ylabel('Change (%)')
        # plt.xlabel('Gamma')
    # plt.savefig('summarized_results/plots/total_average_flow.png')
