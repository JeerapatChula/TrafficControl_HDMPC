#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function

import os
import sys

os.system('export SUMO_HOME=/home/mild/sumo-1.2.0/')

try:
    sys.path.append(os.path.join(os.path.dirname(__file__), ""))
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(os.path.dirname(__file__), "..", "..", "..")), "tools"))
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit("please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")


import traci
import time
import threading
import libs.etc.progress as ps
import random
import keras
from module_managers import ModuleManager
from intermediaries import Intermediary
from network_configs import NetworkConfig
from performance_measurements import PerformanceMeasurement
from controllers.controllers import Controllers
from libs.etc.files import File
# from objbrowser import browse


class Runner():

    def __init__(self, simulation_configs):
        self.simulation_configs = simulation_configs

    def run(self):
        start_time = time.time()
        intermediary = Intermediary()
        module_manager = ModuleManager(intermediary, self.network_config)
        controller = Controllers.load_controller(intermediary, self.controller_config, self.network_config)
        performance_measurement = PerformanceMeasurement(intermediary, self.network_config, self.sim_length)


        for step in range(self.sim_length+1):
            traci.simulationStep()
            module_manager.update()
            controller.update()
            ps.progress(step+1, self.sim_length, start_time)
            # input([name for name,thing in traci._simulation.Stage.travelTime.getmembers([])])
            # tmp = traci._simulation.Stage
            # if step == 500:
                # browse(locals())
            # input(traci._simulation.Stage.travelTime.getter(5))
            # for property, value in vars(traci._simulation.Stage.travelTime).iteritems():
                # input("{}: {}".format(property, value))

        module_manager.save(self.output_path)
        controller.save(self.output_path)
        performance_measurement.summarize(self.simulation_options)
        ps.end()
        traci.close()
        # tmp = asdict(traci._simulation.Stage)
        # input(tmp)
        # browse(locals())
        sys.stdout.flush()
        print("\n")
        end_time = time.time()
        current_time = time.time()
        # print(current_time)
        # while current_time - end_time < 5:
        #     current_time = time.time()


    def load_simulator(self, trial, scenario, profile):
        sumoBinary = checkBinary('sumo-gui') if self.simulation_configs["sumo_gui"] else checkBinary('sumo')
        scenario_config = self.simulation_configs["scenarios"][scenario]
        path = "scenarios/{}/network.sumo.cfg".format(scenario)
        sumo_options = [sumoBinary, "-c", path, "--seed", str(trial*self.simulation_configs["seed_step"])]

        for option in scenario_config["sumo_options"]:
            tmp = scenario_config["sumo_options"][option]
            tmp = "scenarios/{}/{}".format(scenario, tmp) if option == '-a' else tmp
            sumo_options.append(option)
            sumo_options.append(tmp)

        self.sim_length = self.simulation_configs["scenarios"][scenario]["sim_length"]
        traci.start(sumo_options, label="{} : {}".format(scenario, profile["name"]))
        self.run()

    def load_all_scenarios(self, trial):
        simulation_queue = File.load_json('simulation_queue.json')
        simulation_queue[f"trial_{trial}"] = []

        for scenario in self.simulation_configs["scenarios"]:
            for profile in self.simulation_configs["scenarios"][scenario]["profiles"]:
                if profile["active"]:

                    # self.network_config = NetworkConfig(scenario, profile["name"])
                    self.load_all_fixed_controllers(trial, scenario, profile, simulation_queue)
                    self.load_all_weighted_controllers(trial, scenario, profile, simulation_queue)
        File.save_json('simulation_queue.json', simulation_queue)

    def load_all_fixed_controllers(self, trial, scenario, profile, simulation_queue):

        for controller in self.simulation_configs["controllers"]:
            if self.simulation_configs["controllers"][controller]["active"]:
                if not self.simulation_configs["controllers"][controller]["weighted"]:
                    self.controller_config = {}
                    self.controller_config["controller"] = self.simulation_configs["controllers"][controller]["name"]
                    self.controller_config["options"] = self.simulation_configs["controllers"][controller]["options"]
                    self.controller_config["weighted"] = False
                    # print("Controller : {}".format(controller))
                    self.output_path = '{}/trial_{}/{}/{}/{}'.format(self.simulation_configs["output_path"], trial, scenario, profile["name"], controller)
                    self.simulation_options = {"trial": trial, "scenario": scenario, "profile": profile, "controller": controller}

                    temp = {"controller_config": self.controller_config, "output_path": self.output_path, "simulation_options": self.simulation_options, "scenario": scenario, "profile": profile}
                    simulation_queue[f"trial_{trial}"].append(temp)

                    # not_completed = True
                    # # self.load_simulator(trial, scenario, profile)
                    # while not_completed:
                    #     try:
                    #         self.load_simulator(trial, scenario, profile)
                    #         not_completed = False
                    #     except KeyboardInterrupt:
                    #         sys.exit("============= Keyboard Interrupt =============")
                    #     except:
                    #         print("============= simulation error =============")

    def load_all_weighted_controllers(self, trial, scenario, profile, simulation_queue):

        start = self.simulation_configs["weighting"]["gamma_start"]
        step = self.simulation_configs["weighting"]["gamma_step"]
        stop = self.simulation_configs["weighting"]["gamma_stop"]

        for gamma in range(start, stop+1, step):
            for controller in self.simulation_configs["controllers"]:
                if self.simulation_configs["controllers"][controller]["active"]:
                    if self.simulation_configs["controllers"][controller]["weighted"]:
                        self.controller_config = {}
                        self.controller_config["controller"] = self.simulation_configs["controllers"][controller]["name"]
                        self.controller_config["options"] = self.simulation_configs["controllers"][controller]["options"]
                        self.controller_config["weighted"] = True
                        self.controller_config["gamma"] = gamma
                        # if "offset" in self.simulation_configs["weighting"]:
                        #     self.controller_config["gamma"] = gamma + self.simulation_configs["weighting"]["offset"]
                        # print("Controller : {}, gamma = {}".format(controller, gamma))
                        self.output_path = '{}/trial_{}/{}/{}/{}/{}'.format(self.simulation_configs["output_path"], trial, scenario, profile["name"], controller, gamma)
                        self.simulation_options = {"trial": trial, "scenario": scenario, "profile": profile, "controller": controller, "gamma": gamma}
                        not_completed = True
                        # self.load_simulator(trial, scenario, profile)

                        temp = {"controller_config": self.controller_config, "output_path": self.output_path, "simulation_options": self.simulation_options, "scenario": scenario, "profile": profile}
                        simulation_queue[f"trial_{trial}"].append(temp)

                        # while not_completed:
                        #     try:
                        #         self.load_simulator(trial, scenario, profile)
                        #         # self.load_simulator(trial, scenario, profile)
                        #         not_completed = False
                        #     except KeyboardInterrupt:
                        #         sys.exit("============= Keyboard Interrupt =============")
                        #     except:
                        #         print("============= simulation error =============")
                        #         self.controller_config["gamma"] = gamma + random.randint(0, 5)



if __name__ == "__main__":

    def clear_queue():
        File.save_json('simulation_queue.json', {})

    def load_queue(trial):
        simulation_queue = File.load_json('simulation_queue.json')
        if f"trial_{trial}" in simulation_queue:
            if len(simulation_queue[f"trial_{trial}"]) > 0:
                temp = simulation_queue[f"trial_{trial}"].pop(0)
                File.save_json('simulation_queue.json', simulation_queue)

                simulation_configs = File.load_json('simulation_configs.json')
                # simulation_configs["output_path"] = simulation_configs["output_path"].format(os.environ["HOME"])
                simulation_configs["output_path"] = simulation_configs["output_path"]
                runner = Runner(simulation_configs)

                scenario = temp["scenario"]
                profile = temp["profile"]

                runner.network_config = NetworkConfig(scenario, profile["name"])
                runner.controller_config = temp["controller_config"]
                runner.output_path = temp["output_path"]
                runner.simulation_options = temp["simulation_options"]

                not_completed = True
                print("=========== Trial {} ===========".format(trial))
                if temp["controller_config"]["weighted"]:
                    print("{} : {}".format(temp["scenario"], temp["profile"]["name"]))
                    print("Controller : {}, gamma = {}".format(runner.simulation_options["controller"], runner.controller_config["gamma"]))
                # runner.load_simulator(trial, scenario, profile)
                # self.load_simulator(trial, scenario, profile)

                runner.load_simulator(trial, temp["scenario"], temp["profile"])
                # while not_completed:
                #     try:
                #         runner.load_simulator(trial, temp["scenario"], temp["profile"])
                #         not_completed = False
                #     except KeyboardInterrupt:
                #         sys.exit("============= Keyboard Interrupt =============")
                #     except:
                #         print("============= simulation error =============")

                keras.backend.clear_session()
                myThread = threading.Timer(5, load_queue, args=(trial,))
                myThread.start()

    clear_queue()
    simulation_configs = File.load_json('simulation_configs.json')
    # simulation_configs["output_path"] = simulation_configs["output_path"].format(os.environ["HOME"])
    simulation_configs["output_path"] = simulation_configs["output_path"]
    runner = Runner(simulation_configs)
    print("1. auto, 2. manual, 3. fix gamma, 4. gamma sloted")
    mode = input("mode : ")
    if mode == '1':
        start = runner.simulation_configs["start_trial"]
        stop = runner.simulation_configs["max_trial"]
        for trial in range(start, stop+1):
            runner.load_all_scenarios(trial)
            load_queue(trial)
    elif mode == '2':
        trial = int(input("trial : "))
        start_gamma = int(input("start gamma : "))
        runner.simulation_configs["weighting"]["gamma_start"] = start_gamma
        runner.load_all_scenarios(trial)
        load_queue(trial)
    elif mode == '3':
        trial = int(input("trial : "))
        gamma = int(input("start gamma : "))
        # offset = int(input("offset : "))
        runner.simulation_configs["weighting"]["gamma_start"] = gamma
        runner.simulation_configs["weighting"]["gamma_stop"] = gamma
        # runner.simulation_configs["weighting"]["offset"] = offset
        runner.load_all_scenarios(trial)
        load_queue(trial)
    elif mode == '4':
        trial = 1
        slot = int(input("slot : "))
        if slot > 0:
            runner.simulation_configs["weighting"]["gamma_start"] = (slot*3000)
            runner.simulation_configs["weighting"]["gamma_stop"] = (slot*3000)
        else:
            runner.simulation_configs["weighting"]["gamma_start"] = slot
            runner.simulation_configs["weighting"]["gamma_stop"] = slot
        runner.load_all_scenarios(trial)
