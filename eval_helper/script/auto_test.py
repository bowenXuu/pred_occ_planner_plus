#!/usr/bin/env python3

"""
Date: 2022-03-02
Author: Gang Chen
Email: 947089399@qq.com
---
thesis-planner
"""

import os
import time
import sys
import re
import subprocess
import argparse
import csv
import numpy as np

import calculate

parser = argparse.ArgumentParser()
parser.add_argument(
    "--world", type=str, default="_4uav_19obs_cls0.2_", help="world name"
)
parser.add_argument("--iters", type=int, default="1", help="number of agents")
parser.add_argument("--num_agents", type=int, default="4", help="number of agents")
parser.add_argument("--save_path", type=str, default="results", help="folder name")
parser.add_argument("--waiting_time", type=float, default="20", help="waiting time")
parser.add_argument(
    "--node_name", type=str, default="/uav0/planner", help="key planning node name"
)
parser.add_argument("--no-run", action="store_true")

parser.add_argument(
    "--cmd_source",
    type=str,
    default="source /home/siyuan/workspace/thesis_workspace/devel/setup.zsh",
)
parser.add_argument(
    "--cmd_recorder",
    type=str,
    default="roslaunch eval_helper eval.launch",
)
parser.add_argument(
    "--cmd_launch",
    type=str,
    default="roslaunch plan_manager sim_baseline_fkpcp_4.launch",
)
parser.add_argument("--cmd_trigger", type=str, default="rosrun eval_helper trigger")


def startPlanner():
    pass


def checkIfNodeRunning(node_name):
    pid = os.popen("rosnode list | grep " + node_name).read()
    # print('pid of ' + node_name + ' is ' + pid)
    return pid


def cleanROSLog():
    os.system("gnome-terminal -- bash -c '%s; rosclean purge -y'" % "source ~/.bashrc")


def stopNode(node_name):
    node_pid = checkIfNodeRunning(node_name)
    if node_pid:
        os.system("kill " + node_pid)
        time.sleep(1)


def startAll(arg):
    # Start the planner
    os.system("gnome-terminal -- bash -c '%s; %s;'" % (arg.cmd_source, arg.cmd_launch))
    time.sleep(0.5)
    # Start the recorder
    os.system(
        "gnome-terminal -- bash -c '%s; %s;'" % (arg.cmd_source, arg.cmd_recorder)
    )
    time.sleep(2)
    # Start planning
    os.system("gnome-terminal -- bash -c '%s; %s;'" % (arg.cmd_source, arg.cmd_trigger))


def stopAll():
    os.system("kill $(pgrep bash)")
    time.sleep(2)

    # stopNode("gzserver")
    # time.sleep(1)
    #
    # stopNode("gzclient")
    # time.sleep(1)


def findAndRecord(args, save_path):
    log_root = "/home/siyuan/.ros/log/latest/"

    eval_log_path = None
    for file in os.listdir(log_root):
        if re.match(r".*multi_eval.*", file):
            eval_log_path = log_root + file

    if eval_log_path is None:
        print("No evaluation log found!")
        return

    # extract data from file:
    data = calculate.get_data(args.num_agents, eval_log_path)

    ctrl_efforts = np.array([calculate.get_sum_control_efforts(d) for d in data])
    flight_times = np.array([calculate.get_avg_flight_time(d) for d in data])

    n_cld_obs = np.array([calculate.get_collision_occurance(d, 12) for d in data])
    d_cld_obs = np.array([np.min(d[:, 12]) for d in data])

    n_cld_uav = np.array([calculate.get_collision_occurance(d, 13) for d in data])
    d_cld_uav = np.array([np.min(d[:, 13]) for d in data])

    arr_to_write = np.hstack(
        [ctrl_efforts, flight_times, n_cld_uav, d_cld_uav, n_cld_obs, d_cld_obs]
    )
    return arr_to_write


def run(arg):
    save_path = arg.save_path + "_" + arg.world + "_ground_truth" + ".csv"

    if not arg.no_run:
        cleanROSLog()
        time.sleep(0.5)
        if not os.path.exists(save_path):
            with open(save_path, "w+") as result:
                writer = csv.writer(result)
                ctr_eft_headers = [
                    "ctrl_effort_uav" + str(i) for i in range(arg.num_agents)
                ]
                flt_tim_headers = [
                    "flight_time_uav" + str(i) for i in range(arg.num_agents)
                ]
                n_cls_obs_headers = [
                    "num_collide_obs" + str(i) for i in range(arg.num_agents)
                ]
                n_cls_uav_headers = [
                    "num_collide_uav" + str(i) for i in range(arg.num_agents)
                ]
                min_dist_uav_headers = [
                    "min_dist_uav" + str(i) for i in range(arg.num_agents)
                ]
                min_dist_obs_headers = [
                    "min_dist_obs" + str(i) for i in range(arg.num_agents)
                ]
                # TODO: min dist to obs, min dist to agents
                headers = (
                    ctr_eft_headers
                    + flt_tim_headers
                    + n_cls_uav_headers
                    + min_dist_uav_headers
                    + n_cls_obs_headers
                    + min_dist_obs_headers
                )
                writer.writerow(headers)

        startAll(arg)

        counter = 0
        while counter < arg.waiting_time:  # Wait time max: 60s
            failed_list = np.zeros(arg.num_agents)
            for i in range(arg.num_agents):
                node_name = re.sub(r"\d", str(i), arg.node_name)
                if checkIfNodeRunning(node_name) == "":
                    failed_list[i] += 1
                    print("Agents {} are not running. Break. ".format(i))

            counter += 1
            time.sleep(1)

            if np.min(failed_list) > 0:
                break

        " Stop "
        stopNode("node_name")
        time.sleep(1)
        stopAll()

    arr = findAndRecord(args, save_path)
    if arr is not None:
        with open(save_path, "a") as result:
            csv_row = ["{:.4f}".format(a) for a in arr]
            csv_text = ", ".join(csv_row) + "\n"
            result.write(csv_text)
            print("Data: " + csv_text)
        print("Data saved to " + save_path)


if __name__ == "__main__":
    args = parser.parse_args()
    for i in range(args.iters):
        run(args)
    # print(args.no_run)
    # save_path = args.save_path + args.world + "_measurement_noise_" + ".csv"
    # print(findAndRecord(args, save_path))
    #
    # arr = findAndRecord(args, save_path)
    # if arr is not None:
    #     with open(save_path, "a") as result:
    #         csv_row = ["{:.4f}".format(a) for a in arr]
    #         csv_text = ", ".join(csv_row) + "\n"
    #         result.write(csv_text)
    #     print("Data saved to " + save_path)
