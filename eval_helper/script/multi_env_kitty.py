#!/usr/bin/env python3

import os
import time
import sys
import re
import subprocess
import argparse
import csv
import numpy as np

import calculate
from kitty_auto_test import run

parser = argparse.ArgumentParser()
parser.add_argument(
    "--world", type=str, default="_4uav_20obs_v2_a8_cls0.15_", help="world name"
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
    default="source /home/siyuan/workspace/thesis_workspace/devel/setup.bash",
)
parser.add_argument(
    "--cmd_recorder",
    type=str,
    default="roslaunch eval_helper eval.launch",
)
parser.add_argument(
    "--cmd_launch",
    type=str,
    default="roslaunch plan_manager sim_baseline_fkpcp_4.launch rviz:=false",
    # default="roslaunch plan_manager sim_baseline_fkpcp_4.launch rviz:=true",
)
parser.add_argument("--cmd_trigger", type=str, default="rosrun eval_helper trigger")

if __name__ == "__main__":
    args = parser.parse_args()
    launch_cmds = [
        # "roslaunch plan_manager sim_fkpcp_4_case_2.launch rviz:=false obs_num:=50",
        # "roslaunch plan_manager sim_fkpcp_4_case_2.launch rviz:=false obs_num:=40",
        # "roslaunch plan_manager sim_fkpcp_4_case_2.launch rviz:=false obs_num:=30",
        # "roslaunch plan_manager sim_fkpcp_4_case_2.launch rviz:=false obs_num:=20",
        # "roslaunch plan_manager sim_fkpcp_4_case_2.launch rviz:=false obs_num:=10",
        # "roslaunch plan_manager sim_fkpcp_4_case_1.launch rviz:=false obs_num:=50",
        # "roslaunch plan_manager sim_fkpcp_4_case_1.launch rviz:=false obs_num:=40",
        # "roslaunch plan_manager sim_fkpcp_4_case_1.launch rviz:=false obs_num:=30",
        # "roslaunch plan_manager sim_fkpcp_4_case_1.launch rviz:=false obs_num:=20",
        # "roslaunch plan_manager sim_fkpcp_4_case_1.launch rviz:=false obs_num:=10",
        "roslaunch plan_manager sim_fkpcp_4_case_0.launch rviz:=false obs_num:=50",
        "roslaunch plan_manager sim_fkpcp_4_case_0.launch rviz:=false obs_num:=40",
        "roslaunch plan_manager sim_fkpcp_4_case_0.launch rviz:=false obs_num:=30",
        "roslaunch plan_manager sim_fkpcp_4_case_0.launch rviz:=false obs_num:=20",
        "roslaunch plan_manager sim_fkpcp_4_case_0.launch rviz:=false obs_num:=10",
        "roslaunch plan_manager sim_fkpcp_4_case_3.launch rviz:=false obs_num:=50",
        "roslaunch plan_manager sim_fkpcp_4_case_3.launch rviz:=false obs_num:=40",
        "roslaunch plan_manager sim_fkpcp_4_case_3.launch rviz:=false obs_num:=30",
        "roslaunch plan_manager sim_fkpcp_4_case_3.launch rviz:=false obs_num:=20",
        "roslaunch plan_manager sim_fkpcp_4_case_3.launch rviz:=false obs_num:=10",
        "roslaunch plan_manager sim_fkpcp_4_case_4.launch rviz:=false obs_num:=10",
        "roslaunch plan_manager sim_fkpcp_4_case_4.launch rviz:=false obs_num:=20",
        "roslaunch plan_manager sim_fkpcp_4_case_4.launch rviz:=false obs_num:=30",
        "roslaunch plan_manager sim_fkpcp_4_case_4.launch rviz:=false obs_num:=40",
        "roslaunch plan_manager sim_fkpcp_4_case_4.launch rviz:=false obs_num:=50",
    ]
    world_names = [
        "ours_4uav_case0_50obs_v2_a6_cls0.15_ir1.5",
        "ours_4uav_case0_40obs_v2_a6_cls0.15_ir1.5",
        "ours_4uav_case0_30obs_v2_a6_cls0.15_ir1.5",
        "ours_4uav_case0_20obs_v2_a6_cls0.15_ir1.5",
        "ours_4uav_case0_10obs_v2_a6_cls0.15_ir1.5",
        "ours_4uav_case3_50obs_v2_a6_cls0.15_ir1.5",
        "ours_4uav_case3_40obs_v2_a6_cls0.15_ir1.5",
        "ours_4uav_case3_30obs_v2_a6_cls0.15_ir1.5",
        "ours_4uav_case3_20obs_v2_a6_cls0.15_ir1.5",
        "ours_4uav_case3_10obs_v2_a6_cls0.15_ir1.5",
        "ours_4uav_case4_10obs_v2_a6_cls0.15_ir1.5",
        "ours_4uav_case4_20obs_v2_a6_cls0.15_ir1.5",
        "ours_4uav_case4_30obs_v2_a6_cls0.15_ir1.5",
        "ours_4uav_case4_40obs_v2_a6_cls0.15_ir1.5",
        "ours_4uav_case4_50obs_v2_a6_cls0.15_ir1.5",
        # "ours_4uav_case2_50obs_v2_a6_cls0.15_ir1.5",
        # "ours_4uav_case2_40obs_v2_a6_cls0.15_ir1.5",
        # "ours_4uav_case2_30obs_v2_a6_cls0.15_ir1.5",
        # "ours_4uav_case2_20obs_v2_a6_cls0.15_ir1.5",
        # "ours_4uav_case2_10obs_v2_a6_cls0.15_ir1.5",
        # "ours_4uav_case1_50obs_v2_a6_cls0.15_ir1.5",
        # "ours_4uav_case1_40obs_v2_a6_cls0.15_ir1.5",
        # "ours_4uav_case1_30obs_v2_a6_cls0.15_ir1.5",
        # "ours_4uav_case1_20obs_v2_a6_cls0.15_ir1.5",
        # "ours_4uav_case1_10obs_v2_a6_cls0.15_ir1.5",
    ]
    for i in range(args.iters):
        print("=====    Iteration: {}    =====".format(i))
        for (cmd, world_name) in zip(launch_cmds, world_names):
            print("=====    World: {}    =====".format(world_name))
            args.cmd_launch = cmd
            args.world = world_name
            run(args)
            time.sleep(1)
    os.system(
        "notify-send -u critical -t 2000 --hint int:transient:1 AutoScript finished"
    )
