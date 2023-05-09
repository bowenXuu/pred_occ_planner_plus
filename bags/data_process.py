#!/usr/bin/env python3

import math
import numpy as np
import scipy.io as sio
import sys
import matplotlib.pyplot as plt
import matplotlib as mpl

import bagpy
from bagpy import bagreader
import pandas as pd
import os
from os import listdir
from os.path import isfile, join
import time
from geometry_msgs.msg import PoseStamped

mypath = str("./")
bagfile = [f for f in listdir(mypath) if "bag" in join(mypath, f)]

for count in range(len(bagfile)):
    file_name = join(mypath, bagfile[count])
    print(file_name)

    # load bag
    bag = bagreader(file_name)
    bag_uav_set = bag.message_by_topic("/mavros/setpoint_raw/local")
    bag_success = bag.message_by_topic("/test_over")
    bag_time = bag.message_by_topic("/one_plan_time")
    pd_uav_set = pd.read_csv(bag_uav_set)
    pd_success = pd.read_csv(bag_success)
    pd_time = pd.read_csv(bag_time)

    # data process
    # 1. one_round_duration
    print("1. one round duration")
    one_round_time = []
    print("one_round_time size:", len(pd_time["Time"]))
    for ele in pd_time["Time"]:
        one_round_time.append(ele)
    print("one_round_time:", one_round_time)

    one_round_duration = []
    for i, ele in enumerate(one_round_time):
        if (i < len(one_round_time)-1):
            one_round_duration.append(one_round_time[i+1] - one_round_time[i])
    print("one_round_duration size:", len(one_round_duration))
    print("one round duration: ", one_round_duration)

    # TODO: segment these data to 11 parts and get all max accs
    corse_time = []
    for i, ele in enumerate(pd_uav_set["Time"]):
        for ele_time in one_round_time:
            if abs(ele - ele_time) < 0.02:
                corse_time.append(ele)

    filtered_time = []
    for i, ele in enumerate(corse_time):
        if corse_time.index(ele)+1 < len(corse_time):
            if abs(corse_time[corse_time.index(ele)+1] - ele) > 1.:
                filtered_time.append(ele)
    filtered_time.append(corse_time[-1])
    # print("filtered_time size: ", len(filtered_time))
    # print("filtered_time: ", filtered_time)

    pd_uav_set_time = pd_uav_set["Time"].tolist()
    slice_index = []
    for i, ele_i in enumerate(filtered_time):
        # print(pd_uav_set_time.index(ele_i))
        slice_index.append(pd_uav_set_time.index(ele_i))
    # print("len of slice_index: ", len(slice_index))
    # print("slice_index: ", slice_index)

    # 2. max acc
    print("2. max acc")
    max_acc_x_value = []
    print("acc.x size:", len(pd_uav_set["acceleration_or_force.x"]))
    for i, ele in enumerate(slice_index):
        if i < len(slice_index) - 1:
            pre_index = slice_index[i]
            next_index = slice_index[i+1]
            max_acc_x_value.append(
                max(
                    [
                        abs(ele)
                        for ele in pd_uav_set["acceleration_or_force.x"]
                        .to_numpy()
                        .tolist()[pre_index: next_index]
                    ]
                )
            )
    print("max_acc_x_value: ", max_acc_x_value)
    print("len max_acc_x_value: ", len(max_acc_x_value))

    max_acc_y_value = []
    print("acc.y size:", len(pd_uav_set["acceleration_or_force.y"]))
    for i, ele in enumerate(slice_index):
        if i < len(slice_index) - 1:
            pre_index = slice_index[i]
            next_index = slice_index[i+1]
            max_acc_y_value.append(
                max(
                    [
                        abs(ele)
                        for ele in pd_uav_set["acceleration_or_force.y"]
                        .to_numpy()
                        .tolist()[pre_index: next_index]
                    ]
                )
            )
    print("max_acc_y_value: ", max_acc_y_value)
    print("len max_acc_y_value: ", len(max_acc_y_value))


    # # 3. max vel
    print("3. max vel")
    max_vel_x_value = []
    print("vel.x size:", len(pd_uav_set["velocity.x"]))
    for i, ele in enumerate(slice_index):
        if i < len(slice_index) - 1:
            pre_index = slice_index[i]
            next_index = slice_index[i+1]
            max_vel_x_value.append(
                max(
                    [
                        abs(ele)
                        for ele in pd_uav_set["velocity.x"]
                        .to_numpy()
                        .tolist()[pre_index: next_index]
                    ]
                )
            )
    print("max_vel_x_value: ", max_vel_x_value)

    max_vel_y_value = []
    print("vel.x size:", len(pd_uav_set["velocity.y"]))
    for i, ele in enumerate(slice_index):
        if i < len(slice_index) - 1:
            pre_index = slice_index[i]
            next_index = slice_index[i+1]
            max_vel_y_value.append(
                max(
                    [
                        abs(ele)
                        for ele in pd_uav_set["velocity.y"]
                        .to_numpy()
                        .tolist()[pre_index: next_index]
                    ]
                )
            )
    print("max_vel_y_value: ", max_vel_y_value)

    # 4. success rate
    print("4. success rate")
    success_rate = []
    print("success_rate size:", len(pd_success["pose.position.x"]))
    for ele in pd_success["pose.position.x"]:
        success_rate.append(ele)
    print("success_rate: ", success_rate[0])
    success_rate_list = [success_rate[0]]

    # 5. output
    mylist = [one_round_duration, max_acc_x_value, max_acc_y_value, max_vel_x_value, max_vel_y_value, success_rate_list]
    index =["one_plan_time", "max_acc_x", "max_acc_y", "max_vel_x", "max_vel_y", "success_rate%"]
    df = pd.DataFrame(mylist, index=index)
    file_name = "./output/" + file_name + "_data.csv"
    df.to_csv(file_name)
