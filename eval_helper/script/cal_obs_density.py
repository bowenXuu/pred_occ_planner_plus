#!/usr/bin/env python3
"""
Date: 2023-02-18
Author: Siyuan Wu 
Email: siyuanwu99@gmail.com
"""


import numpy as np
import sys
import rospy
from visualization_msgs.msg import Marker, MarkerArray

map_volumn = 1
obstacle_volum = 0


def obsCallback(msg):
    obstacle_volum = 0
    max_vel = 0  # max velocity
    min_vel = 100  # min velocity
    count = 0
    for mk in msg.markers:
        count += 1
        px = mk.points[0].x
        py = mk.points[0].y
        w = mk.scale.x / 2
        h = mk.points[0].z
        vx = mk.points[1].x - mk.points[0].x
        vy = mk.points[1].y - mk.points[0].y
        print(
            "obstacle {} with width {} occupied {} area".format(count, w, np.pi * w * w)
        )
        obstacle_volum += np.pi * w * w
        v = np.sqrt(vx * vx + vy * vy)
        if v > max_vel:
            max_vel = v
        if v < min_vel:
            min_vel = v

    print("obstacle count: %d  " % count)
    print("obstacle volumn: %.4f  " % obstacle_volum)
    print("obstacle density: %.6f percent " % (100 * obstacle_volum / map_volumn))
    print("max velocity: %.4f  " % max_vel)
    print("min velocity: %.4f  " % min_vel)
    rospy.signal_shutdown("Done")


if __name__ == "__main__":
    rospy.init_node("cal_obstacle_density", anonymous=True)
    map_x = map_y = map_z = 1  # map size
    map_x = rospy.get_param("/map_generator/map/x_size")
    map_y = rospy.get_param("/map_generator/map/y_size")
    map_z = rospy.get_param("/map_generator/map/z_size")
    map_volumn = map_x * map_y
    print("map volumn: %.4f  " % map_volumn)

    sub = rospy.Subscriber(
        "/map_generator/global_cylinder_state", MarkerArray, obsCallback
    )

    rospy.spin()
