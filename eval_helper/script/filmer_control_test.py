#!/usr/bin/env python

from os import wait
import roslib

roslib.load_manifest("view_controller_msgs")
import numpy as np
import rospy
from math import cos, sin, pi
from view_controller_msgs.msg import CameraPlacement, CameraTrajectory, CameraMovement
from geometry_msgs.msg import Point, Vector3

rospy.init_node("camera_test", anonymous=True)

pub = rospy.Publisher("/rviz/camera_placement", CameraPlacement, queue_size=1)
# pub = rospy.Publisher("/rviz/camera_trajectory", CameraTrajectory, queue_size=1)

T = 60
dt = 0.2
# dt = 1
rate_float = 1 / dt
rate = rospy.Rate(rate_float)

# ct = CameraTrajectory()

while not rospy.is_shutdown():

    # print "Top of loop!"

    t = rospy.get_time()
    cp = CameraPlacement()

    r = 15
    p = Point(r * cos(2 * pi * t / T), r * sin(2 * pi * t / T), 20)
    cp.eye.point = p
    cp.eye.header.frame_id = "base_link"

    f = Point(-5 * cos(2 * pi * t / T), -5 * sin(2 * pi * t / T), -10)
    cp.focus.point = f
    cp.focus.header.frame_id = "base_link"

    up = Vector3(0, 0, 3)
    cp.up.vector = up
    cp.up.header.frame_id = "base_link"
    cp.interpolation_mode = 1

    cp.time_from_start = rospy.Duration(dt)
    pub.publish(cp)
    rate.sleep()

# while not rospy.is_shutdown():
#     cm = CameraMovement()
#     ct.trajectory = []
#     for t in range(0, T, dt):
#         r = 15
#         p = Point(r * cos(2 * pi * t / T), r * sin(2 * pi * t / T), 20)
#         cm.eye.point = p
#         cm.eye.header.frame_id = "base_link"

#         f = Point(-5 * cos(2 * pi * t / T), -5 * sin(2 * pi * t / T), -10)
#         cm.focus.point = f
#         cm.focus.header.frame_id = "base_link"

#         up = Vector3(0, 0, 3)
#         cm.up.vector = up
#         cm.up.header.frame_id = "base_link"
#         cm.transition_duration = rospy.Duration(dt)

#         cm.interpolation_speed = 2
#         ct.trajectory.append(cm)

#     ct.render_frame_by_frame = True
#     ct.target_frame = "base_link"
#     print("Publishing a message!")
#     pub.publish(ct)
#     rate.sleep()
