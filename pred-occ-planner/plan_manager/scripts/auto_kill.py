#! /usr/bin/python3

"""
auto kill all nodes when landing detected
"""

import math
from random import uniform
from std_msgs.msg import Header
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
import rospy
import os

rospy.init_node("auto_kill_all_nodes")
# start_time = rospy.Time.now()


def uav_status_callback(data):
    # now = rospy.Time.now()
    # if (now - start_time).to_sec() > 300:
    #     rospy.logerr()("TIME OUT!")
    #     os.system("rosnode kill -a")
    rospy.loginfo("test over!")
    os.system("rosnode kill -a")


def main():
    """
    use rosnode kill -a to kill all nodes
    """

    uav_status_sub = rospy.Subscriber(
        "/test_over", PoseStamped, uav_status_callback, queue_size=1
    )

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
