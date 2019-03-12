#!/usr/bin/env python
import os, subprocess
import rospy, rosgraph
import time
from bwi_tools import start_roslaunch_process, stop_roslaunch_process

import std_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

def roscore():
    subprocess.Popen('roscore')

def make_pose_stamped(x, y):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "level_mux_map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 1.0
    pose.pose.orientation.w = 0.0

    return pose

def star_publisher():
    process = None

    try:
        process = start_roslaunch_process("bwi_launch", "simulation_v2.launch")
        time.sleep(30)

        pub = rospy.Publisher('/move_base_interruptable_simple/goal', PoseStamped, queue_size=3)
        pub.publish(make_pose_stamped(8, 108))


        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    if process is not None:
        stop_roslaunch_process(process)


if __name__  == '__main__':
    if not rosgraph.is_master_online():
        roscore()
    rospy.init_node('star_publisher')
    star_publisher()
