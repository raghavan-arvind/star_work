#!/usr/bin/env python
import os, subprocess
import rospy, rosgraph
import random
from bwi_tools import start_roslaunch_process, stop_roslaunch_process

import std_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from bwi_msgs.srv import DoorHandlerInterface

POINTS = [(-1.77, 104.77),
          (-1.98, 112.47),
          (23.73, 109.22),
          (51.25, 113.38),
          (52.88, 106.30),
          (37.00, 101.64),
          (34.92, 108.36),
          (44.84, 108.27)]


def roscore():
    subprocess.Popen('roscore')

def open_all_doors():
    rospy.wait_for_service("update_doors")
    update_doors = rospy.ServiceProxy("update_doors", DoorHandlerInterface)
    res = update_doors("", True, True, 0)

    if not res.success:
        print("-- FAILED TO OPEN ALL DOORS! --")


def make_pose_stamped(x, y):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "level_mux_map"
    pose.pose.position.x = x
    pose.pose.position.y = y

    pose.pose.orientation.z = 1.0

    return pose

def star_publisher():
    process = None

    try:
        process = start_roslaunch_process("bwi_launch", "simulation_v2.launch")
        pub = rospy.Publisher('/move_base_interruptable_simple/goal', PoseStamped, queue_size=3)
        rospy.sleep(25)

        open_all_doors()
        pub.publish(make_pose_stamped(*random.choice(POINTS)))

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
