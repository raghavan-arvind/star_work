#!/usr/bin/env python
import os, subprocess
import rospy, rosgraph
import random
from bwi_tools import start_roslaunch_process, stop_roslaunch_process

import std_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from bwi_msgs.srv import DoorHandlerInterface

rooms = {'r2': (8.96481704712, 115.404937744), 'r4': (24.3392028809, 115.429176331), 'r14': (36.7606925964, 108.823013306), 'r11': (41.097858429, 108.535583496), 'r10': (45.0729446411, 108.629150391), 'r24': (14.609085083, 108.930099487), 'r20': (27.9855327606, 101.263656616), 'r8': (47.2909011841, 115.680419922), 'r25': (14.2544584274, 116.043502808), 'r17': (24.0002746582, 108.502052307), 'r12': (54.8262786865, 106.72668457), 'r6': (32.1917800903, 115.396934509), 'r1': (5.29522514343, 115.758872986), 'r23': (15.3893146515, 102.381370544), 'r5': (28.5052490234, 115.402694702), 'r26': (8.9367389679, 101.593757629), 'r21': (24.374458313, 101.63004303), 'r9': (52.9659118652, 115.366699219), 'r27': (9.1826210022, 108.222724915), 'r19': (32.3290367126, 101.768623352), 'r28': (39.008392334, 114.981956482), 'r3': (20.504863739, 115.55393219), 'r16': (29.9390983582, 109.440086365), 'r13': (45.0136451721, 102.41947937), 'r22': (20.5140609741, 102.019271851), 'r15': (38.2078781128, 100.458351135), 'r18': (18.7453746796, 109.395431519), 'r7': (43.5528831482, 115.033187866)}

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
