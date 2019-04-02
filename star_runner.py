#!/usr/bin/env python
import os, subprocess
import rospy, rosgraph
import random, math
from bwi_tools import start_roslaunch_process, stop_roslaunch_process

import std_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from bwi_msgs.srv import DoorHandlerInterface
from move_base_msgs.msg import MoveBaseActionResult

LOGFILE = "star.log"

def log(msg):
    with open(LOGFILE, "a+") as f:
        f.write(str(msg) + "\n")

START = (15.0, 110.0)
POINTS = [(23.73, 109.22), 
          (-1.98, 112.47), 
          (-1.77, 104.77), 
          (34.92, 108.36), 
          (37.00, 101.64), 
          (44.84, 108.27), 
          (51.25, 113.38), 
          (52.88, 106.3)]

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

# Using global variable until we can find a better solution.
timeStart, timeEnd = None, None

def clearTimes():
    global timeStart, timeEnd
    timeStart, timeEnd = None, None

def runStarted(data):
    global timeStart
    #print("---- RUN STARTED WITH DATA %s ---- " % (data.header.stamp))
    timeStart = rospy.Time.now()

def runEnded(data):
    global timeEnd
    #print("---- RUN ENDED WITH DATA %s ---- " % (data.header.stamp))
    timeEnd = rospy.Time.now()

def time_point(point):
    process = None

    try:
        process = start_roslaunch_process("bwi_launch", "simulation_v2.launch")
        pub = rospy.Publisher('/move_base_interruptable_simple/goal', PoseStamped, queue_size=3)
        sub_start = rospy.Subscriber('/move_base_interruptable_simple/goal', PoseStamped, runStarted)
        sub_end = rospy.Subscriber('/move_base_interruptable/result', MoveBaseActionResult, runEnded)
        clearTimes()
        rospy.sleep(15)

        open_all_doors()
        pub.publish(make_pose_stamped(*point))

        # Wait for run to finish.
        r = rospy.Rate(5)
        while timeStart == None or timeEnd == None:
            r.sleep()

        stop_roslaunch_process(process)
        rospy.sleep(18)
        return timeEnd - timeStart

    except rospy.ROSInterruptException:
        pass

    if process is not None:
        stop_roslaunch_process(process)
        return False

def time_all_points():
    # Euclidean distance between two points. 
    dist = lambda p1, p2: math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))

    for point in POINTS:
        log("Timing point " + str(point) + "...")
        t = time_point(point)
        dist_from_start = dist(START, point)
        log(str(dist_from_start) + ", " + str(t))


if __name__  == '__main__':
    if not rosgraph.is_master_online():
        roscore()
    rospy.init_node('star_publisher')
    time_all_points()

