#!/usr/bin/env python
import os, subprocess
import rospy, rosgraph
import random, math
from bwi_tools import start_roslaunch_process, stop_roslaunch_process

import std_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from bwi_msgs.srv import DoorHandlerInterface
from move_base_msgs.msg import MoveBaseActionResult
import atexit
import points

LOGFILE = "star.log"

def log(msg):
    with open(LOGFILE, "a+") as f:
        f.write(str(msg) + "\n")

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

# Using global variables until we can find a better solution.
timeStart, timeEnd, process, pub, sub_start, sub_end = [None] * 6

def clearTimes():
    global timeStart, timeEnd
    timeStart, timeEnd = None, None

def runStarted(data):
    global timeStart
    #print("---- RUN STARTED WITH DATA %s ---- " % (data.header.stamp))
    timeStart = rospy.Time.now().secs

def runEnded(data):
    global timeEnd
    #print("---- RUN ENDED WITH DATA %s ---- " % (data.header.stamp))
    timeEnd = rospy.Time.now().secs

def stop_process():
    global process
    if process is not None:
        stop_roslaunch_process(process)
        rospy.sleep(18)

atexit.register(stop_process)

def time_point(start, end):
    global pub

    # Get to start location.
    clearTimes()
    pub.publish(make_pose_stamped(*start))
    r = rospy.Rate(5)
    while timeStart == None or timeEnd == None:
        r.sleep()

    # Time how long it takes to get to end.
    clearTimes()
    pub.publish(make_pose_stamped(*end))
    r = rospy.Rate(5)
    while timeStart == None or timeEnd == None:
        r.sleep()

    return timeEnd - timeStart

def time_all_points():
    adj, coords = points.adj, points.coords
    # Euclidean distance between two points. 
    dist = lambda p1, p2: math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))

    for u in adj:
        for v in adj[u]:    
            log("Timing point " + str(u) + "," + str(v) + "...")
            t = time_point(coords[u], coords[v])
            dist_from_start = dist(coords[u], coords[v])
            log(str(dist_from_start) + ", " + str(t))

if __name__  == '__main__':
    if not rosgraph.is_master_online():
        roscore()
    rospy.init_node('star_publisher')

    # Start simulation
    process = start_roslaunch_process("bwi_launch", "simulation_v2.launch")
    pub = rospy.Publisher('/move_base_interruptable_simple/goal', PoseStamped, queue_size=3)
    sub_start = rospy.Subscriber('/move_base_interruptable_simple/goal', PoseStamped, runStarted)
    sub_end = rospy.Subscriber('/move_base_interruptable/result', MoveBaseActionResult, runEnded)
    clearTimes()
    rospy.sleep(25)

    open_all_doors()
    time_all_points()

