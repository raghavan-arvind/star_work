#!/usr/bin/env python
import os, subprocess
import rospy, rosgraph
import random, math
from bwi_tools import start_roslaunch_process, stop_roslaunch_process

import std_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Twist
from bwi_msgs.srv import DoorHandlerInterface
from nav_msgs.msg import Odometry
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

def rostime():
    return round(rospy.Time.now().secs + rospy.Time.now().nsecs * (10 ** -9), 3)

def make_pose_stamped(x, y):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "level_mux_map"
    pose.pose.position.x = x
    pose.pose.position.y = y

    pose.pose.orientation.z = 1.0

    return pose

# Using global variables until we can find a better solution.
timeStart, timeLastTurn, timeEnd, process, pub, vel, odom, curX, curY, lastX, lastY, goal = [None] * 12
subStart, sub_end = [None] * 2
started, stopped = [False] * 2

def runEnded(data):
    global started, stopped
    if data.status == 3 and started and not stopped:
        timeEnd = rostime()
        stopped = True
        print("--- TIME OVERRIDED BY SUB: %s\nMESSAGE: %s" % (timeEnd, data))

def odometer(data):
    global curX, curY
    curX = data.pose.pose.position.x
    curY = data.pose.pose.position.y

def speedometer(data):
    global timeStart, timeLastTurn, timeEnd, started, stopped, curX, curY, lastX, lastY, goal

    lin, ang = [data.linear.x, data.linear.y, data.linear.z], [data.angular.x, data.angular.y, data.angular.z]

    moving = any(x != 0 for x in lin)
    turning = any(x != 0 for x in ang)
    completely_still = not moving and not turning
    in_thresh = lambda a, b: abs(a-b) <= 0.8

    def is_turn_sequence():
        global lastX, lastY, curX, curY
        if turning:
            if lastX != None and lastY != None and in_thresh(curX, lastX) and in_thresh(curY, lastY):
                # If we've already seen this turn sequence, ignore.
                return False

            lastX, lastY = curX, curY
            return True
        return False

    if started and not stopped:
        if timeStart == None:
            if moving:
                timeStart = rostime()
                print("--- TIME START: ", timeStart)
        else:
            if is_turn_sequence:
                timeLastTurn = rostime()
                #print("--- TIME LAST TURN: ", timeLastTurn)

            if completely_still and in_thresh(goal[0], curX) and in_thresh(goal[1], curY):
                timeEnd = timeLastTurn if timeLastTurn is not None else rostime()
                stopped = True
                print("--- TIME END: ", timeEnd)


def clearTimes():
    global timeStart, timeLastTurn, timeEnd, goal, started, stopped, lastX, lastY, goal, subStart
    timeStart, timeLastTurn, timeEnd, goal, lastX, lastY, goal = [None] * 7
    started, stopped = [False] * 2
    subStart = None

def stop_process():
    global process
    if process is not None:
        stop_roslaunch_process(process)
        rospy.sleep(5)
atexit.register(stop_process)

def time_point(start, end, POLL_RATE=5, TIMEOUT=15):
    global pub, timeStart, timeEnd, subStart, started, stopped, goal

    # Used for polling for completion.
    r = rospy.Rate(POLL_RATE)
    t, TIMEOUT_ITERS = 0, TIMEOUT * POLL_RATE

    # Get to start location.
    clearTimes()
    started, goal, subStart = True, start, rostime()
    pub.publish(make_pose_stamped(*start))

    while not stopped or timeEnd is None:
        r.sleep()
        t += 1
        if t > TIMEOUT_ITERS:
            break
    t = 0
    rospy.sleep(1)

    # Time how long it takes to get to end.
    clearTimes()
    started, goal, subStart = True, end, rostime()
    pub.publish(make_pose_stamped(*end))
    while not stopped or timeEnd is None:
        r.sleep()
        t += 1
        if t > TIMEOUT_ITERS: return False

    # If we calculated a precise start/end time using cmd_vel, use that.
    # Otherwise use the basic measurements.

    true_start = timeStart if timeStart != None else subStart
    true_end = timeEnd   # is overwritten by sub_end if cmd_vel does not find end

    return true_end - true_start

def time_all_points():
    adj, coords = points.adj, points.coords

    # Euclidean distance between two points. 
    dist = lambda p1, p2: math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))
    tot = sum([len(adj[u]) for u in adj]) / 2
    done,fin = 0, set()

    edge = lambda u, v: tuple(sorted([u, v]))

    for u in adj:
        for v in adj[u]:    
            if edge(u,v) not in fin:
                log("Timing point " + str(u) + " -> " + str(v) + " ...")
                t = time_point(coords[u], coords[v])
                while t is False:
                    print("Failed to time point, retiming!")
                    t = time_point(coords[u], coords[v])
                log("Took: " + str(t) + " !")
                log("Distance was: " + str(dist(coords[u], coords[v])))

                done += 1
                fin.add(edge(u,v))
                log("Finished %s out of %s" % (done, tot))

if __name__  == '__main__':
    if not rosgraph.is_master_online():
        roscore()
    rospy.init_node('star_publisher')

    # Start simulation
    process = start_roslaunch_process("bwi_launch", "simulation_v2.launch")
    pub = rospy.Publisher('/move_base_interruptable_simple/goal', PoseStamped, queue_size=3)
    sub_end = rospy.Subscriber('/move_base_interruptable/result', MoveBaseActionResult, runEnded)
    odom = rospy.Subscriber('/odom', Odometry, odometer)
    vel = rospy.Subscriber('/cmd_vel', Twist, speedometer)
    clearTimes()
    rospy.sleep(20)

    open_all_doors()
    time_all_points()
