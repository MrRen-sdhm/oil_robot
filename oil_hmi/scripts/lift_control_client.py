#!/usr/bin/env python

import rospy
from lift_driver.srv import *


def lift_control_client(command, pose):
    if pose is None:
        pose = -1
    else:
        pose = int(pose)

    rospy.wait_for_service('lift_control')
    try:
        lift_control = rospy.ServiceProxy('lift_control', LiftCtl)
        resp = lift_control(command, pose)
        return resp.success
    except rospy.ServiceException, e:
        print "[SRVICE] Service call failed: %s" % e


def lift_status_client(command):
    rospy.wait_for_service('lift_status')
    try:
        lift_status = rospy.ServiceProxy('lift_status', LiftStat)
        resp = lift_status(command)
        return resp.success
    except rospy.ServiceException, e:
        print "[SRVICE] Service call failed: %s" % e


def lift_pose_client(command):
    rospy.wait_for_service('lift_pose')
    try:
        lift_pose = rospy.ServiceProxy('lift_pose', LiftPose)
        resp = lift_pose(command)
        return resp.pose
    except rospy.ServiceException, e:
        print "[SRVICE] Service call failed: %s" % e


def usage():
    return "%s [command] Supported command: Open/Close/Move/MoveUp/MoveDown" % sys.argv[0]


if __name__ == "__main__":
    pose = None
    if len(sys.argv) == 2:
        command = sys.argv[1]
    elif len(sys.argv) == 3:
        command = sys.argv[1]
        pose = sys.argv[2]
    else:
        print usage()
        sys.exit(1)

    if (command != "Stop") and (command != "Home") and (command != "Running") and (command != "Move") and (command != "MoveUp") and (command != "MoveDown") :
        print "[ERROR] Supported command: Stop/Home/Running/Move/MoveUp/MoveDown"
    elif command == "Move" and pose == None:
        print "[ERROR] Please specify the pose [0-24000]"
    else:
        print "[SRVICE] Requesting %s" % command
        print "[SRVICE] Result = %s" % lift_control_client(command, pose)

