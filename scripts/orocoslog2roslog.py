#!/usr/bin/env python

import rospy

# This file reads the orocos log line by line and converts them to ROS log statements
rospy.init_node('hardwarelog', anonymous=True)
rate = rospy.Rate(10) # 10hz

logfile = '/home/amigo/.ros/orocos.log'

# Find the ending of the current log file (no use in repeating yesterdays session
with open(logfile) as f:
    f.seek(0, 2)
    cur=f.tell()

# Continue to open new logfiles because restarts of hardware might happen
while not rospy.is_shutdown():
    with open(logfile) as f:
        f.seek(0,2)
        if f.tell() < cur:
            f.seek(0,0)
        else:
            f.seek(cur,0)
        for line in f:
            if line[7:15].strip().lower() == "debug":
                rospy.logdebug(line[17:].strip())
            if line[7:15].strip().lower() == "info":
                rospy.loginfo(line[17:].strip())
            if line[7:15].strip().lower() == "warning":
                rospy.logwarn(line[17:].strip())
            if line[7:15].strip().lower() == "error":
                rospy.logerr(line[17:].strip())
            if line[7:15].strip().lower() == "fatal":
                rospy.logfatal(line[17:].strip())
        cur = f.tell()
    rate.sleep()
