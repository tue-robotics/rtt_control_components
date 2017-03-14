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
            severety = line[line.index('[')+1:line.index(']')-1].strip().lower()
            log = line[line.index(']')+1:].strip()
            if severety == "debug":
                rospy.logdebug(log)
            if severety == "info":
                rospy.loginfo(log)
            if severety == "warning":
                rospy.logwarn(log)
            if severety == "error":
                rospy.logerr(log)
            if severety == "fatal":
                rospy.logfatal(log)
        cur = f.tell()
    rate.sleep()
