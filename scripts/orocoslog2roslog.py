#!/usr/bin/env python

import rospy
import re

# This file reads the orocos log line by line and converts them to ROS log statements
rospy.init_node('hardwarelog', anonymous=True)
rate = rospy.Rate(10)  # 10hz

logfile = '/home/amigo/.ros/orocos.log'

# Typical line: "5.616 [ Info   ][Thread] Supervisor: RGB_light_controller: Peer can be succesfully added"
pattern = re.compile(r'\d+\.\d\d\d\s\[\s*(?P<severity>\w+)\s*\](?P<logline>.+)')

# Find the ending of the current log file (no use in repeating yesterdays session)
with open(logfile) as f:
    f.seek(0, 2)
    cur = f.tell()

# Continue to open new logfiles because restarts of hardware might happen
while not rospy.is_shutdown():
    with open(logfile) as f:
        f.seek(0, 2)
        if f.tell() < cur:
            f.seek(0, 0)
        else:
            f.seek(cur, 0)
        for line in f:
            if not line.strip(): # Skip empty lines
                result = pattern.search(line.decode('utf-8'))
                if result is not None:
                    severity = result.group('severity').lower()
                    logline = result.group('logline')
                    if severity == "debug":
                        rospy.logdebug(logline)
                    elif severity == "info":
                        rospy.loginfo(logline)
                    elif severity == "warning":
                        rospy.logwarn(logline)
                    elif severity == "error":
                        rospy.logerr(logline)
                    elif severity == "fatal":
                        rospy.logfatal(logline)
                    else:
                        rospy.logerr('Unrecognized severity: ' + severity)
                else:
                    rospy.logerr("Unparsable error from orocos: %s",line)
        cur = f.tell()
    rate.sleep()
