#! /usr/bin/env python

#---------------------------------------------------------------------------
#  roundtrip_client.py - Python roundtrip test action client
#
#  Created: Wed Aug 11 18:56:23 2010 (at Intel Research, Pittsburgh)
#  Copyright  2010  Tim Niemueller [www.niemueller.de]
#
#---------------------------------------------------------------------------

#  Licensed under BSD license


import roslib; roslib.load_manifest('actionlib_benchmark')
import rospy

# Brings in the SimpleActionClient
import actionlib

import actionlib_benchmark.msg

def roundtrip_client():
    client = actionlib.SimpleActionClient('roundtrip', actionlib_benchmark.msg.RoundtripAction)

    client.wait_for_server()

    goal = actionlib_benchmark.msg.RoundtripGoal(start=rospy.Time.now())
    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()

    now = rospy.Time.now()
    difftime_send  = (result.received - goal.start).to_sec()
    difftime_recv  = (now - result.received).to_sec()
    difftime_total = (now - goal.start).to_sec()
    print("Sending time:    %f" % difftime_send)
    print("Receiving time:  %f" % difftime_recv)
    print("Round trip time: %f" % difftime_total)


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('roundtrip_client')
        roundtrip_client()

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
