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
import sys

# Brings in the SimpleActionClient
import actionlib

import actionlib_benchmark.msg

def roundtrip_client(num_runs):
    client = actionlib.SimpleActionClient('roundtrip', actionlib_benchmark.msg.RoundtripAction)

    client.wait_for_server()

    results = []
    rate = rospy.Rate(20)

    for i in range(num_runs):
        goal = actionlib_benchmark.msg.RoundtripGoal(start=rospy.Time.now())
        client.send_goal(goal)
        client.wait_for_result()

        result = client.get_result()

        now = rospy.Time.now()
        if num_runs == 1:
            difftime_send  = (result.received - goal.start).to_sec()
            difftime_recv  = (now - result.received).to_sec()
            difftime_total = (now - goal.start).to_sec()
            print("Sending time:    %f" % difftime_send)
            print("Receiving time:  %f" % difftime_recv)
            print("Round trip time: %f" % difftime_total)
        else:
            res = { "start": goal.start, "rcvd": result.received, "now": now }
            results.append(res)
            rate.sleep()

    if num_runs > 1:
        # calc results
        average_send  = 0
        average_recv  = 0
        average_total = 0
        n = len(results)

        for r in results:
            average_send  += (r["rcvd"] - r["start"]).to_sec()
            average_recv  += (r["now"]  - r["rcvd"]).to_sec()
            average_total += (r["now"]  - r["start"]).to_sec()
        average_send  /= n
        average_recv  /= n
        average_total /= n

        deviation_send  = 0
        deviation_recv  = 0
        deviation_total = 0
        for r in results:
            deviation_send  += abs((r["rcvd"] - r["start"]).to_sec() - average_send)
            deviation_recv  += abs((r["now"]  - r["rcvd"]).to_sec() - average_recv)
            deviation_total += abs((r["now"]  - r["start"]).to_sec() - average_total)
        deviation_send  /= n
        deviation_recv  /= n
        deviation_total /= n

        print("SEND   Average: %f    Deviation: %f" % (average_send, deviation_send))
        print("RECV   Average: %f    Deviation: %f" % (average_recv, deviation_recv))
        print("TOTAL  Average: %f    Deviation: %f" % (average_total, deviation_total))


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('roundtrip_client')
        num_runs = 1
        if len(sys.argv) == 2:
            num_runs = int(sys.argv[1])
        roundtrip_client(num_runs)

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
