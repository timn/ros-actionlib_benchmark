#! /usr/bin/env python

#---------------------------------------------------------------------------
#  roundtrip_server.py - Python roundtrip test action server
#
#  Created: Wed Aug 11 19:12:41 2010 (at Intel Research, Pittsburgh)
#  Copyright  2010  Tim Niemueller [www.niemueller.de]
#
#---------------------------------------------------------------------------

#  Licensed under BSD license

import roslib; roslib.load_manifest('actionlib_benchmark')
import rospy

import actionlib

import actionlib_benchmark.msg

class RoundtripAction(object):
	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(
                    self._action_name,
                    actionlib_benchmark.msg.RoundtripAction, execute_cb=self.execute_cb)
		
	def execute_cb(self, goal):
            result = actionlib_benchmark.msg.RoundtripResult()
            result.received = rospy.Time.now()
            self._as.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('roundtrip')
    RoundtripAction(rospy.get_name())
    rospy.spin()
