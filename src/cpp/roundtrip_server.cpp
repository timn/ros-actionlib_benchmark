
/***************************************************************************
 *  rosspawn.cpp - ROSspawn main application
 *
 *  Created: Tue Aug  3 17:06:44 2010
 *  Copyright  2010  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_benchmark/RoundtripAction.h>

class RoundtripAction
{
public:
    
  RoundtripAction(std::string name) : 
    __as(__node_handle, name, boost::bind(&RoundtripAction::executeCB, this, _1)),
    __action_name(name)
  {
  }

  void executeCB(const actionlib_benchmark::RoundtripGoalConstPtr &goal)
  {
    // publish info to the console for the user
    //ROS_INFO("%s", __action_name.c_str());
        
    __result.received = ros::Time::now();
    __as.setSucceeded(__result);
  }

protected:
  ros::NodeHandle __node_handle;
  actionlib::SimpleActionServer<actionlib_benchmark::RoundtripAction> __as;
  std::string __action_name;
  // create messages that are used to published feedback/result
  actionlib_benchmark::RoundtripResult __result;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "roundtrip");

  RoundtripAction roundtrip(ros::this_node::getName());
  ros::spin();

  return 0;
}
