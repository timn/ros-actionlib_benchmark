
----------------------------------------------------------------------------
--  roundtrip_client.lua - Lua roundtrip test action client
--
--  Created: Wed Aug 11 18:00:00 2010 (at Intel Research, Pittsburgh)
--  Copyright  2010  Tim Niemueller [www.niemueller.de]
--
----------------------------------------------------------------------------

-- Licensed under BSD license

require("roslua")
require("actionlib")

roslua.init_node{master_uri=os.getenv("ROS_MASTER_URI"), node_name="/roundtrip_client"}

print()
print("Action client roundtrip benchmark")

print()
local acl = actionlib.action_client("/roundtrip", "actionlib_benchmark/Roundtrip")
acl:wait_for_server()

local goal = acl.actspec.goal_spec:instantiate()
goal.values.start = roslua.Time.now()

function calc_roundtrip(gh)
   if gh.state == gh.SUCCEEDED then
      local now = roslua.Time.now()
      local difftime_send  = gh.result.values.result.values.received - goal.values.start
      local difftime_recv  = now - gh.result.values.result.values.received
      local difftime_total = now - goal.values.start
      printf("Sent:            %s", tostring(goal.values.start))
      printf("Received:        %s", tostring(gh.result.values.result.values.received))
      printf("Now:             %s", tostring(now))
      printf("Sending time:    %f", difftime_send:to_sec())
      printf("Receiving time:  %f", difftime_recv:to_sec())
      printf("Round trip time: %f", difftime_total:to_sec())
      roslua.exit()
   end
end

acl:send_goal(goal, calc_roundtrip)

roslua.run()
