
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
require("socket")

roslua.init_node{master_uri=os.getenv("ROS_MASTER_URI"), node_name="/roundtrip_client"}

local num_runs = tonumber(arg[1]) or 1

print()
printf("Action client roundtrip benchmark (%d runs)", num_runs)

print()
local acl = actionlib.action_client("/roundtrip", "actionlib_benchmark/Roundtrip")
acl:wait_for_server()

local results = {}
for i = 1, num_runs do
   local goal = acl.actspec.goal_spec:instantiate()
   goal.values.start = roslua.Time.now()
   acl:send_goal(goal)
   acl:wait_for_result()

   local result = acl.result

   local start = goal.values.start
   local rcvd = roslua.Time:new(result.values.result.values.received[1],
				result.values.result.values.received[2])
   local now = roslua.Time.now()
   if num_runs == 1 then
      -- we only want the output and exit if there is only one run
      local difftime_send  = rcvd - start
      local difftime_recv  = now - rcvd
      local difftime_total = now - start
      printf("Sent:            %s (%d.%d)", tostring(start), start.sec, start.nsec)
      printf("Received:        %s (%d.%d)", tostring(rcvd), rcvd.sec, rcvd.nsec)
      printf("Now:             %s (%d.%d)", tostring(now), now.sec, now.nsec)
      printf("Sending time:    %f", difftime_send:to_sec())
      printf("Receiving time:  %f", difftime_recv:to_sec())
      printf("Round trip time: %f", difftime_total:to_sec())
      roslua.exit()
   else -- multiple runs
      table.insert(results, {start=start, rcvd=rcvd, now=now})
   end

   roslua.sleep(0.05)
end

if num_runs > 1 then
   local average_send    = 0
   local average_recv    = 0
   local average_total   = 0
   local deviation_send  = 0
   local deviation_recv  = 0
   local deviation_total = 0
   local n = #results

   for _, t in ipairs(results) do
      average_send  = average_send  + (t.rcvd - t.start):to_sec()
      average_recv  = average_recv  + (t.now  - t.rcvd):to_sec()
      average_total = average_total + (t.now  - t.start):to_sec()
   end
   average_send    = average_send / n
   average_recv    = average_recv / n
   average_total   = average_total / n
	 
   for _, t in ipairs(results) do
      deviation_send = deviation_send +
	 math.abs((t.rcvd - t.start):to_sec() - average_send)
      deviation_recv = deviation_recv +
	 math.abs((t.now - t.rcvd):to_sec() - average_recv)
      deviation_total = deviation_total +
	 math.abs((t.now - t.start):to_sec() - average_total)
   end
   deviation_send  = deviation_send / n
   deviation_recv  = deviation_recv / n
   deviation_total = deviation_total / n

   printf("SEND   Average: %f    Deviation: %f", average_send, deviation_send)
   printf("RECV   Average: %f    Deviation: %f", average_recv, deviation_recv)
   printf("TOTAL  Average: %f    Deviation: %f", average_total, deviation_total)
end

roslua.finalize()
