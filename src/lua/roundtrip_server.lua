
----------------------------------------------------------------------------
--  roundtrip_server.lua - Lua roundtrip test action server
--
--  Created: Wed Aug 18 18:35:23 2010 (at Intel Research, Pittsburgh)
--  Copyright  2010  Tim Niemueller [www.niemueller.de]
--
----------------------------------------------------------------------------

-- Licensed under BSD license

require("roslua")
require("actionlib")

roslua.init_node{master_uri=os.getenv("ROS_MASTER_URI"), node_name="/roundtrip_server"}

function goal_cb(goal_handle, as)
   local result = as.actspec.result_spec:instantiate()
   result.values.received = roslua.Time.now()
   as:publish_result(goal_handle, result)
end

local as = actionlib.action_server("/roundtrip", "actionlib_benchmark/Roundtrip", goal_cb)

roslua.run(0)
