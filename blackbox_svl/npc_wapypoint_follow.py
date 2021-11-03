#!/usr/bin/env python3
#
# Copyright (c) 2019-2021 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

import copy
from environs import Env
import lgsvl
import json



def deserialize_npc_behavior(json_file_name,state,sim):
    # This block creates the list of waypoints that the NPC will follow
    # Each waypoint is an position vector paired with the speed that the NPC will drive to it
    waypoints = []
    # Read json file to get waypoints
    with open(json_file_name) as json_data:
        d = json.load(json_data)
        for agent in d['agents']:
            if(agent['type'] == 2):
                print(agent['variant'])
                for waypoint in agent['waypoints']:
                    speed = 6
                    print(waypoint['position'])
                    pos = lgsvl.Vector(waypoint['position']['x'], waypoint['position']['y'], waypoint['position']['z'])
                    angle = lgsvl.Vector(waypoint['angle']['x'], waypoint['angle']['y'], waypoint['angle']['z'])
                    if(waypoint['ordinalNumber'] == 0):
                        npc_state = copy.deepcopy(state)
                        npc_state.transform.position = pos
                        npc_state.transform.rotation = angle
                        npc = sim.add_agent(agent['variant'], lgsvl.AgentType.NPC, npc_state)
                        continue
                    wp = lgsvl.DriveWaypoint(pos, speed, angle, 0)
                    waypoints.append(wp)
                npc.follow(waypoints)
                waypoints = []