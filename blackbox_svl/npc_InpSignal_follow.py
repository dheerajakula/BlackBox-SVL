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

from lgsvl.geometry import Transform



def npc_InpSignal_follow(json_file_name,state,sim,steptime,InpSignal):
    npc_state = copy.deepcopy(state)
    npc = None
    waypoints = []
    speed = 1
    # Read json file to get the initial position of the NPC
    with open(json_file_name) as json_data:
        d = json.load(json_data)
        for agent in d['agents']:
            if(agent['type'] == 2):
                transform = agent['transform']
                pos = lgsvl.Vector(transform['position']['x'], transform['position']['y'], transform['position']['z'])
                angle = lgsvl.Vector(transform['rotation']['x'], transform['rotation']['y'], transform['rotation']['z'])
                npc_state.transform.position = pos
                npc_state.transform.rotation = angle
                npc = sim.add_agent(agent['variant'], lgsvl.AgentType.NPC, npc_state)
    
    
    forward = lgsvl.utils.transform_to_forward(npc_state.transform)
    for i,inp in enumerate(InpSignal):
        pos = npc.state.transform.position + forward * inp
        angle = npc.state.transform.rotation
        wp = lgsvl.DriveWaypoint(position= pos, angle = angle,speed=100, timestamp=steptime[i])
        waypoints.append(wp)
    npc.follow(waypoints)
                