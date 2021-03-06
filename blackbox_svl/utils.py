import copy
import json
import gzip
import lgsvl
import math
from environs import Env
import numpy as np

def set_lights_green(sim):
    controllables = sim.get_controllables("signal")
    for signal in controllables:
        # Create a new control policy
        control_policy = "trigger=500;green=300;yellow=0;red=0;loop"
        # Control this traffic light with a new control policy
        signal.control(control_policy)

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

def deserialize_log_file(log_file_path):
    # read every alternate line as json
    final_data = []
    # open file
    with gzip.open(log_file_path, 'r') as f:
        # read every alternate line as json
        # return data
        data = []
        for line in f:
            if line.startswith(str.encode('{')):
                data.append(json.loads(line))
        intial_frame_tick = data[0]
        intial_time = intial_frame_tick['Time']

        for frame_tick in data:
            per_frame_data = [0.0 for i in range(11)]
            elapsed_time = frame_tick['Time'] - intial_time
            per_frame_data[0] = elapsed_time
            for vehicle in frame_tick['Data']:
                if vehicle['Id'] == 1:
                    ego_position = vehicle['Position']
                    per_frame_data[1] = ego_position['x']
                    per_frame_data[2] = ego_position['z']
                    ego_rotation = vehicle['Rotation']
                    per_frame_data[3] = ego_rotation['w']
                    ego_velocity = vehicle['LinearVelocity']
                    per_frame_data[4] = ego_velocity['x']
                if vehicle['Id'] > 2:
                    npc_position = vehicle['Position']
                    per_frame_data[5] = npc_position['x']
                    per_frame_data[6] = npc_position['z']
                    npc_rotation = vehicle['Rotation']
                    per_frame_data[7] = npc_rotation['w']
                    npc_velocity = vehicle['LinearVelocity']
                    per_frame_data[8] = npc_velocity['x']
            final_data.append(per_frame_data)
        # final data has the format [time, ego_x, ego_z, ego_rotation, ego_velocity, npc_x, npc_z, npc_rotation, npc_velocity]
        return final_data