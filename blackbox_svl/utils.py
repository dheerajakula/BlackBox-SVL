import copy
import json
import gzip
import lgsvl
import math
from environs import Env

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
        return final_data

def npc_follow_inpsignal(json_file_name, state, sim, steptime, InpSignal_Forward, InpSignal_Rightward):
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
    rightward = lgsvl.utils.transform_to_right(npc_state.transform)
    # Create waypoints based on the input signal
    length_of_signal = max(len(InpSignal_Forward), len(InpSignal_Rightward))
    for i in range(0,length_of_signal):
        inp_forward = InpSignal_Forward[i] if InpSignal_Forward else 0
        inp_rightward = InpSignal_Rightward[i] if InpSignal_Rightward else 0
        pos = npc.state.transform.position + forward * inp_forward + rightward * inp_rightward
        angle_change = math.degrees(math.atan(inp_rightward/(inp_forward+1e-6)))
        angle = npc.state.transform.rotation + lgsvl.Vector(0, angle_change, 0)
        wp = lgsvl.DriveWaypoint(position = pos, angle = angle, speed=100, timestamp=steptime[i])
        waypoints.append(wp)
    npc.follow(waypoints)