#!/usr/bin/env python3
#
# Copyright (c) 2019-2021 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#
from npc_InpSignal_follow import npc_InpSignal_follow
import os
from environs import Env
import lgsvl
from npc_wapypoint_follow import deserialize_npc_behavior
from log_reader import deserialize_log_file
from utils import set_lights_green
from random import randrange

env = Env()
sim = lgsvl.Simulator(env.str("LGSVL__SIMULATOR_HOST", lgsvl.wise.SimulatorSettings.simulator_host), env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port))



def run_test(ego_init_speed_m_s=5.0, ego_x_pos=5.0, pedestrian_speed=3.0, steptime=None, Inpsignal=None, sim_duration=5, for_matlab=False):
    
    # if sim.current_scene == lgsvl.wise.DefaultAssets.map_borregasave:
    if sim.current_scene == "5d272540-f689-4355-83c7-03bf11b6865f":    
        sim.reset()
    else:
        #sim.load(lgsvl.wise.DefaultAssets.map_borregasave, 42)
        sim.load("5d272540-f689-4355-83c7-03bf11b6865f", 42)

    spawns = sim.get_spawn()
    state = lgsvl.AgentState()
    state.transform = spawns[0]
    # print(spawns[0])
    # print(state.transform.position)
    forward = lgsvl.utils.transform_to_forward(spawns[0])
    right = lgsvl.utils.transform_to_right(spawns[0])
    
    state.transform.position += ego_x_pos * forward

    # Agents can be spawned with a velocity. Default is to spawn with 0 velocity
    state.velocity = ego_init_speed_m_s * forward

    ego = sim.add_agent("2e9095fa-c9b9-4f3f-8d7d-65fa2bb03921", lgsvl.AgentType.EGO, state)
    
    BRIDGE_HOST = "10.218.101.181"
    # Dreamview setup
    dv = lgsvl.dreamview.Connection(sim, ego, BRIDGE_HOST)
    dv.set_hd_map('San Francisco')
    dv.set_vehicle('Lincoln2017MKZ LGSVL')
    modules = [
        'Localization',
        'Transform',
        'Routing',
        'Prediction',
        'Planning',
        'Control'
    ]
    destination = spawns[0].destinations[1]
    
    dv.enable_apollo(destination.position.x, destination.position.z, modules)


    # connect ego vehicle with our apollo bridge
    BRIDGE_HOST = "10.218.101.181"
    BRIDGE_PORT = 9090
    ego.connect_bridge(BRIDGE_HOST, BRIDGE_PORT)
    # change the state so that dummy ego is behind the ego
    spawns = sim.get_spawn()
    dummy_state = lgsvl.AgentState()
    dummy_state.transform = spawns[0]
    
    dummy_state.transform.position += 5*right
    # spawn a dummy ego to collect the trace of the ego vehicle. 
    dummy_ego = sim.add_agent("34c713df-2fd3-43f7-8756-9576735013f1", lgsvl.AgentType.EGO, dummy_state)
   

    # connect dummy ego vehicle with our logging bridge
    # dummy host and dummy port are used to connect to the simulator
    BRIDGE_HOST_LOG = os.environ.get("LGSVL__AUTOPILOT_0_HOST", "LOG")
    BRIDGE_PORT_LOG = int(os.environ.get("LGSVL__AUTOPILOT_0_PORT", 9090))
    dummy_ego.connect_bridge(BRIDGE_HOST_LOG,BRIDGE_PORT_LOG)

    #The bounding box of an agent are 2 points (min and max) such that the box formed from those 2 points completely encases the agent
    #print("Vehicle bounding box =", ego.bounding_box)

    
    # print("Current starting time = ", sim.current_time)
    # print("Current starting frame = ", sim.current_frame)
    

    # get the npc behavior by reading the visual scenario editor file 
    # deserialize_npc_behavior("falsification.json", state, sim)

    # get the npc behavior by reading the InpSignal from STALIRO
    #npc_InpSignal_follow("falsification.json", state, sim, steptime, Inpsignal)
    npc_InpSignal_follow("falsification2.json", state, sim, steptime, Inpsignal)
    #set_lights_green(sim)
    # The simulator can be run for a set amount of time. time_limit is optional and if omitted or set to 0, then the simulator will run indefinitely
    sim.run(time_limit=sim_duration)
    
    sim.stop()
    BRIDGE_HOST = os.environ.get("LGSVL__AUTOPILOT_0_HOST", "DISCONNECT")
    BRIDGE_PORT = int(os.environ.get("LGSVL__AUTOPILOT_0_PORT", 9090))
    dummy_ego.connect_bridge(BRIDGE_HOST,BRIDGE_PORT)
    # windows log file path
    # final_data = deserialize_log_file("C:\\Users\\dheer\\AppData\\LocalLow\\LGElectronics\\SVLSimulator\\simulation_log.json.gz")
    # linux log file path
    final_data = deserialize_log_file("/home/local/ASUAD/vakula1/.config/unity3d/LGElectronics/SVLSimulator/simulation_log.json.gz")
    # print("Current ending time = ", sim.current_time)
    # print("Current ending frame = ", sim.current_frame)
    return final_data

if __name__ == "__main__":
    input("Press Enter to run the simulation")
    x = [i for i in range(0,100,1)]
    y = [i for i in range(0,100,1)]
    run_test(13.2160, 100.5596, 0,x,y,10, False)
    #run_test(2.755, 15.498, 0,x,y,10, False)
   
    

