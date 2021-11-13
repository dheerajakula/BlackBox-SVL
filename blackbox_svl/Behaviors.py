import copy
import json
import gzip
import lgsvl
import math
from environs import Env
import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

class Behaviors:
    def __init__(self, ConfigFile, state, sim, steptime):
        self.ConfigFile = ConfigFile
        self.state = state
        self.sim = sim
        self.steptime = steptime

    def FollowInpSignal(self, InpSignal):
        if(len(InpSignal) == 1):
            self.FollowForward(InpSignal[0])
        else:
            self.FollowForwardRightward(InpSignal[0], InpSignal[1])

    def FollowForwardRightward(self, InpSignal_Forward, InpSignal_Rightward):
        npc_state = copy.deepcopy(self.state)
        npc = None
        waypoints = []
        speed = 1
        npc_type = None
        if InpSignal_Forward is None:
            InpSignal_Forward = np.zeros(len(self.steptime))
        if InpSignal_Rightward is None:
            InpSignal_Rightward = np.zeros(len(self.steptime))

        # Read json file to get the initial position of the NPC
        with open(self.ConfigFile) as json_data:
            d = json.load(json_data)
            for agent in d['agents']:
                if(agent['type'] == 2):
                    transform = agent['transform']
                    pos = lgsvl.Vector(transform['position']['x'], transform['position']['y'], transform['position']['z'])
                    angle = lgsvl.Vector(transform['rotation']['x'], transform['rotation']['y'], transform['rotation']['z'])
                    npc_state.transform.position = pos
                    npc_state.transform.rotation = angle
                    # npc = sim.add_agent(agent['variant'], lgsvl.AgentType.NPC, npc_state)
                    npc_type = agent['variant']
        
        npc_origin = npc_state.transform.position
        forward = lgsvl.utils.transform_to_forward(npc_state.transform)
        rightward = lgsvl.utils.transform_to_right(npc_state.transform)
        npc_start = npc_origin + forward * InpSignal_Forward[0] + rightward * InpSignal_Rightward[0]
        
        # Create waypoints based on the input signal
        length_of_signal = max(len(InpSignal_Forward), len(InpSignal_Rightward))

    
        prev_timestamp = 0
        angle_change = math.degrees(math.atan((InpSignal_Rightward[0]+1e-6)/(InpSignal_Forward[0]+1e-6)))

        npc_state.transform.position = npc_start
        initial_angle = npc_state.transform.rotation
        npc_state.transform.rotation = initial_angle + lgsvl.Vector(0, angle_change, 0)

        # add npc at the initial position
        npc = self.sim.add_agent(npc_type, lgsvl.AgentType.NPC, npc_state)
        print(length_of_signal)

        for i in range(1,length_of_signal):

            inp_forward = InpSignal_Forward[i]
            inp_rightward = InpSignal_Rightward[i]
            prev_inp_forward = InpSignal_Forward[i-1]
            prev_inp_rightward = InpSignal_Rightward[i-1]

            pos = npc_origin + forward * inp_forward + rightward * inp_rightward
            
            angle_change = math.degrees(math.atan((inp_rightward-prev_inp_rightward+1e-6)/(inp_forward-prev_inp_forward+1e-6)))

            if(inp_forward-prev_inp_forward < 0):
                angle_change += 180
            angle = initial_angle + lgsvl.Vector(0, angle_change, 0)

            distance = ((inp_forward - prev_inp_forward)**2 + (inp_rightward - prev_inp_rightward)**2)**0.5
            speed = distance / (self.steptime[i]- self.steptime[i-1])

            wp = lgsvl.DriveWaypoint(position = pos, angle = angle, speed=speed, timestamp=self.steptime[i])
            waypoints.append(wp)

        npc.follow(waypoints)

    def FollowForward(self, InpSignal):
        npc_state = copy.deepcopy(self.state)
        npc = None
        waypoints = []
        speed = 1
        # Read json file to get the initial position of the NPC
        with open(self.ConfigFile) as json_data:
            d = json.load(json_data)
            for agent in d['agents']:
                if(agent['type'] == 2):
                    transform = agent['transform']
                    pos = lgsvl.Vector(transform['position']['x'], transform['position']['y'], transform['position']['z'])
                    angle = lgsvl.Vector(transform['rotation']['x'], transform['rotation']['y'], transform['rotation']['z'])
                    npc_state.transform.position = pos
                    npc_state.transform.rotation = angle
                    npc = self.sim.add_agent(agent['variant'], lgsvl.AgentType.NPC, npc_state)
        
        
        forward = lgsvl.utils.transform_to_forward(npc_state.transform)
        for i,inp in enumerate(InpSignal):
            pos = npc.state.transform.position + forward * inp
            angle = npc.state.transform.rotation
            wp = lgsvl.DriveWaypoint(position= pos, angle = angle,speed=100, timestamp=self.steptime[i])
            waypoints.append(wp)
        npc.follow(waypoints)
    
    def FollowPoints(self, points):
        npc_state = copy.deepcopy(self.state)
        npc = None
        waypoints = []
        speed = 1
        # Read json file to get the initial position of the NPC
        with open(self.ConfigFile) as json_data:
            d = json.load(json_data)
            for agent in d['agents']:
                if(agent['type'] == 2):
                    transform = agent['transform']
                    pos = lgsvl.Vector(transform['position']['x'], transform['position']['y'], transform['position']['z'])
                    angle = lgsvl.Vector(transform['rotation']['x'], transform['rotation']['y'], transform['rotation']['z'])
                    npc_state.transform.position = pos
                    npc_state.transform.rotation = angle
                    npc = self.sim.add_agent(agent['variant'], lgsvl.AgentType.NPC, npc_state)


        

        # get the initial position of the NPC and add it to points
        pos_x = npc.state.transform.position.x
        pos_y = npc.state.transform.position.y
        pos_z =  npc.state.transform.position.z
        print(points)
        points = np.append(points, [[pos_x, pos_z]], axis = 0)
        print(points)
        points = points[points[:, 0].argsort()]
        print(points)

        points_x = points[:,0]
        points_z = points[:,1]

       
        # create waypoints based on points by smooth spline interpolation
        X_Y_Spline = make_interp_spline(points_x, points_z)
        spline_x = np.linspace(points_x.min(), points_x.max(), num=100)
        spline_z = X_Y_Spline(spline_x)
        print(spline_z)

        print(pos_x, pos_y, pos_z)
        # Define interpolators.
        f_linear = interp1d(points_x, points_z)
        f_cubic = interp1d(points_x, points_z, kind='cubic')

        # Plot.
        plt.plot(points_x, points_z, 'o', label='data')
        plt.plot(spline_x, f_linear(spline_x), '-', label='linear')
        plt.plot(spline_x, f_cubic(spline_x), '--', label='cubic')
        plt.legend(loc='best')
        plt.savefig('test.png')

        # let the npc follow the spline
        for i in range(len(spline_x)):
            pos = lgsvl.Vector(spline_x[i], pos_y, spline_z[i])
            wp = lgsvl.DriveWaypoint(position= pos, speed = 10)
            waypoints.append(wp)
            
        npc.follow(waypoints)