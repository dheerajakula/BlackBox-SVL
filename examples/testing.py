import logging
import math

import numpy as np
from blackbox_svl.SVLSimulation import SVLSimulation
from numpy.typing import NDArray
from staliro.core.model import Failure
from staliro.core.model import Model, ModelData, Failure, StaticInput, Signals
from staliro.core.interval import Interval
from staliro.core.sample import Sample
from staliro.models import StaticInput, SignalTimes, SignalValues, ModelData, blackbox
from staliro.optimizers import DualAnnealing
from staliro.options import Options, SignalOptions
from staliro.specifications import RTAMTDense,TLTK
from staliro.staliro import staliro, simulate_model
from staliro.signals import Pchip, PiecewiseLinear

svlDataT = ModelData[NDArray[np.float_], None]

svl_sim = None

@blackbox()
def svl_model(static: StaticInput, times: SignalTimes, signals: SignalValues) -> svlDataT:
 
    ego_init_speed_m_s = static[0]
    ego_x_pos = 0
    print("ego_init_speed_m_s: " + str(ego_init_speed_m_s))
    print("ego_x_pos: " + str(ego_x_pos))
    step = 1 / 10

    forward = signals[0]
    #rightward = signals[1]
    #signal = [forward, rightward]
    signal = [forward]

    #result = svl_sim.run_test(ego_init_speed_m_s, ego_x_pos, pedestrian_speed=0, steptime=times, InpSignal=signal, sim_duration=15, for_matlab=False)
    points = np.array([[-200, 20], [5.4, 15], [5.6, 10]])
    points = np.array([[-206, 217], [5.4, 15], [5.6, 10]])
    result = svl_sim.run_test(ego_init_speed_m_s, ego_x_pos, pedestrian_speed=0, steptime=times, Points = points, InpSignal=None, sim_duration=15, for_matlab=False)

    #result = run_test(ego_init_speed_m_s, ego_x_pos, pedestrian_speed, steptime=times, Inpsignal_Forward = forward, Inpsignal_Rightward = rightward, sim_duration=10, for_matlab=False)
    #result = [[0.0, -4.998257, 3.815833, 0.9999994, 10.0147066, 43.00901, 155.4264, 0.323286742, 100.0001, 0.0, 0.0], [0.09999990463256836, -4.982434, 4.809959, 0.9999802, 9.868695, 39.3453026, 150.689972, 0.323287636, 1.70551479, 0.0, 0.0], [0.19999980926513672, -4.967915, 5.78444672, 0.9999616, 9.648887, 39.2508049, 150.567749, 0.323288351, 1.70337427, 0.0, 0.0], [0.2999997138977051, -4.95423365, 6.737262, 0.9999609, 9.43425, 39.1563225, 150.445541, 0.323288649, 1.70123374, 0.0, 0.0], [0.39999961853027344, -4.94004154, 7.669006, 0.9999631, 9.225157, 39.0619545, 150.3236, 0.32328862, 1.69788623, 0.0, 0.0], [0.4999995231628418, -4.92532, 8.579951, 0.9999624, 9.017039, 38.96768, 150.201843, 0.323288441, 0.1141549, 0.0, 0.0], [0.5999994277954102, -4.910553, 9.469978, 0.999960542, 8.806789, 38.8639259, 150.0677, 0.323288172, 1.69574571, 0.0, 0.0], [0.6999993324279785, -4.89621973, 10.3387918, 0.9999596, 8.593539, 38.7699928, 149.946259, 0.323287845, 1.69267154, 0.0, 0.0], [0.7999992370605469, -4.88225031, 11.1827526, 0.99995923, 8.319785, 38.6762161, 149.825043, 0.323287517, 1.69146466, 0.0, 0.0], [0.8999991416931152, -4.868645, 12.0019789, 0.999959, 8.09741, 38.58259, 149.703949, 0.323287219, 1.68811715, 0.0, 0.0]]
    result = np.array(result)

    # get the column for agent_x at index 5 and subtract ego_x at index 1
    Xdiff = result[:,5] - result[:,1]
    #print(Xdiff)
    # get the column for agent_y at index 6 and subtract ego_y at index 2
    Ydiff = result[:,6] - result[:,2]
    #print(Ydiff)
    # stack these two columns together
    traj = np.column_stack((Xdiff, Ydiff))
    
    trajectories: NDArray[np.float_] = traj
    timestamps: NDArray[np.float_] = result[:, 0]
    return ModelData(trajectories.T, timestamps)



# phi = "[](!(AGENT_Y - EGO_Y <= 1.5 /\ EGO_Y - AGENT_Y <= 1.5  /\ AGENT_X - EGO_X <= 8 /\ EGO_X - AGENT_X <= 0))"
#phi = "always[0,14.9] (not (Ydiff <= 1.5 and Ydiff >= -1.5 and Xdiff <= 8 and Xdiff >= 0))"
phi = "always[0,inf] (not (Ydiff <= 1.5 and Ydiff >= -1.5 and Xdiff <= 8 and Xdiff >= 0))"
# phi = "always (not (Ydiff <= 2 and Ydiff >= -2 and Xdiff <= 5 and Xdiff <= 0))"
# phi = "always[0,14.9] (not (Ydiff <= 2 and Ydiff >= -2 and Xdiff <= 0))"
# predicates = {
#     "EGO_X" : 0,
#     "EGO_Y" : 1,
#     "EGO_THETA": 2,3waaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
#     "EGO_V": 3,
#     "AGENT_X": 4,
#     "AGENT_Y": 5,
#     "AGENT_THETA": 6,
#     "AGENT_V": 7,
#     "PED_X": 8,
#     "PED_Y": 9
# }
predicates = {
    "Xdiff" : 0,
    "Ydiff" : 1,
}
#specification = RTAMTDense(phi, predicates)
specification = TLTK(phi, predicates)

optimizer = DualAnnealing()

initial_conditions = [
    np.array([0.0, 10.0]),  #ego_init_speed_m_s
    #np.array([0.0, 1.0]),  #ego_x_pos
    #np.array([2.0, 5.0]),  #pedestrian_speed
]

signals = [
    SignalOptions((0, 100), factory=PiecewiseLinear, control_points=3),
    #SignalOptions((0, 100), factory=PiecewiseLinear, control_points=3),
]

options = Options(runs=1, iterations=100, interval=(0, 15), static_parameters=initial_conditions, signals=signals)

if __name__ == "__main__":
    svl_sim = SVLSimulation("5d272540-f689-4355-83c7-03bf11b6865f")
    #logging.basicConfig(level=logging.DEBUG)
    result = staliro(svl_model, specification, optimizer, options)
    best_sample = result.best_run.best_eval.sample
    #best_sample = Sample(values=(0.26528216898441315, 27.32218211889267, 23.120462901890278, 84.91331632435322))
    print(best_sample)
    best_result = simulate_model(svl_model, options, best_sample)

    assert not isinstance(best_result, Failure)

# adding comments