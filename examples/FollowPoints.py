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


    #result = svl_sim.run_test(ego_init_speed_m_s, ego_x_pos, pedestrian_speed=0, steptime=times, InpSignal=signal, sim_duration=15, for_matlab=False)
    points = np.array([[-200, 20], [5.4, 15], [5.6, 10]])
    points = np.array([[-206, 217], [5.4, 15], [5.6, 10]])
    result = svl_sim.run_test(ego_init_speed_m_s, ego_x_pos, steptime=times, Points = points, sim_duration=15, for_matlab=False)

    result = np.array(result)

    # get the column for agent_x at index 5 and subtract ego_x at index 1
    Xdiff = result[:,5] - result[:,1]
    
    # get the column for agent_y at index 6 and subtract ego_y at index 2
    Ydiff = result[:,6] - result[:,2]

    # stack these two columns together
    traj = np.column_stack((Xdiff, Ydiff))
    
    trajectories: NDArray[np.float_] = traj
    timestamps: NDArray[np.float_] = result[:, 0]
    return ModelData(trajectories.T, timestamps)

phi = "always[0,inf] (not (Ydiff <= 1.5 and Ydiff >= -1.5 and Xdiff <= 8 and Xdiff >= 0))"

predicates = {
    "Xdiff" : 0,
    "Ydiff" : 1,
}

specification = TLTK(phi, predicates)

optimizer = DualAnnealing()

initial_conditions = [
    np.array([0.0, 10.0]),  #ego_init_speed_m_s
    np.array([0.0, 1.0]),  #ego_x_pos
]



options = Options(runs=1, iterations=100, interval=(0, 15), static_parameters=initial_conditions)

if __name__ == "__main__":
    svl_sim = SVLSimulation("5d272540-f689-4355-83c7-03bf11b6865f")
    result = staliro(svl_model, specification, optimizer, options)
    best_sample = result.best_run.best_eval.sample
    print(best_sample)
    best_result = simulate_model(svl_model, options, best_sample)
    assert not isinstance(best_result, Failure)
