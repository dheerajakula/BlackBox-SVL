import lgsvl
def set_lights_green(sim):
    controllables = sim.get_controllables("signal")
    for signal in controllables:
        # Create a new control policy
        control_policy = "trigger=500;green=300;yellow=0;red=0;loop"
        # Control this traffic light with a new control policy
        signal.control(control_policy)
