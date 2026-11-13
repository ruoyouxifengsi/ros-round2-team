"""Keep simulated sensor age separate from bounded wall-clock delivery delays."""
import math

def navigation_data_fresh(sim,wall,stamp,received,simulated_time=False):
    return (all(math.isfinite(v) for v in (sim,wall,stamp,received)) and stamp>0 and
            0<=sim-stamp<=.5 and 0<=wall-received<=(5. if simulated_time else 2.))
