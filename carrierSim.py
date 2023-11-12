import sys
import os
cwd = os.getcwd()
sys.path.append(cwd)
import numpy as np
import keyboard
import lib.ACanim as anim
import lib.ACdynamics as acd
import lib.ACaero as aca
import lib.carrier_dynamics as car
from obj.hangar import *
from lib.simparams import *

###### Initialize Misc Classes ######
anim = anim.animation(25, 0.4)
ac_dyn = acd.ACdynamics()
ac_aero = aca.Aero()
car_dyn = car.carrier_dynamics(0)



## Initalize Sim Time ##
t = start_time



## Main Sim Loop ##
while t < end_time:
    t_next_plot = t + ts_plotting
    while t < t_next_plot:
        # update carrier dynamics
        car_dyn.update(t)
        
        # do wind
        
        # autopilot
        
        # aero
        
        # dynamics

        # anim update
        anim.update(f18_verts, carrier_verts, ac_dyn.state, car_dyn.state, ["b"], ["g"])
        
        # iterate time
        t += ts_simulation
        
    # do plotting
    
    # check for keybaord press
    plt.pause(0.01)
    if keyboard.is_pressed('q'): break