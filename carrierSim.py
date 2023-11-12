import sys
import os
cwd = os.getcwd()
sys.path.append(cwd)
import numpy as np
import keyboard
import lib.ACanim as anim
import lib.ACdynamics as acd
import lib.ACaero as aca
import lib.waves as wave
from obj.hangar import *
from lib.simparams import *

###### Initialize Misc Classes ######
anim = anim.animation()
ac_dyn = acd.ACdynamics()
ac_aero = aca.Aero()



## Initalize Sim Time ##
Ts = start_time


## Verts for AC and Carrier ##




## Main Sim Loop ##
while Ts < end_time:
    ## Carrier stuff ##
    

    ## Wind stuff to get alpha, beta, Va ##
    

    ## Autopilot to get deflection angles ##


    ## Aero stuff to get forces/moments ##


    ## Dynamics stuff to get state ##


    ## Animate the state ##
    anim.update()

    if keyboard.is_pressed('q'): break
    plt.pause(0.1)
    Ts += ts_simulation