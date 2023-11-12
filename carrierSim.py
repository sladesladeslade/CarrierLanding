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



## Main Sim Loop ##
while Ts < end_time:
    """"""

    if keyboard.is_pressed('q'): break
plt.pause(0.1)
Ts += ts_simulation