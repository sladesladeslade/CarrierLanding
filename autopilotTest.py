# Autopilot Testing File
# Everyone
# lets test this boy

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
import lib.wind as wind
from lib.autopilot import autopilot
from lib.calcWreq import calcWreq
from obj.hangar import *
from lib.simparams import *
plt.ion()


###### Initialize Misc Classes ######
anim = anim.animation(25, 0.4)
ac_dyn = acd.ACdynamics()
ac_aero = aca.Aero()
car_dyn = car.carrier_dynamics(0.)
Vsteady = np.array([[0.], [0.], [0.]])
wind = wind.wind(Vsteady)
autop = autopilot(ts_simulation, 2.)

## Initalize Sim Time ##
t = start_time

# init state
states0 = np.array([[-5500.], #pn
                  [0.], #pe
                  [-336.4], # pd
                  [35.], # u
                  [0.], # v
                  [0.], # w
                  [0.], # phi
                  [0.], # theta
                  [0.], # psi
                  [0.], # p
                  [0.], # q
                  [0.]]) # r
ac_dyn.state = states0

# commanded vals
Va = 35.
Va_c = 35.
alpha_c = np.deg2rad(8.)
chi_c = 0.
h_c = 0.

## Main Sim Loop ##
while t < end_time:
    t_next_plot = t + ts_plotting
    while t < t_next_plot:
        # update carrier dynamics
        car_dyn.update(t, False)
        
        # do wind
        Va, alpha, beta = wind.wind_char(ac_dyn.state, Va, ts_simulation)
        
        # autopilot
        pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = ac_dyn.state.flatten()
        w_c = calcWreq(car_dyn.state, ac_dyn.state)
        u = np.array([t, w, phi, theta, psi, p, q, r, Va, -pd, Va_c, h_c, chi_c, alpha_c, alpha, w_c])
        delta_e, delta_a, delta_r, delta_t = autop.update(u, True)
        
        # aero
        fx, fy, fz = ac_aero.forces(ac_dyn.state, delta_e, delta_a, delta_r, delta_t, alpha, beta, Va)
        l, m, n = ac_aero.moments(ac_dyn.state, delta_e, delta_a, delta_r, delta_t, alpha, beta, Va)
        
        # dynamics
        ac_dyn.update(fx, fy, fz, l, m, n)

        # anim update
        anim.update(f18_verts, carrier_verts, ac_dyn.state, car_dyn.state, ["b"], ["g"])
        
        # iterate time
        t += ts_simulation
        
    # do plotting
    
    # check for keybaord press
    plt.pause(0.01)
    if keyboard.is_pressed('q'): break