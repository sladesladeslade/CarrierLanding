# Autopilot Tuning Stuff File
# Everyone
# lets tune this boy

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
import lib.nav as nav
plt.ion()


###### Initialize Misc Classes ######
anim = anim.animation(10, 0.5)
ac_dyn = acd.ACdynamics()
ac_aero = aca.Aero()
car_dyn = car.carrier_dynamics(0.)
Vsteady = np.array([[0.], [0.], [0.]])
wind = wind.wind(Vsteady)
autop = autopilot(ts_simulation, 2., 5.)
nav = nav.nav(car_dyn.chi, 1500.)

## Initalize Sim Time ##
t = start_time

# init state
states0 = np.array([[-2000.], #pn
                  [2000.], #pe
                  [-100.], # pd
                  [50.], # u
                  [0.], # v
                  [0.], # w
                  [0.], # phi
                  [0.], # theta
                  [0.], # psi
                  [0.], # p
                  [0.], # q
                  [0.]]) # r
ac_dyn.state = states0
pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = ac_dyn.state.flatten()

# commanded vals
Va = 50.
Va_c = 50.
theta_c = np.deg2rad(3.)
chi_c = np.deg2rad(0.)
h_c = 100.
ws = []
ts = []
appFlag = False
appTol = 350.

## Main Sim Loop ##
while t < end_time:
    t_next_plot = t + ts_plotting
    while t < t_next_plot:
        # update carrier dynamics
        car_dyn.update(t, False)
        
        # determine approach position and commanded chi
        an, ae = nav.approachLoc(car_dyn.state)
        if np.abs(pn) <= np.abs(an) + appTol and np.abs(pe) <= np.abs(ae) + appTol:
            appFlag = True
            chi_c = nav.courseToCar(ac_dyn.state, car_dyn.state)
        elif appFlag == False:
            chi_c = nav.courseToApproach(ac_dyn.state, car_dyn.state)
        
        # do wind
        Va, alpha, beta = wind.wind_char(ac_dyn.state, Va, ts_simulation)
        
        # autopilot
        pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = ac_dyn.state.flatten()
        w_c = calcWreq(car_dyn.state, ac_dyn.state)
        u = np.array([t, w, phi, theta, psi, p, q, r, Va, -pd, Va_c, h_c, chi_c, theta_c, theta, w_c])
        delta_e, delta_a, delta_r, delta_t = autop.update(u, appFlag)
        
        # aero
        fx, fy, fz = ac_aero.forces(ac_dyn.state, delta_e, delta_a, delta_r, delta_t, alpha, beta, Va)
        l, m, n = ac_aero.moments(ac_dyn.state, delta_e, delta_a, delta_r, delta_t, alpha, beta, Va)
        
        # dynamics
        ac_dyn.update(fx, fy, fz, l, m, n)
        pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = ac_dyn.state.flatten()

        # anim update
        anim.update(f4_verts, carrier_verts, ac_dyn.state, car_dyn.state, ["b"], ["g"])
        
        # end at 0 height
        if pd >= 0.:
            print(pd)
            t = end_time
        
        # iterate time
        t += ts_simulation
    
    # check for keybaord press
    plt.pause(0.01)
    if keyboard.is_pressed('q'): break

print(pn, pe)
print(car_dyn.state[0][0], car_dyn.state[1][0])
plt.waitforbuttonpress()