# Autopilot Tuning Stuff File
# Everyone
# lets tune this boy
import matplotlib.pyplot as plt
plots = plt.figure("Plots")
plots.add_subplot
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
plt.ion()
import lib.simparams as SIM
import lib.ACtrim as trim

###### Initialize Misc Classes ######
anim = anim.animation(10, 0.5)
ac_dyn = acd.ACdynamics()
ac_aero = aca.Aero()
car_dyn = car.carrier_dynamics(0.)
Vsteady = np.array([[5.], [10.], [0.]])
wind = wind.wind(Vsteady)
autop = autopilot(SIM.ts_simulation, 2.)

ACthrottle = plt.figure(1).add_subplot(1, 20, 1); throttlep = ACthrottle.get_position(); throttlep.x0-=0.075; throttlep.x1-=0.075
ACthrottle.set_position(throttlep)
ACposition = plt.figure(1).add_subplot(4,3,1)
ACforces = plt.figure(1).add_subplot(4,3,2)
carposition = plt.figure(1).add_subplot(4,3,3)
ACangles = plt.figure(1).add_subplot(4,3,4)
ACdefl = plt.figure(1).add_subplot(4,3,5)
carangles = plt.figure(1).add_subplot(4,3,6)
ACvelocity = plt.figure(1).add_subplot(4,3,7)
ACmoments = plt.figure(1).add_subplot(4,3,8)
carvelocity = plt.figure(1).add_subplot(4,3,9)
ACrates = plt.figure(1).add_subplot(4,3,10)

# initialize variables for append
simtime = []
ACnors = []
ACeass = []
ACdows = []
ACfxs = []
ACfys = []
ACfzs = []
ACls = []
ACms = []
ACns = []
ACpns = []
ACpes = []
ACpds = []
ACus = []
ACvs = []
ACws = []
ACphis = []
ACthetas = []
ACpsis = []
ACps = []
ACqs = []
ACrs = []
ACtht = []
ACdas = []
ACdes = []
ACdrs = []
carpns = []
carpes = []
carpds = []
carphis = []
carthetas = []
carpsis = []
carus = []
carvs = []
carws = []
# fig2 = plt.figure("Plots")
# ax2 = fig2.add_subplot(111)

## Initalize Sim Time ##
t = SIM.start_time
end_time = SIM.end_time
ts_plotting = SIM.ts_plotting
ts_simulation = SIM.ts_simulation


# init state
states0 = np.array([[0.], #pn
                  [0.], #pe
                  [-25.], # pd
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
theta_c = np.deg2rad(3.)
chi_c = np.deg2rad(10.)
h_c = 50.
ws = []
ts = []## Compute trim conditions ##
# xtrim, utrim = trim.compute_trim(35, 0, np.inf)
# print(xtrim)
# print('~~~~~~~~~~~~~~~~')
# print(utrim)

## Main Sim Loop ##
while t < end_time:
    t_next_plot = t + ts_plotting
    while t < t_next_plot:
        # update carrier dynamics
        car_dyn.update(t, False)
        carpe, carpd, carpn, caru, carv, carw, carphi, cartheta, carpsi, carp, carq, carr= car_dyn.state.flatten()
        
        # do wind
        Va, alpha, beta = wind.wind_char(ac_dyn.state, Va, ts_simulation)
        
        # AC autopilot
        ACpn, ACpe, ACpd, ACu, ACv, ACw, ACphi, ACtheta, ACpsi, ACp, ACq, ACr = ac_dyn.state.flatten()
        # w_c = calcWreq(car_dyn.state, ac_dyn.state)
        w_c = 0.
        u = np.array([t, ACw, ACphi, ACtheta, ACpsi, ACp, ACq, ACr, Va, -ACpd, Va_c, h_c, chi_c, theta_c, ACtheta, w_c])
        ACdelta_e, ACdelta_a, ACdelta_r, ACdelta_t = autop.update(u, False)
        # print(np.rad2deg(np.arctan(ACpe/ACpn)))
        # ws.append(np.rad2deg(np.arctan(pe/pn)))
        # ts.append(t)
        
        # aero
        ACfx, ACfy, ACfz = ac_aero.forces(ac_dyn.state, ACdelta_e, ACdelta_a, ACdelta_r, ACdelta_t, alpha, beta, Va)
        ACl, ACm, ACn = ac_aero.moments(ac_dyn.state, ACdelta_e, ACdelta_a, ACdelta_r, ACdelta_t, alpha, beta, Va)
        
        # AC dynamics
        ac_dyn.update(ACfx, ACfy, ACfz, ACl, ACm, ACn)
        # pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = ac_dyn.state.flatten()

        # anim update
        anim.update(f4_verts, carrier_verts, ac_dyn.state, car_dyn.state, ["b"], ["g"])
        
        # iterate time
        t += ts_simulation
        
    # do plotting
    # ax2.clear()
    # ax2.plot(ts, ws)
    # ax2.hlines(np.rad2deg(chi_c), 0, ts[-1], "r", linestyle="--")
    
    # check for keybaord press
    
    # do plotting
    # do plotting for aircraft
    simtime.append(t)
    ACfxs.append(ACfx)
    ACfys.append(ACfy)
    ACfzs.append(ACfz)
    ACls.append(ACl)
    ACms.append(ACm)
    ACns.append(ACn)
    ACtht.append(ACdelta_t)
    ACpns.append(ACpn)
    ACpes.append(ACpe)
    ACpds.append(ACpd)
    ACus.append(ACu)
    ACvs.append(ACv)
    ACws.append(ACw)
    ACphis.append(ACphi)
    ACthetas.append(ACtheta)
    ACpsis.append(ACpsi)
    ACps.append(ACp)
    ACqs.append(ACq)
    ACrs.append(ACr)
    ACdas.append(ACdelta_a)
    ACdes.append(ACdelta_e)
    ACdrs.append(ACdelta_r)

    # do plotting for carrier
    carpns.append(carpn)
    carpes.append(carpe)
    carpds.append(carpd)
    carphis.append(carphi)
    carthetas.append(cartheta)
    carpsis.append(carpsi)
    carus.append(caru)
    carvs.append(carv)
    carws.append(carw)

        # plot data on subplots
    ACangles.clear()
    ACforces.clear()
    ACmoments.clear()
    ACrates.clear()
    ACdefl.clear()
    ACposition.clear() 
    ACvelocity.clear() 
    ACthrottle.clear()
    carposition.clear()
    carangles.clear()
    carvelocity.clear()
    ACthrottle.bar(0, ACdelta_t)
    ACangles.plot(simtime, np.rad2deg(ACphis), "g-", label="$\phi$")
    ACangles.plot(simtime, np.rad2deg(ACthetas), "r-", label="$\ttheta$")
    ACangles.plot(simtime, np.rad2deg(ACpsis), "b-", label="$\psi$")
    carangles.plot(simtime, np.rad2deg(carphis), "g-", label="$\phi$")
    carangles.plot(simtime, np.rad2deg(carthetas), "r-", label="$\psi$")
    carangles.plot(simtime, np.rad2deg(carpsis), "b-", label="$\psi$")
    ACforces.plot(simtime, ACfxs, "g-", label="Fx")
    ACforces.plot(simtime, ACfys, "r-", label="Fy")
    ACforces.plot(simtime, ACfzs, "b-", label="Fz")
    ACmoments.plot(simtime, ACls, "g-", label="l")
    ACmoments.plot(simtime, ACms, "r-", label="m")
    ACmoments.plot(simtime, ACns, "b-", label="n")
    ACposition.plot(simtime, ACpns, "m-", label="North")
    ACposition.plot(simtime, ACpes, "y-", label="East")
    ACposition.plot(simtime, ACpds, "k-", label="Down")
    carposition.plot(simtime, carpns, "m-", label="North")
    carposition.plot(simtime, carpes, "y-", label="East")
    carposition.plot(simtime, carpds, "k-", label="Down")
    ACrates.plot(simtime, np.rad2deg(ACps), "g-", label="p")
    ACrates.plot(simtime, np.rad2deg(ACqs), "r-", label="q")
    ACrates.plot(simtime, np.rad2deg(ACrs), "b-", label="r")
    ACdefl.plot(simtime, ACdes, "g-", label="e")
    ACdefl.plot(simtime, ACdas, "r-", label="a")
    ACdefl.plot(simtime, ACdrs, "b-", label="r")
    ACvelocity.plot(simtime, ACus, "g-", label="u")
    ACvelocity.plot(simtime, ACvs, "r-", label="v")
    ACvelocity.plot(simtime, ACws, "b-", label="w")
    carvelocity.plot(simtime, carus, "g-", label="u")
    carvelocity.plot(simtime, carvs, "r-", label="v")
    carvelocity.plot(simtime, carws, "b-", label="w")
    ACthrottle.set_ylabel('Throttle')
    ACthrottle.set_ylim(0, 1)
    ACthrottle.tick_params(axis='y', which='both', left=False, right=False)
    ACangles.grid('major')
    ACangles.set_ylabel('Angle (deg)')
    ACangles.legend(loc='upper right')
    carangles.grid('major')
    carangles.set_ylabel('Angle (deg)')
    carangles.legend(loc='upper right')
    ACforces.grid('major')
    ACforces.set_ylabel('Force (N)')
    ACforces.legend(loc='upper right')
    ACmoments.grid('major')
    ACmoments.set_ylabel('Moment')
    ACmoments.legend(loc='upper right')
    ACmoments.set_xlabel('Time (s)')
    ACposition.grid('major')
    ACposition.set_ylabel('Position (m)')
    ACposition.legend(loc='upper right')
    ACposition.set_title('Aircraft Dynamics')
    carposition.grid('major')
    carposition.set_ylabel('Position (m)')
    carposition.set_title('Carrier Dynamics')
    carposition.legend(loc='upper right')
    ACrates.grid('major')
    ACrates.set_ylabel('Ang Velocity (deg/s)')
    ACrates.legend(loc='upper right')
    ACrates.set_xlabel('Time (s)')
    ACdefl.grid('major')
    ACdefl.set_xlabel('Time (s)')
    ACdefl.set_ylabel('Deflection (deg)')
    ACdefl.legend(loc='upper right')
    ACvelocity.grid('major')
    ACvelocity.set_ylabel('Velocity (m/s)')
    ACvelocity.legend(loc='upper right')
    carvelocity.grid('major')
    carvelocity.set_ylabel('Velocity (m/s)')
    carvelocity.legend(loc='upper right')
    carvelocity.set_xlabel('Time (s)')
    



    # check for keybaord press
    plt.pause(0.01)
    if keyboard.is_pressed('q'): break
