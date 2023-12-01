# Autopilot Tuning Stuff File
# Everyone
# lets tune this boy

# import sys
# import os
# cwd = os.getcwd()
# sys.path.append(cwd)
# import numpy as np
# import keyboard
# import lib.ACanim as anim
# import lib.ACdynamics as acd
# import lib.ACaero as aca
# import lib.carrier_dynamics as car
# import lib.wind as wind
# from lib.autopilot import autopilot
# from lib.calcWreq import calcWreq
# from obj.hangar import *
# from lib.simparams import *
# import lib.nav as nav
# plt.ion()


# ###### Initialize Misc Classes ######
# anim = anim.animation(50, 0.6)
# ac_dyn = acd.ACdynamics()
# ac_aero = aca.Aero()
# car_dyn = car.carrier_dynamics(0.)
# Vsteady = np.array([[0.], [0.], [0.]])
# wind = wind.wind(Vsteady)
# autop = autopilot(ts_simulation, 2.)
# nav = nav.nav(car_dyn.chi, 1500.)

# ## Initalize Sim Time ##
# t = start_time

# # init state
# states0 = np.array([[-1500.], #pn
#                   [1000.], #pe
#                   [-250.], # pd
#                   [50.], # u
#                   [0.], # v
#                   [0.], # w
#                   [0.], # phi
#                   [0.], # theta
#                   [-np.pi/2], # psi
#                   [0.], # p
#                   [0.], # q
#                   [0.]]) # r
# ac_dyn.state = states0
# pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = ac_dyn.state.flatten()

# # commanded vals
# Va = 50.
# Va_c = 35.
# w_c = 0.
# theta_c = np.deg2rad(3.)
# chi_c = np.deg2rad(0.)
# h_c = 100.
# ws = []
# ts = []
# appFlag = False
# doApp = False
# landFlag = False
# goround = False
# appTol = 435.
# point = 0
# hland = 0.
# chi_a = 0.

# ## Main Sim Loop ##
# while t < end_time:
#     t_next_plot = t + ts_plotting
#     while t < t_next_plot:
#         # update carrier dynamics
#         car_dyn.update(t, False)
        
#         # check landing status
#         if landFlag == False:
#             # determine approach position and commanded chi
#             an, ae = nav.approachLoc(car_dyn.state)
#             if np.abs(pn) <= np.abs(an) + appTol and np.abs(pe) <= np.abs(ae) + appTol and \
#                 np.abs(pn) >= np.abs(an) - appTol and np.abs(pe) >= np.abs(ae) - appTol and goround != True:
#                 appFlag = True
#                 goaround = False
#                 chi_c = nav.courseToCar(ac_dyn.state, car_dyn.state)
#                 landFlag = nav.checkSuccess(car_dyn.state, pn, pe)
#                 cn, ce, ch = nav.landLoc(car_dyn.state)
#                 hland = -h_c + 0.75
#                 w_c = calcWreq(car_dyn.state, ac_dyn.state, cn, ce, ch)
#             elif appFlag == False:
#                 chi_c = nav.courseToApproach(ac_dyn.state, car_dyn.state)
#                 appFlag = False
        
#         # do wind
#         Va, alpha, beta = wind.wind_char(ac_dyn.state, Va, ts_simulation)
        
#         # autopilot
#         pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = ac_dyn.state.flatten()
#         u_a = np.array([t, w, phi, theta, psi, p, q, r, Va, -pd, Va_c, h_c, chi_c, theta_c, theta, w_c])
#         delta_e, delta_a, delta_r, delta_t, goround = autop.update(u_a, hland, appFlag, landFlag)
        
#         # do goround if necessary
#         if goround == True:
#             h_c = 250.
#             chi_c = car_dyn.chi - np.deg2rad(30.)
#             landFlag = False
#             appFlag = False
        
#         # aero
#         fx, fy, fz = ac_aero.forces(ac_dyn.state, delta_e, delta_a, delta_r, delta_t, alpha, beta, Va)
#         l, m, n = ac_aero.moments(ac_dyn.state, delta_e, delta_a, delta_r, delta_t, alpha, beta, Va)
        
#         # dynamics
#         ac_dyn.update(fx, fy, fz, l, m, n)
#         pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = ac_dyn.state.flatten()
#         chi_a = psi

#         # anim update
#         anim.update(f4_verts, carrier_verts, ac_dyn.state, car_dyn.state, ["b"], ["g"])
        
#         # end at 0 height
#         _, _, ch = nav.landLoc(car_dyn.state)
#         if pd >= ch:
#             print(f"AC Height: {-pd:.2f} m, Car Height: {-ch:.2f} m")
#             t = end_time
        
#         # iterate time
#         t += ts_simulation

#     # check for keybaord press
#     plt.pause(0.01)
#     if keyboard.is_pressed('q'): break

# print(f"AC Position: {pn:.2f} m N, {pe:.2f} m E")
# print(f"Car Position: {nav.landLoc(car_dyn.state)[0]:.2f} m N, {nav.landLoc(car_dyn.state)[1]:.2f} m E")
# print(f"Diff: {(pn-nav.landLoc(car_dyn.state)[0]):.2f} m N, {(pe-nav.landLoc(car_dyn.state)[1]):.2f} m E")
# plt.waitforbuttonpress()

# Autopilot Tuning Stuff File
# Everyone
# lets tune this boy
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
import matplotlib.pyplot as plt
plots = plt.figure("Plots")
plots.add_subplot
from lib.autopilot import autopilot
from lib.calcWreq import calcWreq
from obj.hangar import *
from lib.simparams import *
import lib.nav as nav
plt.ion()


###### Initialize Misc Classes ######
anim = anim.animation(10, 0.6)
ac_dyn = acd.ACdynamics()
ac_aero = aca.Aero()
car_dyn = car.carrier_dynamics(0.)
Vsteady = np.array([[0.], [0.], [0.]]) # for wind [-10,-7.5,0]
wind = wind.wind(Vsteady)
autop = autopilot(ts_simulation, 2.)
nav = nav.nav(car_dyn.chi, 1500.)

## Initalize Sim Time ##
t = start_time
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

# init state
states0 = np.array([[-3500.], #pn # -2000 for wind, -3500 for no wind, -2000 for failed approach
                  [-1000.], #pe # 1200 for wind, -1000 for no wind, 0 for failed approach
                  [-330.], # pd # -150 for wind, -330 for no wind, -100 for failed approach
                  [50.], # u # 35 for wind, 50 for no wind
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
Va = 50. # 35 for wind, 50 for no wind
Va_c = 35.
theta_c = np.deg2rad(3.)
chi_c = np.deg2rad(0.)
h_c = 100.
ws = []
ts = []
appFlag = False
landFlag = False
goround = False
appTol = 225.

## Main Sim Loop ##
while t < end_time:
    t_next_plot = t + ts_plotting
    while t < t_next_plot:
        # update carrier dynamics
        car_dyn.update(t, False)
        
        # check landing status
        if landFlag == False:
            # determine approach position and commanded chi
            an, ae = nav.approachLoc(car_dyn.state)
            if goround == True:
                h_c = 100.
                chi_c = car_dyn.chi - np.deg2rad(30.)
                appFlag = False
                landFlag = False
                goround = True
            elif np.abs(pn) <= np.abs(an) + appTol and np.abs(pe) <= np.abs(ae) + appTol and goround != True:
                appFlag = True
                goaround = False
                chi_c = nav.courseToCar(ac_dyn.state, car_dyn.state)
                landFlag = nav.checkSuccess(car_dyn.state, pn, pe)
            elif appFlag == False:
                chi_c = nav.courseToApproach(ac_dyn.state, car_dyn.state)
                appFlag = False
        
        # do wind
        Va, alpha, beta = wind.wind_char(ac_dyn.state, Va, ts_simulation)
        
        # autopilot
        pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = ac_dyn.state.flatten()
        cn, ce, ch = nav.landLoc(car_dyn.state)
        hland = -ch + 0.75 # for head/X-wind + 0.75
        w_c = calcWreq(car_dyn.state, ac_dyn.state, cn, ce, ch)
        u = np.array([t, w, phi, theta, psi, p, q, r, Va, -pd, Va_c, h_c, chi_c, theta_c, theta, w_c])
        delta_e, delta_a, delta_r, delta_t, goround = autop.update(u, hland, appFlag, landFlag)
        
        # do goround if necessary
        if goround == True:
            h_c = 100.
            chi_c = car_dyn.chi - np.deg2rad(30.)
            landFlag = False
            appFlag = False
        
        # aero
        fx, fy, fz = ac_aero.forces(ac_dyn.state, delta_e, delta_a, delta_r, delta_t, alpha, beta, Va)
        l, m, n = ac_aero.moments(ac_dyn.state, delta_e, delta_a, delta_r, delta_t, alpha, beta, Va)
        
        # dynamics
        ac_dyn.update(fx, fy, fz, l, m, n)
        pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = ac_dyn.state.flatten()

        # anim update
        anim.update(f4_verts, carrier_verts, ac_dyn.state, car_dyn.state, ["b"], ["g"])
        
        # end at 0 height
        if pd >= ch:
            print(f"AC Height: {-pd:.2f} m, Car Height: {-ch:.2f} m")
            t = end_time
        # do plotting for aircraft
        simtime.append(t)
        ACfxs.append(fx)
        ACfys.append(fy)
        ACfzs.append(fz)
        ACls.append(l)
        ACms.append(m)
        ACns.append(n)
        ACtht.append(delta_t)
        ACpns.append(pn)
        ACpes.append(pe)
        ACpds.append(pd)
        ACus.append(u)
        ACvs.append(v)
        ACws.append(w)
        ACphis.append(phi)
        ACthetas.append(theta)
        ACpsis.append(psi)
        ACps.append(p)
        ACqs.append(q)
        ACrs.append(r)
        ACdas.append(delta_a)
        ACdes.append(delta_e)
        ACdrs.append(delta_r)

        # do plotting for carrier
        carpns.append(cn)
        carpes.append(ce)
        carpds.append(ch)
        carphis.append(car_dyn.state[6][0])
        carthetas.append(car_dyn.state[7][0])
        carpsis.append(car_dyn.state[8][0])
        carus.append(car_dyn.state[3][0])
        carvs.append(car_dyn.state[4][0])
        carws.append(car_dyn.state[5][0])

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
        ACthrottle.bar(0, delta_t)
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

        # iterate time
        t += ts_simulation

    # check for keybaord press
    plt.pause(0.01)
    if keyboard.is_pressed('q'): break

print(f"AC Position: {pn:.2f} m N, {pe:.2f} m E")
print(f"Car Position: {nav.landLoc(car_dyn.state)[0]:.2f} m N, {nav.landLoc(car_dyn.state)[1]:.2f} m E")
print(f"Diff: {(pn-nav.landLoc(car_dyn.state)[0]):.2f} m N, {(pe-nav.landLoc(car_dyn.state)[1]):.2f} m E")
plt.waitforbuttonpress()