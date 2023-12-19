# Plots

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
from obj.hangar import *
import lib.simparams as SIM


# ###### Initialize Misc Classes ######
anim = anim.animation(100, 0.4)
ac_dyn = acd.ACdynamics()
ac_aero = aca.Aero()
car_dyn = car.carrier_dynamics(0)

ACthrottle = plt.figure(1).add_subplot(1, 20, 1); throttlep = ACthrottle.get_position(); throttlep.x0-=0.075; throttlep.x1-=0.075
ACthrottle.set_position(throttlep)
ACposition = plt.figure(1).add_subplot(4,3,1)
# positionp = position.get_position(); positionp.x0 += 0.1; positionp.x1 += 0.075; position.set_position(positionp)
ACforces = plt.figure(1).add_subplot(4,3,2)
# forcep = forces.get_position(); forcep.x0 += 0.1; forcep.x1 += 0.075; forces.set_position(forcep)
carposition = plt.figure(1).add_subplot(4,3,3)
# positionp = position.get_position(); positionp.x0 += 0.1; positionp.x1 += 0.075; position.set_position(positionp)
ACangles = plt.figure(1).add_subplot(4,3,4)
# anglep = angles.get_position(); anglep.x0 += 0.1; anglep.x1 += 0.075; angles.set_position(anglep)
ACdefl = plt.figure(1).add_subplot(4,3,5)
# deflp = defl.get_position(); deflp.x0 += 0.1; deflp.x1 += 0.075; defl.set_position(deflp)
carangles = plt.figure(1).add_subplot(4,3,6)
# anglep = angles.get_position(); anglep.x0 += 0.1; anglep.x1 += 0.075; angles.set_position(anglep)
ACvelocity = plt.figure(1).add_subplot(4,3,7)
# velocityp = velocity.get_position(); velocityp.x0 += 0.1; velocityp.x1 += 0.075; velocity.set_position(velocityp)
ACmoments = plt.figure(1).add_subplot(4,3,8)
# momentp = moments.get_position(); momentp.x0 += 0.1; momentp.x1 += 0.075; moments.set_position(momentp)
carvelocity = plt.figure(1).add_subplot(4,3,9)
# velocityp = velocity.get_position(); velocityp.x0 += 0.1; velocityp.x1 += 0.075; velocity.set_position(velocityp)
ACrates = plt.figure(1).add_subplot(4,3,10)
# ratep = rates.get_position(); ratep.x0 += 0.1; ratep.x1 += 0.075; rates.set_position(ratep)



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

## Initalize Sim Time ##
t = SIM.start_time
end_time = SIM.end_time
ts_plotting = SIM.ts_plotting
ts_simulation = SIM.ts_simulation

# ## Main Sim Loop ##
while t < end_time:
    t_next_plot = t + ts_plotting
    while t < t_next_plot:
        # update carrier dynamics
        car_dyn.update(t)
        
        # do wind
        ACfx = 0.
        ACfy = 10.
        ACfz = 50.
        ACl = 1.
        ACm = 1.
        ACn = 1.
        ACdeltat = 1.
        ACpn = 1 + t
        ACpe = 2.
        ACpd = 5.
        ACu = 1.
        ACv = 2.
        ACw = 3.
        ACphi = np.sin(10)
        ACtheta = np.cos(10)
        ACpsi = np.tan(10)
        ACp = 1.
        ACq = 2.
        ACr = 3.
        ACdeltaa = 1.
        ACdeltae = 2.
        ACdeltar = 3.

        # autopilot
        carpn = 1 + t
        carpe = 0.
        carpd = 0.
        carphi = np.sin(10)
        cartheta = np.sin(50)
        carpsi = np.cos(10)
        caru = 1.
        carv = 2.
        carw = 3.
        # aero
        
        # dynamics

        # anim update
        
        
        # iterate time
        t += ts_simulation
        
    # do plotting for aircraft
    simtime.append(t)
    ACfxs.append(ACfx)
    ACfys.append(ACfy)
    ACfzs.append(ACfz)
    ACls.append(ACl)
    ACms.append(ACm)
    ACns.append(ACn)
    ACtht.append(ACdeltat)
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
    ACdas.append(ACdeltaa)
    ACdes.append(ACdeltae)
    ACdrs.append(ACdeltar)

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
    ACthrottle.bar(0, ACdeltat)
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
    carposition.grid('major')
    carposition.set_ylabel('Position (m)')
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