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

forces = anim.fig.add_subplot(3,3,2)
# forcep = forces.get_position(); forcep.x0 += 0.1; forcep.x1 += 0.075; forces.set_position(forcep)
moments = anim.fig.add_subplot(3,3,5)
# momentp = moments.get_position(); momentp.x0 += 0.1; momentp.x1 += 0.075; moments.set_position(momentp)
position = anim.fig.add_subplot(3,3,8)
# positionp = position.get_position(); positionp.x0 += 0.1; positionp.x1 += 0.075; position.set_position(positionp)
velocity = anim.fig.add_subplot(4,3,3)
# velocityp = velocity.get_position(); velocityp.x0 += 0.1; velocityp.x1 += 0.075; velocity.set_position(velocityp)
angles = anim.fig.add_subplot(4,3,6)
# anglep = angles.get_position(); anglep.x0 += 0.1; anglep.x1 += 0.075; angles.set_position(anglep)
rates = anim.fig.add_subplot(4,3,9)
# ratep = rates.get_position(); ratep.x0 += 0.1; ratep.x1 += 0.075; rates.set_position(ratep)
defl = anim.fig.add_subplot(4,3,12)
# deflp = defl.get_position(); deflp.x0 += 0.1; deflp.x1 += 0.075; defl.set_position(deflp)
throttle = plt.figure(1).add_subplot(1, 20, 1); throttlep = throttle.get_position(); throttlep.x0-=0.075; throttlep.x1-=0.075
throttle.set_position(throttlep)


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

carnors = []
careass = []
cardows = []
carphis = []
carpsis = []
carus = []
carvs = []
carus = []

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
        
        # autopilot
        
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
    carnors.append(carpn)
    careass.append(carpe)
    cardows.append(carpd)
    carphis.append(carphi)
    carpsis.append(carpsi)
    carus.append(caru)
    carvs.append(carv)
    carus.append(caru)

        # plot data on subplots
    angles.clear()
    forces.clear()
    moments.clear()
    rates.clear()
    defl.clear()
    position.clear() 
    velocity.clear() 
    throttle.clear()
    throttle.bar(0,ACdeltat)
    angles.plot(simtime, np.rad2deg(phis), "g-", label="$\phi$")
    angles.plot(simtime, np.rad2deg(thetas), "r-", label="$\ttheta$")
    angles.plot(simtime, np.rad2deg(psis), "b-", label="$\psi$")
    forces.plot(simtime, fxs, "g-", label="Fx")
    forces.plot(simtime, fys, "r-", label="Fy")
    forces.plot(simtime, fzs, "b-", label="Fz")
    moments.plot(simtime, ls, "g-", label="l")
    moments.plot(simtime, ms, "r-", label="m")
    moments.plot(simtime, ns, "b-", label="n")
    position.plot(simtime, pns, "m-", label="North")
    position.plot(simtime, pes, "y-", label="East")
    position.plot(simtime, pds, "k-", label="Down")
    rates.plot(simtime, np.rad2deg(ps), "g-", label="p")
    rates.plot(simtime, np.rad2deg(qs), "r-", label="q")
    rates.plot(simtime, np.rad2deg(rs), "b-", label="r")
    defl.plot(simtime, des, "g-", label="e")
    defl.plot(simtime, das, "r-", label="a")
    defl.plot(simtime, drs, "b-", label="r")
    velocity.plot(simtime, us, "g-", label="u")
    velocity.plot(simtime, vs, "r-", label="v")
    velocity.plot(simtime, ws, "b-", label="w")
    throttle.set_ylabel('Throttle')
    throttle.set_ylim(0, 1)
    angles.grid('major')
    angles.set_ylabel('Angle (deg)')
    angles.legend(loc='upper right')
    forces.grid('major')
    forces.set_ylabel('Force (N)')
    forces.legend(loc='upper right')
    moments.grid('major')
    moments.set_ylabel('Moment')
    moments.legend(loc='upper right')
    position.grid('major')
    position.set_xlabel('Time (s)')
    position.set_ylabel('Position (m)')
    position.legend(loc='upper right')
    rates.grid('major')
    rates.set_ylabel('Ang Velocity (deg/s)')
    rates.legend(loc='upper right')
    defl.grid('major')
    defl.set_xlabel('Time (s)')
    defl.set_ylabel('Deflection (deg)')
    defl.legend(loc='upper right')
    velocity.grid('major')
    velocity.set_ylabel('Velocity (m/s)')
    velocity.legend(loc='upper right')



    # check for keybaord press
    plt.pause(0.01)
    if keyboard.is_pressed('q'): break