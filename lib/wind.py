# import sys
# import os
# cwd = os.getcwd()
# sys.path.append(cwd)
import numpy as np
import lib.ACparams as P
# from lib.ACdynamics import ACdynamics
import control
from lib.rotations import Rvb
from control.matlab import *
# Filter out the specific warning
import warnings; warnings.filterwarnings("ignore", category=UserWarning, module="control")


# create random gust noise
noise = np.random.default_rng()
nrange = [-1, 1]

class wind():
    def __init__(self, V_steady):
        self.Vs = V_steady
        

    def drydenGust(self, Va, dt):
        # using low altitude, light turbulence values from table in lecture slides
        L_u = L_v = 200
        L_w = 50
        sig_u = sig_v = 1.06
        sig_w = 0.7

        au=sig_u*np.sqrt(2*Va/L_u)
        av=sig_v*np.sqrt(3*Va/L_v)
        aw=sig_w*np.sqrt(3*Va/L_w)
 
        # transfer functions
        num_u=[0,au]
        den_u=[1,Va/L_u]
        sys_u=tf(num_u,den_u)
 
        num_v=[av,av*Va/(np.sqrt(3)*L_v)]
        den_v=[1,2*Va/L_v, (Va/L_v)**2]
        sys_v=tf(num_v,den_v)
 
        num_w=[aw,aw*Va/(np.sqrt(3)*L_w)]
        den_w=[1,2*Va/L_w, (Va/L_w)**2]
        sys_w=tf(num_w,den_w)

        # random noise inputs
        s_u = noise.random()
        s_v = noise.random()
        s_w = noise.random()

        # transfer function params
        T = np.array([0, dt])
        x0 = 0.0

        yu, T, x_u = lsim(sys_u, s_u, T, x0)
        yv, T, x_v = lsim(sys_v, s_v, T, x0)
        yw, T, x_w = lsim(sys_w, s_w, T, x0)

        u_wg = yu[1] * np.random.choice(nrange)
        v_wg = yv[1] * np.random.choice(nrange)
        w_wg = yw[1] * np.random.choice(nrange)
        

        return np.array([[u_wg], [v_wg], [w_wg]])
    
    def wind_char(self, state, Va, dt):
        pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = state.flatten()

        Gust = self.drydenGust(Va, dt)
        inter = Rvb(phi, theta, psi)*self.Vs + Gust

        airspeedvec = np.array([[u - inter[0][0]],
                                [v - inter[1][0]],
                                [w - inter[2][0]]])
        
        u_r, v_r, w_r = airspeedvec.flatten()
        Va = np.sqrt((u_r**2) + (v_r**2) + (w_r**2))
        alpha = np.arctan(w_r/u_r)
        beta = np.arcsin(v_r/Va)

        return Va, alpha, beta
