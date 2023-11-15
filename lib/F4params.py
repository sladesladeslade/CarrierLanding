# import sys
# import os
# cwd = os.getcwd()
# sys.path.append(cwd)
import numpy as np

######################################################################################
                #  Inertial Params
######################################################################################
jx = 25000*1.3558179619
jy = 122200*1.3558179619
jz = 139800*1.3558179619
jxz = 2200*1.3558179619
gravity = 9.806650
mass = 39000/2.205      # kg
# inital states
states0=np.array([[0.], #pn
                  [0.], #pe
                  [0.], # pd
                  [35.], # u
                  [0.], # v
                  [0.], # w
                  [0.], # phi
                  [0.], # theta
                  [0.], # psi
                  [0.], # p
                  [0.], # q
                  [0.]]) # r

gamma = jx*jz - jxz**2
g1 = (jxz*(jx-jy+jz))/gamma
g2 = (jz*(jx-jy)+jxz**2)/gamma
g3 = jz/gamma
g4 = jxz/gamma
g5 = (jz-jx)/jy
g6 = (jxz/jy)
g7 = ((jx-jy)*jx+jxz**2)/gamma
g8 = jx/gamma

######################################################################################
                #  Aerodynamic Params
######################################################################################
S_wing        = 530/10.764
b             = 38.7/3.281
c             = 16/3.281
rho           = 1.225
e             = 0.8
AR            = b**2/S_wing
C_L_0         = 0.1
C_D_0         = 0.0205
C_m_0         = 0.0205
C_L_alpha     = 3.75
C_D_alpha     = 0.3
C_m_alpha     = -0.4
C_L_q         = 1.8
C_D_q         = 0.0
C_m_q         = -2.7
C_L_delta_e   = 0.4
C_D_delta_e   = -0.1
C_m_delta_e   = -0.58
C_D_p         = 0.0
C_Y_0         = 0.0
C_ell_0       = 0.0
C_n_0         = 0.0
C_Y_beta      = -0.68
C_ell_beta    = -0.08
C_n_beta      = 0.125
C_Y_p         = 0.0
C_ell_p       = -0.24
C_n_p         = -0.036
C_Y_r         = 0.0
C_ell_r       = 0.07
C_n_r         = -0.27
C_Y_delta_a   = 0.016
C_ell_delta_a = -0.042
C_n_delta_a   = 0.001
C_Y_delta_r   = 0.095
C_ell_delta_r = 0.006
C_n_delta_r   = -0.066