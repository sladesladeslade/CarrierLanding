
import numpy as np
import lib.ACparams as P
from lib.ACtrim import ComputeTrim


# set xtrim and utrim
Va = 250.
R = np.inf
Y = np.deg2rad(0.)
trim = ComputeTrim()
x_trim, u_trim = trim.compute_trim(Va, Y, R)

# do the tf stuff to get the answers and woo
Va_trim = np.sqrt(x_trim[3]**2 + x_trim[4]**2 + x_trim[5]**2)
alpha_trim = np.arctan(x_trim[5]/x_trim[3])
beta_trim = np.arctan(x_trim[4]/Va_trim)
theta_trim = x_trim[7]
gamma = P.jx * P.jz - P.jxz ** 2
gamma3 = P.jz / gamma
gamma4 = P.jxz / gamma
Cpp = gamma3 * P.C_ell_p + gamma4 * P.C_n_p
Cpdelta_a = gamma3 * P.C_ell_delta_a + gamma4 * P.C_n_delta_a
a_phi1 = -.5 * P.rho * Va_trim**2 * P.S_wing * P.b * Cpp * (P.b / (2 * Va_trim))
a_phi2 = .5 * P.rho * Va_trim**2 * P.S_wing * P.b * Cpdelta_a
a_theta1 = -(P.rho * Va_trim**2 * P.c * P.S_wing)/(2 * P.jy) * P.C_m_q * (P.c/(2 * Va_trim))
a_theta2 = -(P.rho * Va_trim**2 * P.c * P.S_wing)/(2 * P.jy) * P.C_m_alpha
a_theta3 = (P.rho * Va_trim**2 * P.c * P.S_wing)/(2 * P.jy) * P.C_m_delta_e
a_V1 = ((P.rho * Va_trim * P.S_wing)/P.M) * (P.C_D_0 + (P.C_D_alpha * alpha_trim) \
        + (P.C_D_delta_e * u_trim[0])) + (P.rho * P.S_prop)/(P.M) * P.C_prop * Va_trim
a_V2 = (P.rho * P.S_prop)/(P.M) * P.C_prop * P.k_motor**2 * u_trim[3]
a_V3 = P.gravity * np.cos(theta_trim - alpha_trim)

# zeta
zeta = .707

# roll
tr_roll = 0.5
wn_roll = 2.2/tr_roll
kpphi = (wn_roll**2)/a_phi2
kdphi = (2*zeta*wn_roll - a_phi1)/a_phi2
kiphi = 0

# course hold
tr_course = 20.
wn_course = 2.2/tr_course
kpchi = (2*zeta*wn_course*Va_trim)/P.gravity
kdchi = -3.
kichi = ((wn_course**2)*Va_trim)/P.gravity

# pitch attitude hold
zeta_pitch = 0.1
tr_pitch = 0.1
wn_pitch = 2.2/tr_pitch
kptheta = ((wn_pitch**2) - a_theta2)/a_theta3
kdtheta = (2*zeta_pitch*wn_pitch - a_theta1)/a_theta3
kitheta = 0
ktheta_DC = (kptheta*a_theta3)/(a_theta2 + kptheta*a_theta3)

# altitude from pitch gain
tr_altitude = 1.
wn_altitude = 2.2/tr_altitude
kpa_h = (2*zeta*wn_altitude)/(ktheta_DC*Va_trim)
kda_h = 0
kia_h = (wn_altitude**2)/(ktheta_DC*Va_trim)

# airspeed from pitch
tr_airspeed = 0.01
wn_airspeed = 2.2/tr_airspeed
kpa_p = (a_V1 - 2*zeta*wn_airspeed)/ktheta_DC
kda_p = 0
kia_p = (wn_airspeed**2)/(ktheta_DC*P.gravity)

# airspeed from throttle
tr_throttle = 15.
wn_throttle = 2.2/tr_throttle
kpa_t = (2*zeta*wn_throttle - a_V1)/a_V2
kda_t = 0
kia_t = (wn_throttle**2)/a_V2