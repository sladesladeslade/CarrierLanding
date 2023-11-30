# Autopilot Class
# slabe and bemb probabyl
# hop ehe work

import numpy as np
import lib.ACgains as G


class autopilot():
    def __init__(self, dt, alt_hold_zone):
        self.dt = dt
        self.altitude_hold_zone = alt_hold_zone
        self.initialize_integrator = 1.
        self.altitude_state = 0.
        
    
    def update(self, u, h_land, approach=False, fail=True):
        # get info from the "state" thing
        t, w, phi, theta, chi, p, q, r, Va, h, Va_c, h_c, chi_c, alpha_c, alpha, w_c = u
        goround = False
        
        ### lateral autopilot ###
        if t == 0 and approach == False:
            delta_r = 0
            phi_c = self.course_hold(chi_c, chi, r, 1, self.dt)
            delta_a = self.roll_hold(phi_c, phi, p, 1, self.dt)
        elif approach == False:
            delta_r = 0
            phi_c = self.course_hold(chi_c, chi, r, 0, self.dt)
            delta_a = self.roll_hold(phi_c, phi, p, 0, self.dt)
        
        ### longitudinal autopilot ###
        if t == 0:
            # check height to see which altitude state in (@ start)
            if approach == True:
                self.altitude_state = "Approach"
            elif h <= h_c - self.altitude_hold_zone:
                self.altitude_state = "Climb"
            elif h >= h_c + self.altitude_hold_zone:
                self.altitude_state = "Descend"
            else:
                self.altitude_state = "Hold"
            self.initialize_integrator = 1

        # check for approach or landing
        if approach == True:
            self.altitude_state = "Approach"
            self.initialize_integrator = 1
            if h <= h_land:
                self.altitude_state = "Land"
                self.initialize_integrator = 1
            
        # go around
        if self.altitude_state == "goaround":
            self.altitude_state = "Climb"
        
        # climb
        if self.altitude_state == "Climb":
            # max throttle and hold pitch angle
            delta_t = 1
            theta_c = self.airspeed_hold_pitch(Va_c, Va, self.initialize_integrator, self.dt)
            
            # if above height than set to hold
            if h >= h_c - self.altitude_hold_zone:
                self.altitude_state = "Hold"
                self.initialize_integrator = 1
            else:
                self.initialize_integrator = 0
            
        # descent
        elif self.altitude_state == "Descend":
            # 0 throttle and hold pitch angle
            delta_t = 0
            theta_c = self.airspeed_hold_pitch(Va_c, Va, self.initialize_integrator, self.dt)

            # if below height than set to hold
            if h <= h_c + self.altitude_hold_zone:
                self.altitude_state = "Hold"
                self.initialize_integrator = 1
            else:
                self.initialize_integrator = 0
        
        # alt hold
        elif self.altitude_state == "Hold":
            # set throttle and pitch to hold const alt
            delta_t = self.airspeed_hold_throttle(Va_c, Va, self.initialize_integrator, self.dt)
            theta_c = self.altitude_hold(h_c, h, self.initialize_integrator, self.dt)
            
            # if too low climb
            if h <= h_c - self.altitude_hold_zone:
                self.altitude_state = "Climb"
                self.initialize_integrator = 1
                
            # if too high descend
            elif h >= h_c + self.altitude_hold_zone:
                self.altitude_state = "Descend"
                self.initialize_integrator = 1
            else:
                self.initialize_integrator = 0
                
        # approach
        elif self.altitude_state == "Approach":
            # hold alpha constant
            delta_e = self.pitch_hold(alpha_c, alpha, q, self.initialize_integrator, self.dt)
            
            # then get w
            delta_t = self.w_hold(w_c, w, self.initialize_integrator, self.dt)
            
            # also get course
            delta_r = 0
            phi_c = self.course_hold(chi_c, chi, r, self.initialize_integrator, self.dt)
            delta_a = self.roll_hold(phi_c, phi, p, self.initialize_integrator, self.dt)
            
            # reset flag
            self.initialize_integrator = 0
            
        # landing
        elif self.altitude_state == "Land":
            if fail == True:
                # keep holding alpha
                delta_e = self.pitch_hold(alpha_c, alpha, q, self.initialize_integrator, self.dt)
                
                # cut throttle
                delta_t = 0.
                
                # also get course
                delta_r = 0.
                phi_c = self.course_hold(chi_c, chi, r, self.initialize_integrator, self.dt)
                delta_a = self.roll_hold(0., phi, p, self.initialize_integrator, self.dt)
                
                # reset flag
                self.initialize_integrator = 0
            elif fail == False:
                self.altitude_state = "goaround"
                goround = True
                delta_r = 0.
                phi_c = self.course_hold(chi_c, chi, r, self.initialize_integrator, self.dt)
                delta_a = self.roll_hold(0., phi, p, self.initialize_integrator, self.dt)
                delta_t = 1.
                theta_c = self.airspeed_hold_pitch(Va_c, Va, self.initialize_integrator, self.dt)
                delta_e = self.pitch_hold(theta_c, theta, q, 1, self.dt)    
        
        # set elevator to hold pitch
        if t == 0 and approach == False:
            delta_e = self.pitch_hold(theta_c, theta, q, 1, self.dt)
        elif approach == False:
            delta_e = self.pitch_hold(theta_c, theta, q, 0, self.dt)
        
        return delta_e, delta_a, delta_r, delta_t, goround
    
    
    def roll_hold(self, phi_c, phi, p, flag, dt):
        limit1 = np.deg2rad(45)
        limit2 = -np.deg2rad(45)
        
        kp = G.kpphi
        kd = G.kdphi
        ki = G.kiphi
        
        if flag == 1:
            self.roll_integrator = 0
            self.roll_differentiator = 0
            self.roll_error_d1 = 0
        
        error = phi_c - phi
        self.roll_integrator = self.roll_integrator + (dt/2)*(error + self.roll_error_d1)
        self.roll_differentiator = p
        self.roll_error_d1 = error
        
        u = kp*error + ki*self.roll_integrator + kd*self.roll_differentiator
        
        u_sat = self.sat(u, limit1, limit2)
        if ki != 0:
            self.roll_integrator = self.roll_integrator + dt/ki*(u_sat - u)
        
        return u_sat

        
    def course_hold(self, chi_c, chi, r, flag, dt):
        limit1 = np.deg2rad(45)
        limit2 = -np.deg2rad(45)
        
        kp = G.kpchi
        kd = G.kdchi
        ki = G.kichi
        
        if flag == 1:
            self.course_integrator = 0
            self.course_differentiator = 0
            self.course_error_d1 = 0
        
        error = chi_c - chi
        self.course_integrator = self.course_integrator + (dt/2)*(error + self.course_error_d1)
        self.course_differentiator = r
        self.course_error_d1 = error
        
        u = kp*error + ki*self.course_integrator + kd*self.course_differentiator
        
        u_sat = self.sat(u, limit1, limit2)
        if ki != 0:
            self.course_integrator = self.course_integrator + dt/ki*(u_sat - u)
        
        return u_sat

        
    def pitch_hold(self, theta_c, theta, q, flag, dt):
        limit1 = np.deg2rad(45)
        limit2 = -np.deg2rad(45)
        
        kp = G.kptheta
        kd = G.kdtheta
        ki = G.kitheta
        
        if flag == 1:
            self.pitch_integrator = 0
            self.pitch_differentiator = 0
            self.pitch_error_d1 = 0
        
        error = theta_c - theta
        self.pitch_integrator = self.pitch_integrator + (dt/2)*(error + self.pitch_error_d1)
        self.pitch_differentiator = q
        self.pitch_error_d1 = error
        
        u = kp*error + ki*self.pitch_integrator + kd*self.pitch_differentiator
        
        u_sat = self.sat(u, limit1, limit2)
        if ki != 0:
            self.pitch_integrator = self.pitch_integrator + dt/ki*(u_sat - u)
        
        return u_sat

        
    def airspeed_hold_pitch(self, Va_c, Va, flag, dt):
        limit1 = np.deg2rad(20)
        limit2 = -np.deg2rad(20)
        
        kp = G.kpa_p
        kd = G.kda_p
        ki = G.kia_p
        
        if flag == 1:
            self.ahp_integrator = 0
            self.ahp_differentiator = 0
            self.ahp_error_d1 = 0
        
        tau = 5
        error = Va_c - Va
        error *= -1
        self.ahp_integrator = self.ahp_integrator + (dt/2)*(error + self.ahp_error_d1)
        self.ahp_differentiator = (2*tau - dt)/(2*tau + dt)*self.ahp_differentiator + 2/(2*tau + dt)*(error - self.ahp_error_d1)
        self.ahp_error_d1 = error
        
        u = kp*error + ki*self.ahp_integrator + kd*self.ahp_differentiator

        u_sat = self.sat(u, limit1, limit2)
        if ki != 0:
            self.ahp_integrator = self.ahp_integrator + dt/ki*(u_sat - u)
        
        return u_sat
        
        
    def airspeed_hold_throttle(self, Va_c, Va, flag, dt):
        limit1 = 1.
        limit2 = 0.
        
        kp = G.kpa_t
        kd = G.kda_t
        ki = G.kia_t
        
        if flag == 1:
            self.aht_integrator = 0
            self.aht_differentiator = 0
            self.aht_error_d1 = 0
        
        tau = 5
        error = Va_c - Va
        error *= -1
        self.aht_integrator = self.aht_integrator + (dt/2)*(error + self.aht_error_d1)
        self.aht_differentiator = (2*tau - dt)/(2*tau + dt)*self.aht_differentiator + 2/(2*tau + dt)*(error - self.aht_error_d1)
        self.aht_error_d1 = error
        
        u = kp*error + ki*self.aht_integrator + kd*self.aht_differentiator
        
        u_sat = self.sat(u, limit1, limit2)
        if ki != 0:
            self.aht_integrator = self.aht_integrator + dt/ki*(u_sat - u)
        
        return u_sat
        
        
    def altitude_hold(self, h_c, h, flag, dt):
        limit1 = np.deg2rad(45)
        limit2 = -np.deg2rad(45)
        
        kp = G.kpa_h
        kd = G.kda_h
        ki = G.kia_h
        
        if flag == 1:
            self.ah_integrator = 0
            self.ah_differentiator = 0
            self.ah_error_d1 = 0
        
        tau = 5
        error = h_c - h
        self.ah_integrator = self.ah_integrator + (dt/2)*(error + self.ah_error_d1)
        self.ah_differentiator = (2*tau - dt)/(2*tau + dt)*self.ah_differentiator + 2/(2*tau + dt)*(error - self.ah_error_d1)
        self.ah_error_d1 = error
        
        u = kp*error + ki*self.ah_integrator + kd*self.ah_differentiator

        u_sat = self.sat(u, limit1, limit2)
        if ki != 0:
            self.ah_integrator = self.ah_integrator + dt/ki*(u_sat - u)
        
        return u_sat
    
    
    def alpha_hold(self, alpha_c, alpha, q, flag, dt):
        limit1 = np.deg2rad(45)
        limit2 = -np.deg2rad(45)
        
        kp = 1.
        kd = 5.
        ki = 1.
        
        if flag == 1:
            self.alphh_integrator = 0
            self.alphh_differentiator = 0
            self.alphh_error_d1 = 0
            
        error = alpha_c - alpha
        self.alphh_integrator = self.alphh_integrator + (dt/2)*(error + self.alphh_error_d1)
        self.alphh_differentiator = q
        self.alphh_error_d1 = error
        
        u = kp*error + ki*self.alphh_integrator + kd*self.alphh_differentiator
        print(u)
        u_sat = self.sat(u, limit1, limit2)
        if ki != 0:
            self.alphh_integrator = self.alphh_integrator + dt/ki*(u_sat - u)
        
        return u_sat
    
    
    def w_hold(self, w_c, w, flag, dt):
        limit1 = 1.
        limit2 = 0.
        
        kp = -0.3
        kd = -7.
        ki = -1.
        
        if flag == 1:
            self.w_integrator = 0
            self.w_differentiator = 0
            self.w_error_d1 = 0
            
        tau = 5
        error = w_c - w
        self.w_integrator = self.w_integrator + (dt/2)*(error + self.w_error_d1)
        self.w_differentiator = (2*tau - dt)/(2*tau + dt)*self.w_differentiator + 2/(2*tau + dt)*(error - self.w_error_d1)
        self.w_error_d1 = error
        
        u = kp*error + ki*self.w_integrator + kd*self.w_differentiator

        u_sat = self.sat(u, limit1, limit2)
        if ki != 0:
            self.w_integrator = self.w_integrator + dt/ki*(u_sat - u)
        
        return u_sat


    @staticmethod
    def sat(inn, up_limit, low_limit):
        if inn > up_limit:
            out = up_limit
        elif inn < low_limit:
            out = low_limit
        else:
            out = inn
        return out