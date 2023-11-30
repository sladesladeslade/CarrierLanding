# Navigation Class
# Slade Brooks
# brooksl@mail.uc.edu
# this boy gonna navimagate

import numpy as np
from numpy import sin, cos, arctan2, arctan
import matplotlib.pyplot as plt


class nav():
    def __init__(self, car_chi, app_dist):
        self.car_chi = car_chi
        self.app_dist = app_dist
        
    
    def courseToApproach(self, state, car_state):
        """
        Determine course command to approach point.
        """
        # get aircraft position
        pn = state[0][0]
        pe = state[1][0]
        
        # get approach position
        an, ae = self.approachLoc(car_state)
        
        # get dists
        dn = an - pn
        de = ae - pe
        
        # do atan to get angle between points
        theta = arctan2(de, dn)
        chi_c = (theta + np.pi) % (2*np.pi) - np.pi

        return chi_c
    
    
    def courseToCar(self, state, car_state):
        """
        Determine course command to carrier landing point.
        """
        # get aircraft position
        pn = state[0][0]
        pe = state[1][0]
        
        # get carrier position
        cn, ce, _ = self.landLoc(car_state)
        
        # get Q intermediate guy
        Q = ((cos(self.car_chi)*(cn - pn) + sin(self.car_chi)*(ce - pe))/((cn - pn)**2 + (ce - pe)**2))
        
        # magnitude of d
        if (cn - pn) > 0 and (ce - pe) > 0:
            dmag = np.sqrt((Q*(cn - pn)**2) + (Q*(ce - pe)**2))
        else:
            dmag = 0.
        
        # delta d
        deltad = 0.5*dmag
        
        # actual point to target
        deln = cn - deltad*cos(self.car_chi)
        dele = ce - deltad*sin(self.car_chi)
        
        # distance btn plane and target
        dn = deln - pn
        de = dele - pe
        
        # do atan to get angle between points
        theta = arctan2(de, dn)
        chi_c = (theta + np.pi)%(2*np.pi) - np.pi

        return chi_c
        
    
    def approachLoc(self, car_state):
        """
        Determine approach point position.
        """
        # get carrier position
        cn = car_state[0][0]
        ce = car_state[1][0]
        
        # calculate location behind carrier to start approach
        ae = ce - self.app_dist*sin(self.car_chi)
        an = cn - self.app_dist*cos(self.car_chi)
        
        return an, ae
    
    
    def checkSuccess(self, car_state, pn, pe):
        """
        Determine if we gone die or nah.
        """
        ln, le, _ = self.landLoc(car_state)
        chi = car_state[8][0]
        
        # get max dists
        mep = le + 5*np.cos(chi)
        men = le - 5*np.cos(chi)
        mnp = ln + 10*np.cos(chi)
        mnn = ln - 25*np.cos(chi)
        
        # check within max
        if pn > mnp or pn < mnn or pe > mep or pe < men:
            return False
        else:
            return True
        
        
    @staticmethod
    def landLoc(car_state):
        """
        Determine landing location.
        """
        # get carrier position
        cn = car_state[0][0]
        ce = car_state[1][0]
        ch = -car_state[2][0]
        theta = car_state[7][0]
        psi = car_state[8][0]
        
        # determine location
        d = 75./2.
        ln = cn - d*np.cos(psi)
        le = ce - d*np.sin(psi)
        lh = -(14.5 + ch - d*np.sin(theta))
        
        return ln, le, lh
        

# testing
if __name__ == "__main__":
    plt.figure()
    
    # init nav class
    nav = nav(np.deg2rad(10), 100.)
    
    # carrier position
    cn = 100.
    ce = 100.
    car_state = np.array([[cn], [ce]])
    
    # determine approach location
    an, ae = nav.approachLoc(car_state)
    print(an, ae)
    print(np.sqrt((cn - an)**2 + (ce - ae)**2))
    
    # aircraft position
    pn = 250.
    pe = 605.
    state = np.array([[pn], [pe]])
    
    # plotting
    plt.scatter(ce, cn, color = "g")
    plt.scatter(ae, an, color = "r")
    plt.scatter(pe, pn, color = "k")
    
    # calc commanded angle
    chi_c = nav.courseToApproach(state, car_state)
    print(np.rad2deg(chi_c))
    
    # show plot
    plt.gca().set_aspect("equal")
    plt.grid()
    plt.xlim(-250, 250)
    plt.ylim(-250, 250)
    plt.show()