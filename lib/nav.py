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
        chi_c = (theta + 2*np.pi) % (2*np.pi)

        return chi_c
    
    
    def courseToPattern(self, state, car_state, point, lchi):
        """
        Determine course command to specified point.
        """
        # get aircraft position
        pn = state[0][0]
        pe = state[1][0]
        
        # car pos
        cn = car_state[0][0]
        ce = car_state[1][0]
        
        # set distance
        if point == 0:
            an = 50*sin(self.car_chi)
            ae = 50*cos(self.car_chi)
            h_c = 250.
        elif point == 1:
            an = 50*sin(self.car_chi) + 500*sin(np.deg2rad(90.) - self.car_chi)
            ae = 50*cos(self.car_chi) + 500*cos(np.deg2rad(90.) - self.car_chi)
            h_c = 250.
        elif point == 2:
            an = 50*sin(self.car_chi) + 500*sin(np.deg2rad(90.) - self.car_chi) + 650*cos(np.deg2rad(90.) - self.car_chi)
            ae = 50*cos(self.car_chi) + 500*cos(np.deg2rad(90.) - self.car_chi) - 650*sin(np.deg2rad(90.) - self.car_chi)
            h_c = 175.
        elif point == 3:
            An, Ae = self.approachLoc(car_state)
            an = An - 600*cos(np.deg2rad(90.) - self.car_chi)
            ae = Ae - 600*sin(np.deg2rad(90.) - self.car_chi)
            h_c = 100.
            
        # get dists
        nan = cn + an
        nae = ce + ae
        if point == 3: nan = an
        dn = nan - pn
        de = nae - pe
        
        # do atan to get angle between points
        theta = arctan2(de, dn)
        chi_c = (theta + 2*np.pi) % (2*np.pi)
        lchi = (lchi + 2*np.pi) % (2*np.pi)
        if sin(chi_c - lchi) > 0: chi_c = chi_c
        elif sin(chi_c - lchi) < 0: chi_c = chi_c - 2*np.pi
        else: chi_c = chi_c
        return chi_c, h_c, nan, nae
    
    
    def courseToCar(self, state, car_state, an, ae):
        """
        Determine course command to carrier landing point.
        """
        # get aircraft position
        pn = state[0][0]
        pe = state[1][0]
        
        # get carrier position
        cn, ce, _ = self.landLoc(car_state)

        # get distance between a/c and carrier
        lmag = np.sqrt(((cn-pn)**2)+((ce-pe)**2))
        
        # get Q intermediate guy
        Q = ((cos(self.car_chi)*(cn - pn) + sin(self.car_chi)*(ce - pe))/((cn - pn)**2 + (ce - pe)**2))
        
        # magnitude of d
        if (cn - pn) > 0 and (ce - pe) > 0:
            dmag = np.sqrt((Q*(cn - pn)**2) + (Q*(ce - pe)**2))
        else:
            dmag = 0.
        
        # # delta d
        deltad = dmag
        
        # actual point to target
        deln = cn - deltad*cos(self.car_chi)
        dele = ce - deltad*sin(self.car_chi)
        
        # distance btn plane and target
        dn = deln - pn
        de = dele - pe
        
        # do atan to get angle between points
        theta = arctan2(de, dn)
        chi_a = (theta + np.pi)%(2*np.pi) - np.pi

        # perpendicular distance c
        cmag = np.sqrt((lmag**2)-(dmag**2))

        # set approach angle based on perp. distance
        if pe > ae:
            if cmag >= 150:
                chi_c = -10*chi_a
            elif cmag >= 125:
                chi_c = -8*chi_a
            elif cmag >= 100:
                chi_c = -6*chi_a
            elif cmag >= 50:
                chi_c = -5*chi_a
            elif cmag >= 10:
                chi_c = -3*chi_a
            elif cmag >= 5:
                chi_c = -2*chi_a
            else:
                chi_c = chi_a
        elif pe < ae:
            if cmag >= 150:
                chi_c = 10*chi_a
            elif cmag >= 125:
                chi_c = 8*chi_a
            elif cmag >= 100:
                chi_c = 6*chi_a
            elif cmag >= 50:
                chi_c = 5*chi_a
            elif cmag >= 10:
                chi_c = 3*chi_a
            elif cmag >= 5:
                chi_c = 2*chi_a
            else:
                chi_c = chi_a        



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