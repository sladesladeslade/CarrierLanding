import numpy as np
import lib.signalGenerator as sig

class carrier_dynamics():
    def __init__(self, chi=None):
        B=15.5
        if chi == None:
            self.chi = np.random.uniform(0,2*np.pi)
        else:
            self.chi = chi

        self.u=B*np.cos(self.chi)
        self.v=B*np.sin(self.chi)
        self.state=np.array([[0.], #pn
                  [0.], #pe
                  [0.], # pd
                  [self.u], # u
                  [self.v], # v
                  [0.], # w
                  [0.], # phi
                  [0.], # theta
                  [self.chi], # psi
                  [0.], # p
                  [0.], # q
                  [0.]]) # r    
        
    def update(self, t, flag=False):
        if flag == True:
            self.state[6][0]=0
            self.state[7][0]=0
        else:
            delta_max = np.radians(0.1)
            phi = self.state[6][0]
            theta = self.state[7][0]
            self.old_phi= phi
            self.old_theta = theta   
            phi = sig.signalGenerator(np.random.uniform(-np.radians(10),np.radians(10)),1/3).sin(t) + \
                sig.signalGenerator(np.random.uniform(-np.radians(10),np.radians(10)),1/70).sin(t)
            theta = sig.signalGenerator(np.random.uniform(-np.radians(2),np.radians(2)),1/3).sin(t) + \
                sig.signalGenerator(np.random.uniform(-np.radians(2),np.radians(2)),1/70).sin(t)
            if np.abs(phi-self.old_phi)>delta_max:
                phi=self.old_phi
            if np.abs(theta-self.old_theta)>delta_max:
                theta=self.old_theta
            self.state[6][0]=phi
            self.state[7][0]=theta
        self.state[0][0]=self.u*t
        self.state[1][0]=self.v*t 
