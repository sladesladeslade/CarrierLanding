import numpy as np
import lib.signalGenerator as sig

class carrier_dynamics():
    def __init__(self, chi=None):
        B=15.5
        if chi == None:
            self.chi = np.random.uniform(0,2*np.pi)
        else:
            self.chi = chi

        u=B*np.sin(self.chi)
        v=B*np.cos(self.chi)
        self.state=np.array([[0.], #pn
                  [0.], #pe
                  [0.], # pd
                  [u], # u
                  [v], # v
                  [0.], # w
                  [0.], # phi
                  [0.], # theta
                  [0.], # psi
                  [0.], # p
                  [0.], # q
                  [0.]]) # r    
        
    def update(self, t):
        delta_max = 0.5
        phi = self.state[5][0]
        theta = self.state[6][0]
        self.old_phi= phi
        self.old_theta = theta
        phi = sig.signalGenerator(np.random.uniform(-2,2),1/300).sin(t) + sig.signalGenerator(np.random.uniform(-2,2),1/700).sin(t)
        theta = sig.signalGenerator(np.random.uniform(-2,2),1/300).sin(t) + sig.signalGenerator(np.random.uniform(-2,2),1/700).sin(t)
        if np.abs(phi-self.old_phi)>delta_max:
            phi=self.old_phi
        if np.abs(theta-self.old_theta)>delta_max:
            theta=self.old_theta
        self.state[6][0]=phi
        self.state[7][0]=theta
