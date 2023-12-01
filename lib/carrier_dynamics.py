# import numpy as np
# import lib.signalGenerator as sig

# class carrier_dynamics():
#     def __init__(self, chi=None):
#         self.B=5
#         if chi == None:
#             self.chi = np.random.uniform(0,2*np.pi)
#         else:
#             self.chi = chi
#         self.u=self.B*np.cos(self.chi)
#         self.v=self.B*np.sin(self.chi)
#         self.state=np.array([[0.], #pn
#                   [0.], #pe
#                   [0.], # pd
#                   [self.u], # u
#                   [self.v], # v
#                   [0.], # w
#                   [0.], # phi
#                   [0.], # theta
#                   [self.chi], # psi
#                   [0.], # p
#                   [0.], # q
#                   [0.]]) # r    
        
#     def update(self, t, flag=False):
#         if flag == True:
#             self.state[6][0]=0
#             self.state[7][0]=0
#         else:
#             phi = self.state[6][0]
#             theta = self.state[7][0]
#             phi = sig.signalGenerator(np.radians(5), 1/30).sin(t+np.pi/4.3)
#             theta = sig.signalGenerator(np.radians(1), 1/30).sin(t + np.pi/3.5)
#             # self.chi+=sig.signalGenerator(np.radians(.001), 1/30).sin(t)
#             pd = sig.signalGenerator(2, 1/30).sin(t + np.pi/3.5)
#             self.state[6][0]=phi
#             self.state[7][0]=theta
#             self.state[8][0]=self.chi
#             self.state[2][0]=pd
#             # might need to move before
#             self.u=self.B*np.cos(self.chi)
#             self.v=self.B*np.sin(self.chi)
#         self.state[0][0]=self.u*t
#         self.state[1][0]=self.v*t 

import numpy as np
import lib.signalGenerator as sig

class carrier_dynamics():
    def __init__(self, chi=None):
        self.B=5
        if chi == None:
            self.chi = np.random.uniform(0,2*np.pi)
        else:
            self.chi = chi
        self.u=self.B*np.cos(self.chi)
        self.v=self.B*np.sin(self.chi)
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
            phi = self.state[6][0]
            theta = self.state[7][0]
            phi = sig.signalGenerator(np.radians(5), 1/30).sin(t+np.pi/4.3)
            theta = sig.signalGenerator(np.radians(1), 1/30).sin(t + np.pi/3.5)
            # self.chi+=sig.signalGenerator(np.radians(.001), 1/30).sin(t)
            pd = sig.signalGenerator(2, 1/30).sin(t + np.pi/3.5)
            self.state[6][0]=phi
            self.state[7][0]=theta
            self.state[8][0]=self.chi
            self.state[2][0]=pd
            # might need to move before
            self.u=self.B*np.cos(self.chi)
            self.v=self.B*np.sin(self.chi)
        self.state[0][0]=self.u*t
        self.state[1][0]=self.v*t 