# 3D Animation Class
# Slade Brooks
# brooksl@mail.uc.edu
# stolen from my original one :)
# updated to handle 2 objects


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from lib.rotations import Euler2Rotation


class animation():
    """
    Base class for 3D animations in MPL.
    """
    
    def __init__(self, limits1=10, alpha=0.6): 
        # start plot
        self.fig = plt.figure("3D Animation")
        self.ax1 = self.fig.add_subplot(111, projection="3d")
        self.ax1.set_xlim([-limits1, limits1])
        self.ax1.set_ylim([-limits1, limits1])
        self.ax1.set_zlim([-limits1, limits1])
        self.lim1 = limits1
        self.ax1.set_xlabel('East (m)')
        self.ax1.set_ylabel('North (m)')
        self.ax1.set_zlabel('Height (m)')
        self.alpha = alpha
        
        # set init flag
        self.flag_init = True


    def rotmove(self, verts, n, e, d, phi, theta, psi):
        """
        Applies rotation and translation to object of given vertices.
        """
        # position matrix
        pos = np.array([n, e, d])
        posm = np.tile(pos.copy(), (np.shape(verts)[0], 1))
        
        # rotation matrix and rotate then translate verts
        R = Euler2Rotation(phi, theta, psi)
        vertsRot = np.matmul(R, verts.T).T

        vertsTrans = vertsRot + posm
        
        # rotation matrix for plotting and rotate
        R_plot = np.array([[0, 1, 0],
                           [1, 0, 0],
                           [0, 0, -1]])
        newverts = np.matmul(R_plot, vertsTrans.T).T
        
        return newverts
    
    
    def update(self, verts1, verts2, state1, state2, facecolors1=[], facecolors2=[]):
        """
        Updates position and rotation of object and draws it.
        """
        # draw obj 1
        n1, e1, d1, _, _, _, phi1, theta1, psi1, _, _, _ = state1.flatten()
        self.drawObj1(verts1, n1, e1, d1, phi1, theta1, psi1, facecolors=facecolors1)
        
        # draw obj 2
        n2, e2, d2, _, _, _, phi2, theta2, psi2, _, _, _ = state2.flatten()
        self.drawObj2(verts2, n2, e2, d2, phi2, theta2, psi2, facecolors=facecolors2)
        
        # set init flag
        if self.flag_init == True:
            self.flag_init = False
        
        
    def drawObj1(self, verts, n, e, d, phi, theta, psi, facecolors=[]):
        """
        Draws object and its faces.
        """
        # reshape to just vertices
        verts = np.reshape(verts.copy(), (-1, 3))
        
        # update positions
        objverts = self.rotmove(verts, n, e, d, phi, theta, psi)
        
        # get back to faces
        faces = np.reshape(objverts.copy(), (-1, 3, 3))
        
        # collect polys if first time
        if self.flag_init is True:
            poly = Poly3DCollection(faces, facecolors=facecolors, alpha=self.alpha)
            self.obj1 = self.ax1.add_collection3d(poly)
            self.ax1.set_xlim([e-self.lim1, e+self.lim1])
            self.ax1.set_ylim([n-self.lim1, n+self.lim1])
            self.ax1.set_zlim([-d-self.lim1, -d+self.lim1])
        # otherwise update vert location
        else:
            self.obj1.set_verts(faces)
            self.ax1.set_xlim([e-self.lim1, e+self.lim1])
            self.ax1.set_ylim([n-self.lim1, n+self.lim1])
            self.ax1.set_zlim([-d-self.lim1, -d+self.lim1])
            
            
    def drawObj2(self, verts, n, e, d, phi, theta, psi, facecolors=[]):
        """
        Draws object and its faces.
        """
        # reshape to just vertices
        verts = np.reshape(verts.copy(), (-1, 3))
        
        # update positions
        objverts = self.rotmove(verts, n, e, d, phi, theta, psi)
        
        # get back to faces
        faces = np.reshape(objverts.copy(), (-1, 3, 3))
        
        # collect polys if first time
        if self.flag_init is True:
            poly = Poly3DCollection(faces, facecolors=facecolors, alpha=self.alpha)
            self.obj2 = self.ax1.add_collection3d(poly)
        # otherwise update vert location
        else:
            self.obj2.set_verts(faces)