# 3D Animation Class
# Slade Brooks
# brooksl@mail.uc.edu
# stolen from my original one :)
# updated to handle 2 objects


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from rotations import Euler2Rotation


class animation():
    """
    Base class for 3D animations in MPL.
    """
    
    def __init__(self, limits=10, alpha=0.6, flag=False): 
        # start plot
        self.fig = plt.figure(1)
        if flag == True:
            self.ax = self.fig.add_subplot(1, 2, 1, projection="3d")
        else:
            self.ax = self.fig.add_subplot(projection="3d")
        self.ax.set_xlim([-limits, limits])
        self.ax.set_ylim([-limits, limits])
        self.ax.set_zlim([-limits, limits])
        self.lim = limits
        self.ax.set_xlabel('East (m)')
        self.ax.set_ylabel('North (m)')
        self.ax.set_zlabel('Height (m)')
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
    
    
    def update(self, verts, state1, state2, obj=None, facecolors=[]):
        """
        Updates position and rotation of object and draws it.
        """
        # draw obj 1
        n1, e1, d1, phi1, theta1, psi1 = state1.flatten()
        self.drawObj(verts, n1, e1, d1, phi1, theta1, psi1, obj=obj, facecolors=facecolors)
        
        # draw obj 2
        n2, e2, d2, phi2, theta2, psi2 = state2.flatten()
        self.drawObj(verts, n2, e2, d2, phi2, theta2, psi2, obj=obj, facecolors=facecolors)
        
        # set init flag
        if self.flag_init == True:
            self.flag_init = False
        
        
    def drawObj(self, verts, n, e, d, phi, theta, psi, obj=None, facecolors=[]):
        """
        Draws object and its faces.
        """
        # check for type of input
        if obj == None:
            # reshape to just vertices
            verts = np.reshape(verts.copy(), (-1, 3))
            
            # update positions
            objverts = self.rotmove(verts, n, e, d, phi, theta, psi)
            
            # get back to faces
            faces = np.reshape(objverts.copy(), (-1, 3, 3))
        else:
            # update position and get faces
            objverts = self.rotmove(verts, n, e, d, phi, theta, psi)
            faces = obj(objverts)
        
        # collect polys if first time
        if self.flag_init is True:
            poly = Poly3DCollection(faces, facecolors=facecolors, alpha=self.alpha)
            self.obj = self.ax.add_collection3d(poly)
        # otherwise update vert location
        else:
            self.obj.set_verts(faces)