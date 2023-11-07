# Hangar for Anim Aircraft
# Slade Brooks
# brooksl@mail.uc.edu

import numpy as np
from stl import mesh
import os
cwd = os.getcwd()
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


# F-18
f18_mesh = mesh.Mesh.from_file(cwd + "\\obj\\f18.stl")
f18_verts = f18_mesh.vectors*np.array([-1, 1, -1])

# Aircraft Carrier
carrier_mesh = mesh.Mesh.from_file(cwd + "\\obj\\carrier.stl")
carrier_verts = carrier_mesh.vectors*np.array([-1, 1, 1])