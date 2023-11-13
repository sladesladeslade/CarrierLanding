# Calculate Descent Rate Required
# Slade Brooksl
# woo doggy

import numpy as np


def calcWreq(carstate, acstate):
    # find distance to boat
    distA = np.sqrt(acstate[0][0]**2 + acstate[1][0]**2) - np.sqrt(carstate[0][0]**2 + carstate[1][0]**2)
    
    # get height dif
    hdif = -carstate[2][0] - (-acstate[2][0])
    
    # find boat speed
    vB = np.sqrt(carstate[3][0]**2 + carstate[4][0]**2)
    
    # find plane speed
    vA = np.sqrt(acstate[3][0]**2 + acstate[4][0]**2)
    
    # calc required descent rate
    wreq = -(vB - vA)/distA*(hdif)
    
    return wreq