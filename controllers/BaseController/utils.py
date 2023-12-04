import numpy as np


def lidarMsgTCartesianSpace(x,y,theta, ranges, angles):
    
    xs = []
    ys = []
    
    for d, phi in zip(ranges, angles):
        
        new_x = x + d * np.cos(np.pi - phi + theta)
        new_y = y + d * np.sin(np.pi - phi + theta)
        
        xs.append(new_x)
        ys.append(new_y)
        
    return xs,ys
