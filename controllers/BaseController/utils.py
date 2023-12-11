import numpy as np


def transformLidarData(x,y,theta, ranges, angles):
    
    x_tf = x + np.cos(theta + angles) * ranges
    y_tf = y + np.sin(theta + angles) * ranges
    
    return x_tf, y_tf
