import numpy as np
from robot import normalize_angle

PHASE = 0

def compute_velocities(x_ref, y_ref, theta_ref, x_odom, y_odom, theta_odom):
    global PHASE
    w = 0
    v = 0 
     
    dist = np.sqrt((x_ref - x_odom)**2 + (y_ref - y_odom)**2)
    # ugao do tacke (x_ref, y_ref)
    phi = np.arctan2(y_ref - y_odom, x_ref - x_odom)
    phi_error = normalize_angle(phi - theta_odom)
    
    phi_prim_error = normalize_angle(theta_ref - theta_odom)
    
    eps_dist = 0.002 # [m]
    eps_theta = np.deg2rad(2)
    
    Kp_w = 2
    Kp_d = 3

    done = False
    # Rotacija
    if PHASE == 0:
        v = 0
        w = Kp_w * phi_error
        
        if np.abs(phi_error) <= eps_theta and np.abs(w) <= 0.001:
            PHASE = 1
    elif PHASE == 1:
        w = Kp_w * phi_error
        v = Kp_d * dist
        if np.abs(phi_error) > np.deg2rad(5):
            v = -v
        
        if np.abs(dist) <= eps_dist and np.abs(v) <= 0.01:
            PHASE = 2
            
    elif PHASE == 2:
        v = 0
        w = Kp_w * phi_prim_error
        
        if np.abs(phi_prim_error) <= eps_theta and np.abs(w) <= 0.01:
            PHASE = 3
            done = True
        
            
    return v, w, done


def resetPhase():
    global PHASE
    PHASE = 0