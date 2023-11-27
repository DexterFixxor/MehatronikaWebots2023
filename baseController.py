import numpy as np

def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2*np.pi
    while angle < -np.pi:
        angle += 2*np.pi
    return angle    
    

class BaseController():
    
    def __init__(self, v_max, w_max, Kp_lin, Kp_w):
        
        self.v_max = v_max
        self.w_max = w_max
        self.Kp_lin = Kp_lin
        self.Kp_w = Kp_w
        
        self.eps_lin = 0.01     # [m]
        self.eps_w   = 0.01     # [rad]
        
        self.phase = 0
        
        self.x_d = 0
        self.y_d = 0
        self.theta_d = 0
        
    def setDesired(self, x, y, theta):
        self.x_d = x
        self.y_d = y
        self.theta_d = theta
        self.phase = 0
        
    def calculateOutput(self, x_m, y_m, theta_m):
        v = 0
        w = 0
        done = False
        
        phi = np.arctan2(self.y_d - y_m, self.x_d - x_m)
        dist = np.sqrt((self.x_d - x_m)**2 + (self.y_d - y_m)**2)
        phi_prim_error = normalize_angle(self.theta_d - theta_m)
        phi_error = normalize_angle(phi - theta_m)
     
        if self.phase == 0:
            w = self.Kp_w * phi_error
            v = 0
            
            if np.abs(w) < 0.001 and np.abs(np.rad2deg(phi_error)) < 0.01:
                self.phase = 1
                print("Ending phase 1")
        elif self.phase == 1:
            w = self.Kp_w * phi_error
            v = self.Kp_lin * dist
            
            if np.abs(phi_error) > 0.1:
                v = -v
                
            if np.abs(w) < 0.001 and np.abs(v) < 0.001 and dist < 0.005:
                self.phase = 2            
                print("Ending phase 2")
        elif self.phase == 2:
            w = self.Kp_w * phi_prim_error
            v = 0
            
            if np.abs(w) < 0.001 and np.abs(np.rad2deg(phi_prim_error)) < 0.1:
                self.phase = 3
                done = True        
                print("Ending phase 3")
                
        return v, w, done
        
        