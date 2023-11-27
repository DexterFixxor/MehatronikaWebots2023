from robot import MyRobot
import numpy as np

from baseController import BaseController

robot = MyRobot(
    0.25, # x
    0.25, # y
    0.0   # theta
)

controller = BaseController(v_max=6, w_max=6, Kp_lin=1, Kp_w=2)

targets_list = [
    (1.0, 0.25, np.pi/2),
    (1.0, 1.0, np.pi),
    (0.25, 1.0, -np.pi/2)
]

done = True
while robot.step() != -1:
    
    if len(targets_list) > 0 and done:
        x_d, y_d, theta_d = targets_list.pop(0)
        controller.setDesired(x_d, y_d, theta_d)
        
    x, y, theta = robot.updateOdom()
    v, w, done = controller.calculateOutput(x,y,theta)

    robot.setVelocities(v, w)   
    