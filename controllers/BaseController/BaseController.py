import matplotlib.pyplot as plt

from robot import MyRobot
from FSM import resetPhase, compute_velocities
import numpy as np
from pathfinderController import PathFinderController
from utils import lidarMsgTCartesianSpace
from msgStructs import MapMsg



if __name__ == "__main__":
    robot = MyRobot(
        x_init= 0.25,
        y_init= 0.25,
        theta_ini= 0.0
    )
    
    map = MapMsg(width=3.0, height=2.0, resolution=0.05 )
    
    target1 = (1.25, 0.25, np.pi/2)
    target2 = (1.25, 1.25, np.pi)
    target3 = (0.25, 1.25, -np.pi/2)
    target4 = (0.25, 0.25, 0.0)
    
    targets = [target2, target4]
    
    x_ref, y_ref, theta_ref = targets.pop(0)

    fig, ax = plt.subplots(1)
    ax.imshow(map.get_map())

    while robot.step() != -1:
        robot.update_odom()
        
        v, w, done = compute_velocities(x_ref, y_ref, theta_ref, robot.x, robot.y, robot.theta)
        if len(targets) == 0 and done:
            break
                
        if done:
            x_ref, y_ref, theta_ref = targets.pop(0)
            resetPhase()
        robot.setBaseVelocities(v, w)
        
        msg = robot.getLaserScan()
        
        map.update_map((robot.x, robot.y, robot.theta), msg.ranges, msg.angles)
           
        # ax.scatter(xs, ys)
        # plt.show(block=False)
        # plt.pause(0.001)
