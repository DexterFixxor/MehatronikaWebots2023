from controller import Robot
from controller.motor import Motor
from msgStructs import LidarMsg
import numpy as np

def normalize_angle(angle):
    while angle > np.pi:
        angle -= np.pi * 2
    while angle < - np.pi:
        angle += np.pi * 2
    return angle


class MyRobot:
    
    def __init__(self, x_init, y_init, theta_ini):
        
        self.x = x_init
        self.y = y_init
        self.theta = theta_ini
        
        self.prev_left = 0
        self.prev_right = 0
        
        self.robot = Robot()

        self.left_motor = self.robot.getMotor("left wheel motor")
        self.right_motor = self.robot.getMotor("right wheel motor")

        self.left_motor.setControlPID(50, 1, 0)
        self.right_motor.setControlPID(50, 1, 0)
        

        # init in continual mode
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # get the time step of the current world.
        self.timestep = int(self.robot.getBasicTimeStep())
        self.dt = self.timestep / 1000.0
        
        self.left_sensor = self.robot.getPositionSensor("left wheel sensor")
        self.left_sensor.enable(self.timestep)
        self.right_sensor = self.robot.getPositionSensor("right wheel sensor")
        self.right_sensor.enable(self.timestep)

        
        self.WheelR = 0.033  # [m]
        self.L = 0.178       # [m]
        
        
        # -----
        self.lidar = self.robot.getLidar("LDS-01")
        self.lidar.enable(self.timestep)
        
        
    def update_odom(self):
        
        current_left = self.left_sensor.getValue()
        current_right = self.right_sensor.getValue()
        
        w_r = (current_right - self.prev_right)/self.dt
        w_l = (current_left - self.prev_left)/self.dt
        
        self.prev_right = current_right
        self.prev_left = current_left
        
        v_l = w_l * self.WheelR
        v_r = w_r * self.WheelR
        
        v = (v_r + v_l)/2.0
        w = (v_r - v_l)/self.L
        
        self.x += v * np.cos(self.theta + w * self.dt/2.0) * self.dt
        self.y += v * np.sin(self.theta + w * self.dt/2.0) * self.dt
        self.theta = normalize_angle(self.theta + w * self.dt)
        
    def step(self):
        return self.robot.step(self.timestep)
        
    def setMotorVelocities(self, w_l, w_r):
        self.left_motor.setVelocity(w_l)
        self.right_motor.setVelocity(w_r)
        
    def setBaseVelocities(self, v, w):
        w_lin = v / self.WheelR
        w_rot = w * self.L / (2*self.WheelR)
      
        w_r = w_lin + w_rot
        w_l = w_lin - w_rot
        
        self.setMotorVelocities(w_l, w_r)   
        
        
    def getLidarData(self) -> LidarMsg:
        ranges = np.array(self.lidar.getLayerRangeImage(0))
        n_samples = self.lidar.getHorizontalResolution()
        fov = self.lidar.getFov()
        
        delta_phi = fov/n_samples
        
        angles = np.pi - np.arange(0, fov, delta_phi)
        
        msg = LidarMsg(
            angles =angles,
            ranges = ranges,
            fov = fov,
            n_samples= n_samples
        )
        
        return msg
        
                     
    
        
