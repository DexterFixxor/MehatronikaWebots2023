from controller import Robot
import numpy as np

class MyRobot():
    
    def __init__(self, x_init, y_init, theta_init):
        
        self.robot = Robot()
        
        self.TIMESTEP = int(self.robot.getBasicTimeStep())
        self.dt = self.TIMESTEP / 1000.0 # timestep in [sec]
        
        self.x = x_init
        self.y = y_init
        self.theta = theta_init
        
        self.v = 0
        self.w = 0
        
        # init motors
        self.right_motor = self.robot.getMotor("right wheel motor")
        self.left_motor = self.robot.getMotor("left wheel motor")

        self.right_motor.setPosition(float('inf'))
        self.right_motor.setVelocity(0)
        self.left_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        
         
        # Encoders (sensors)
        self.leftMotorSensor = self.robot.getPositionSensor('left wheel sensor')
        self.leftMotorSensor.enable(self.TIMESTEP)

        self.rightMotorSensor = self.robot.getPositionSensor('right wheel sensor')
        self.rightMotorSensor.enable(self.TIMESTEP)

        self.max_motor_vel = self.left_motor.getMaxVelocity()  # rad/s
        
        # robot specs
        self.wheel_radius = 0.033
        self.wheel_distance = 0.178
        
    def setMotorVelocities(self, motor_r, motor_l):
        motor_r = np.clip(motor_r, -self.max_motor_vel, self.max_motor_vel)
        motor_l = np.clip(motor_l, - self.max_motor_vel, self.max_motor_vel)
        
        self.right_motor.setVelocity(motor_r)
        self.left_motor.setVelocity(motor_l)
    
    def getMotorVelocities(self):
        """
            (Right motor velocity, left motor velocity) in [rad/s]
        """
        return self.right_motor.getVelocity(), self.left_motor.getVelocity()
    
    def setVelocities(self, v, w):
        w_r = 0
        w_l = 0
        
        # linearni udeo
        w_lin = v / self.wheel_radius
        
        # ugaoni udeo
        w_ugaono = w * self.wheel_distance / (2 * self.wheel_radius)
        
        w_r = w_lin + w_ugaono
        w_l = w_lin - w_ugaono
        
        self.setMotorVelocities(w_r, w_l)
        
    
    def updateOdom(self):
        w_r, w_l = self.getMotorVelocities()
        
        v_r = w_r * self.wheel_radius # m/s
        v_l = w_l * self.wheel_radius # m/s
        
        self.v = (v_r + v_l)/2
        self.w = (v_r - v_l)/self.wheel_distance
        
        self.x += self.v * np.cos(self.theta + self.w * self.dt/2.0) * self.dt
        self.y += self.v * np.sin(self.theta + self.w * self.dt/2.0) * self.dt
        self.theta += self.w * self.dt
        
        while self.theta > np.pi:
            self.theta -= 2*np.pi
        while self.theta < -np.pi:
            self.theta += 2*np.pi 
            
        return self.x, self.y, self.theta    
            
    def step(self):
        return self.robot.step(self.TIMESTEP)  
        
         