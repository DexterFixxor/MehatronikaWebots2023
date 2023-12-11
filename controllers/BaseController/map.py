import numpy as np


class GridMap:
    
    
    def __init__(self, x_size, y_size, resolution):
        
        self.x_size = x_size
        self.y_size = y_size
        self.resolution = resolution
        
        self.x_grid = int(self.x_size / resolution) + 2
        self.y_grid = int(self.y_size / resolution) + 2
        
        self.grid = np.zeros((self.x_grid, self.y_grid))
        
        self.prob_plus = 0.7
        self.prob_minus = 0.3
        
    def update_map(self, x, y, theta, ranges, angles):
        
        x_values = x + ranges * np.cos(angles + theta)
        y_values = y + ranges * np.sin(angles + theta)
        
        x = np.array(x_values / self.resolution, dtype=np.int32)
        y = np.array(y_values / self.resolution, dtype=np.int32)
        
        # self.grid[self.grid < 0.5] -= self.prob_minus
        self.grid[x,y] += self.prob_plus
        
        self.grid = np.clip(self.grid, 0., 1.)
        
        
        
