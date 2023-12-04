import numpy as np

class LaserScan:
    
    def __init__(self, fov = 2*np.pi, n = 360, ranges = []):
        
        self.fov = fov
        self.n = n
        self.ranges = ranges
        self.angles = np.arange(0, fov, fov/n)


class MapMsg:
    
    def __init__(self, width, height, resolution):
        
        self.res = resolution
        self.w = width  # [m]
        self.h = height # [m]
        
        self.size_x = int(self.w/resolution) + 6
        self.size_y = int(self.h/resolution) + 6
        
        self.grid = np.ones((self.size_x, self.size_y), dtype=np.float32) / 2.0
        
        
    def addObstacle(self, x, y):
        """ x and y are in meters"""
        
        i = int(x / self.res) + 3
        j = int(y / self.res) + 3

        if i < self.size_x and j < self.size_y:
            self.grid[i, j] = 1.0
