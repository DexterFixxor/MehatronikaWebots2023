import numpy as np
import scipy
from typing import Tuple


class LaserScan:
    
    def __init__(self, fov = 2*np.pi, n = 360, ranges = []):
        
        self.fov = fov
        self.n = n
        self.ranges = ranges
        self.angles = np.arange(0, fov, fov/n)


class MapMsg:
    
    def __init__(self, xsize_m, ysize_m, grid_size):
        
        self.xsize_m = xsize_m
        self.ysize_m = ysize_m
        
        xsize = int(xsize_m / grid_size)
        ysize = int(ysize_m / grid_size)
        
        self.xsize = xsize+2 # Add extra cells for the borders
        self.ysize = ysize+2
        self.grid_size = grid_size # save this off for future use, in meters, resolution of a grid
        self.log_prob_map = np.zeros((self.xsize, self.ysize)) # set all to zero

        self.alpha = 0.05 + 0.0043*2 # The assumed thickness of obstacles
        self.beta = np.pi/90.0 # The assumed width of the laser beam
        self.z_max = 3.5 # The max reading from the laser

        # Pre-allocate the x and y positions of all grid positions into a 3D tensor
        # (pre-allocation = faster)
        self.grid_position_m = np.array([np.tile(np.arange(0, self.xsize*self.grid_size, self.grid_size)[:,None], (1, self.ysize)),
                                         np.tile(np.arange(0, self.ysize*self.grid_size, self.grid_size)[:,None].T, (self.xsize, 1))])

        
     
        # Log-Probabilities to add or remove from the map 
        self.prob_occupied = 0.85
        self.prob_free = 0.15
        self.l_occ = np.log(self.prob_occupied/self.prob_free)
        self.l_free = np.log(self.prob_free/self.prob_occupied)

    def update_map(self, pose, ranges, angles):
        # l0 -> prior of occupancy represented as a log-odds ratio
        #               p(mi=1)
        # l0 = log ------------------
        #            1 - p(mi=1)
        
        # log_t,i = log_t-1,i + inverse_sensor_model(mi, xt, zt) - l0
        

        dx = self.grid_position_m.copy() # A tensor of coordinates of all cells
        dx[0, :, :] -= pose[0] # A matrix of all the x coordinates of the cell
        dx[1, :, :] -= pose[1] # A matrix of all the y coordinates of the cell
        theta_to_grid = np.arctan2(dx[1, :, :], dx[0, :, :]) - pose[2] # matrix of all bearings from robot to cell

        # Wrap to +pi / - pi
        theta_to_grid[theta_to_grid > np.pi] -= 2. * np.pi
        theta_to_grid[theta_to_grid < -np.pi] += 2. * np.pi

        dist_to_grid = scipy.linalg.norm(dx, axis=0) # matrix of L2 distance to all cells from robot

        # For each laser beam
        for r, b in zip(ranges, angles):
            # Calculate which cells are measured free or occupied, so we know which cells to update
            free_mask = (np.abs(theta_to_grid - b) <= self.beta/2.0) & (dist_to_grid < (r - self.alpha/2.0))
            occ_mask = (np.abs(theta_to_grid - b) <= self.beta/2.0) & (np.abs(dist_to_grid - r) <= self.alpha/2.0)

            # Adjust the cells appropriately
            self.log_prob_map[occ_mask] += self.l_occ
            self.log_prob_map[free_mask] += self.l_free
            
    def get_map(self):
        """ Returns map with probabilites [0,1] """
        return 1.0 - 1./(1.+np.exp(self.log_prob_map))
    
    
    def getGridCoords(self, x : float, y : float) -> Tuple[int, int]:
        " if x and y are not in limits of the grid, returns (0, 0)"
        x_coords = int(x / self.grid_size)
        y_coords = int(y / self.grid_size)
        if x_coords < 0 or x_coords >= self.xsize or y_coords < 0 or y_coords >= self.ysize:
            return 0, 0
        
        return x_coords, y_coords
    
    
    def getRealPosition(self, x_coords : int, y_coords: int) -> Tuple[float, float]:
        if x_coords < 0 or x_coords >= self.xsize or y_coords < 0 or y_coords >= self.ysize:
            return 0, 0
        
        return x_coords * self.grid_size, y_coords * self.grid_size
