
import numpy as np
import pandas as pd

class Workspace:

    def __init__(self):

        self.file_path = '/home/robot/group5_ws/src/workspace_exploration.csv'
        ws = pd.read_csv(self.file_path)

        self.workspace = ws[['x,', 'y']].to_numpy()

        self.map_xlength = np.max(self.workspace[0])*2
        self.map_ylength = np.max(self.workspace[1])*2
        self.resolution = 3
        self.grid_xlength = int(self.map_xlength/self.resolution)
        self.grid_ylength = int(self.map_ylength/self.resolution)
        self.unknown = -1
        self.grid = np.full((self.grid_ylength, self.grid_xlength), self.unknown, dtype=np.int16)
        
    def create_grid(self):
 
        grid_xlength = int(self.map_xlength/self.resolution)
        grid_ylength = int(self.map_ylength/self.resolution)
        unknown = -1
        grid = np.full((grid_ylength, grid_xlength), unknown, dtype=np.int16)

        return grid
        
    def convert_map_to_grid(self, x, y):
        #Converts from map to grid expecting x and y in centimeters

        grid_x = int(np.floor((x + self.map_xlength/2)/self.resolution))
        grid_y = int(np.floor((y + self.map_ylength/2)/self.resolution))

        return grid_x, grid_y
    
    def convert_grid_to_map(self, grid_x, grid_y):
        #Converts from grid to map giving back x and y in centimeters

        x = (grid_x*self.resolution - self.map_xlength/2)
        y = (grid_y*self.resolution - self.map_ylength/2)

        return x, y
    
