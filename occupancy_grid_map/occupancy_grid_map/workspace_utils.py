
import numpy as np
import pandas as pd

class Workspace:

    def __init__(self):

        self.file_path_ws = '/home/group5/dd2419_ws/outputs/workspace_exploration.csv'
        self.file_path_map = '/home/group5/dd2419_ws/outputs/normal_map2.csv'
        ws = pd.read_csv(self.file_path_ws)
        mp = pd.read_csv(self.file_path_map, header=None)

        self.workspace = ws.to_numpy().transpose()
        self.map = mp[[1, 2]].to_numpy().transpose()
        self.objects_boxes = mp[[0,1,2]].to_numpy().transpose()

        self.map_min_x = np.min(self.workspace[0])
        self.map_min_y = np.min(self.workspace[1])
        self.map_max_x = np.max(self.workspace[0])
        self.map_max_y = np.max(self.workspace[1])

        self.map_xlength = self.map_max_x - self.map_min_x
        self.map_ylength = self.map_max_y - self.map_min_y
        self.resolution = 5

        self.grid_xlength = int(self.map_xlength/self.resolution)
        self.grid_ylength = int(self.map_ylength/self.resolution)
        self.unknown = -1

        self.grid = np.full((self.grid_ylength, self.grid_xlength), self.unknown, dtype=np.int16)
        self.phase = 'exploration' #### 'collection' or 'exploration'
        
    def create_grid(self):
 
        grid_xlength = int(self.map_xlength/self.resolution)
        grid_ylength = int(self.map_ylength/self.resolution)
        unknown = -1
        grid = np.full((grid_ylength, grid_xlength), unknown, dtype=np.int16)

        return grid
        
    def convert_map_to_grid(self, x, y):
        #Converts from map to grid expecting x and y in centimeters

        grid_x = np.floor((x - self.map_min_x)/self.resolution).astype(int)
        grid_y = np.floor((y - self.map_min_y)/self.resolution).astype(int)

        # grid_x = np.floor((x + self.map_xlength/2)/self.resolution)
        # grid_y = np.floor((y + self.map_ylength/2)/self.resolution)

        return grid_x, grid_y
    
    def convert_grid_to_map(self, grid_x, grid_y):
        #Converts from grid to map giving back x and y in centimeters

        x = grid_x*self.resolution + self.map_min_x
        y = grid_y*self.resolution + self.map_min_y

        # x = (grid_x*self.resolution - self.map_xlength/2)
        # y = (grid_y*self.resolution - self.map_ylength/2)

        return x, y
    
