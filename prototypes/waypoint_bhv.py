""" 
    In this package we develop a waypoint behavior tool to tell the vehicle how to get somewhere, given a set of waypoints and obstacles
     - Make a grid of all the possible points
     - Include obstacles on the grid, expand them to account for the vehicle size
     - To impose an alignment constraint, we will create a secondary waypoint, 'behind' the actual target so the vehicle can lead in to the final waypoint, at the desired heading
     - Will need to map an obstacles in the continuous domain to the discrete domain, i.e. get a distance to a target, project it from the vehicles location, and then include this point 
        in the correct grid cell
"""

import numpy as np
from scipy.ndimage.filters import convolve
import matplotlib.pyplot as plt
from queue import PriorityQueue
import heapq

class Grid:
    def __init__(self, length_m, width_m, step_size_m):
        n_idcs_length = int(length_m//step_size_m)
        n_idcs_width = int(width_m//step_size_m)
        self.length_m = length_m
        self.width_m = width_m
        self.length_idcs = n_idcs_length
        self.width_idcs = n_idcs_width
        self.step_size = step_size_m
        self.grid = np.zeros((n_idcs_length,n_idcs_width), dtype=np.int32) 
        self.boundary = np.zeros((n_idcs_length,n_idcs_width), dtype=np.int32) 

        #apply the boundary along the top and bottom
        for i in range(0,n_idcs_width):
            self.boundary[0,i] = 1
            self.boundary[n_idcs_length-1,i] = 1
        #apply the boundary along the left and right
        for j in range(1,n_idcs_length-1):
            self.boundary[j,0] = 1
            self.boundary[j,n_idcs_width-1] = 1
        
        self.search_grid = np.zeros((n_idcs_length,n_idcs_width), dtype=np.int32)

    def get_idcs(self, x_c,y_c):
        """ 
            Map points in the continous domain to the discrete domain
            If we have a step size of 0.1, a width of 3 m, we'll have 30 steps
            At 1.5, we'll want around the 14th index
        """
        if (x_c <= 0 or x_c > self.width_m) or (y_c <= 0 or y_c > self.length_m):
            print(f"Requested point out of bounds! <{x_c}, {y_c}>")
            return False, False
        return int(x_c//self.step_size), int(y_c//self.step_size)
    
    def get_point(self, x_c, y_c):
        if (x_c <= 0 or x_c > self.width_m) or (y_c <= 0 or y_c > self.length_m):
            print(f"Requested point out of bounds! <{x_c}, {y_c}>")
            return False
        x_d, y_d = self.get_idcs(x_c, y_c)
        return self.grid(x_d, y_d)
    
    def get_coords(self, x_d, y_d):
        if (x_d <= 0 or x_d > self.width_idcs) or (y_d <= 0 or y_d > self.length_idcs):
            print(f"Requested point out of bounds! <{x_d}, {y_d}>")
            return False
        return x_d*self.step_size, y_d*self.step_size 

    def insert_obstacle(self, x_c, y_c): 
        """
            Insert an obstacle at some x,y point
        """
        if (x_c <= 0 or x_c > self.width_m) or (y_c <= 0 or y_c > self.length_m):
            print(f"Obstacle out of bounds! <{x_c}, {y_c}>")
            return False
        x_d, y_d = self.get_idcs(x_c, y_c)
        self.grid[y_d, x_d] = 1
        return True
    
    def update_grid(self, obst_r, wall_r):
        """ 
            Take the initial grid, and dilate all the obstacles by the radius of the vehicle
        """

        #Dilate the obstacles
        idcs_r = int(obst_r//self.step_size)
        kernel_size = idcs_r*2+1
        y,x = np.ogrid[-idcs_r:idcs_r+1, -idcs_r:idcs_r+1]
        mask = x**2+y**2 <= idcs_r**2
        kernel = np.zeros((kernel_size, kernel_size))
        kernel[mask] = 1
        self.search_grid = convolve(self.grid, kernel, mode='wrap')

        #Dilate the walls - option to be forgiving
        idcs_r = int(wall_r//self.step_size)
        kernel_size = idcs_r*2+1
        y,x = np.ogrid[-idcs_r:idcs_r+1, -idcs_r:idcs_r+1]
        mask = x**2+y**2 <= idcs_r**2
        kernel = np.zeros((kernel_size, kernel_size))
        kernel[mask] = 1

        boundary = convolve(self.boundary, kernel, mode='wrap')
        boundary = np.where(boundary > 0, 1, 0)
        self.search_grid+=boundary
        return True 
    
    def show_grid(self, initial_point=None, target=None, path=None):
        grid_for_plot = self.search_grid+self.grid #center point has a brighter intensity
        fig, ax = plt.subplots(figsize=(self.width_m*2, self.length_m*2))
        plt.imshow(grid_for_plot, cmap='viridis')
        plt.xlim([0, self.width_idcs])
        plt.ylim([0, self.length_idcs])
        n_ticks = 10

        if initial_point is not None:
            initial_x, initial_y = self.get_idcs(*initial_point)
            plt.plot(initial_x, initial_y, 'go', MarkerSize=8)

        if target is not None:
            target_x, target_y = self.get_idcs(*target)
            plt.plot(target_x, target_y, 'gx', MarkerSize=8)

        if path is not None:
            xs, ys = zip(*path)
            plt.plot(xs, ys, 'r-', linewidth=3) 

        x_ticks = np.linspace(0,self.width_idcs,n_ticks)
        y_ticks = np.linspace(0,self.length_idcs,n_ticks)

        x_tick_labels = [f"{tick:0.2f}" for tick in np.linspace(0,self.width_m, n_ticks)]
        y_tick_labels = [f"{tick:0.2f}" for tick in np.linspace(0,self.length_m, n_ticks)]

        plt.xticks(x_ticks, x_tick_labels)
        plt.yticks(y_ticks, y_tick_labels)

        plt.colorbar()
        plt.show()
        return True

class Waypoint:
    def __init__(self, point, approach_angle, lead_dist):
        self.point = point 
        self.approach_angle = approach_angle
        x_lead = point[0] - lead_dist*np.cos(approach_angle*np.pi/180) 
        y_lead = point[1] - lead_dist*np.sin(approach_angle*np.pi/180) 
        self.lead_point = [x_lead, y_lead]


class WaypointBHV:
    """ 
        Receive high level waypoints, compute secondary waypoints
    """
    def __init__(self, initial_position, goal: Waypoint, grid): 
        self.initial_position = initial_position
        self.goal = goal
        self.grid = grid 

    def get_path(self, lead=True):
        if lead:
            path = self.a_star_search(self.initial_position, self.goal.lead_point)
            print(f"Lead point {self.goal.lead_point}")
            if path == None:
                print("No path found!")
                return path, path
            else:
                goal_by_idx = self.grid.get_idcs(*self.goal.point)
                path.append([goal_by_idx[0], goal_by_idx[1]])
                return path, [grid.get_coords(*pair) for pair in path]
        else:
            path = self.a_star_search(self.initial_position, self.goal.point)
            if path == None:
                print("No path found!")
                return path, path
            else:
                return path, [grid.get_coords(*pair) for pair in path]

    def a_star_search(self, start_m, end_m):
        def heuristic(a, b):
            return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        start_x, start_y = grid.get_idcs(*start_m)
        end_x, end_y = grid.get_idcs(*end_m)
        start = tuple([start_x, start_y])
        end = tuple([end_x, end_y])

        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
        visited = set()
        priority_queue = [(0, start, [], 0)]

        while priority_queue:
            priority_queue.sort()
            _, current, path, cost = priority_queue.pop(0)

            if current == end:
                return path + [current]

            if current in visited:
                continue

            visited.add(current)

            for dx, dy in neighbors:
                x, y = current
                nx, ny = x + dx, y + dy

                if 0 <= nx < self.grid.search_grid.shape[1] and 0 <= ny < self.grid.search_grid.shape[0] and self.grid.search_grid[ny, nx] == 0:
                    priority_queue.append((cost + heuristic((nx, ny), end), (nx, ny), path + [current], cost + 1))
        return None

if __name__ == "__main__":
    arena_length = 2.5 #meters
    arena_width = 3.75 #meters
    step_size = 0.025 #meters
    vehicle_radius = 0.2 #meters
    boundary_buffer = 0.05 #meters
    initial_position = [3.6, 2.2]
    goal_pt = [3.6, 0.1]
    wpt = Waypoint(goal_pt, 0, 0.2)
    grid = Grid(arena_length, arena_width, step_size)
    grid.insert_obstacle(2,2)
    grid.insert_obstacle(1,2)
    for i in np.linspace(0.4, 3.2, 5):
        grid.insert_obstacle(i,1.5)
    grid.insert_obstacle(3.5,1)
    grid.update_grid(vehicle_radius, boundary_buffer)
    wpt_bhv = WaypointBHV(initial_position, wpt, grid)
    path_by_idx, path_by_cord = wpt_bhv.get_path(True)
    print(f"Path: {[f'<{x:0.2f},{y:0.2f}>' for x,y in path_by_cord]}")
    grid.show_grid(initial_position, wpt.point, path_by_idx)