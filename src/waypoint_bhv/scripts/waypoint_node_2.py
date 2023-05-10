#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math 
import numpy as np
from scipy.ndimage.filters import convolve
import matplotlib.pyplot as plt
import time 

class Grid:
    def __init__(self, length_m, width_m, padding_m, step_size_m):
        n_idcs_length = int((length_m+padding_m)//step_size_m)
        n_idcs_width = int((width_m+padding_m)//step_size_m)
        self.length_m = length_m+padding_m
        self.width_m = width_m+padding_m
        self.padding_m = padding_m
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
        if (x_c < -self.padding_m/2 or x_c > (self.width_m+self.padding_m/2)) or (y_c < -self.padding_m/2 or y_c > (self.length_m+self.padding_m/2)):
            print(f"Requested point out of bounds! <{x_c}, {y_c}>")
            return False, False
        return int((x_c)//self.step_size), int((y_c)//self.step_size)
    
    def get_point(self, x_c, y_c):
        if (x_c < -self.padding_m/2 or x_c > (self.width_m+self.padding_m/2)) or (y_c < -self.padding_m/2 or y_c > (self.length_m+self.padding_m/2)):
            print(f"Requested point out of bounds! <{x_c}, {y_c}>")
            return False
        x_d, y_d = self.get_idcs(x_c, y_c)
        return self.grid(x_d, y_d)
    
    def get_coords(self, x_d, y_d):
        if (x_d <= 0 or x_d > self.width_idcs) or (y_d <= 0 or y_d > self.length_idcs):
            print(f"Requested point out of bounds! <{x_d}, {y_d}>")
            return False, False
        return (x_d*self.step_size), (y_d*self.step_size )

    def insert_obstacle(self, x_c, y_c): 
        """
            Insert an obstacle at some x,y point
        """
        if (x_c < -self.padding_m/2 or x_c > (self.width_m+self.padding_m/2)) or (y_c < -self.padding_m/2 or y_c > (self.length_m+self.padding_m/2)):
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
            plt.text(initial_x,initial_y,f"Start {self.get_coords(initial_x, initial_y)}", style='italic')

        if target is not None:
            target_x, target_y = self.get_idcs(*target)
            plt.plot(target_x, target_y, 'gx', MarkerSize=8)
            plt.text(target_x, target_y,f"End {self.get_coords(target_x, target_y)}", style='italic')

        if path is not None:
            xs, ys = zip(*path)
            plt.plot(xs, ys, 'r-', linewidth=3) 

        x_ticks = np.linspace(0,self.width_idcs,n_ticks)
        y_ticks = np.linspace(0,self.length_idcs,n_ticks)

        x_tick_labels = [f"{tick:0.2f}" for tick in np.linspace(-self.padding_m/2,self.width_m+self.padding_m/2, n_ticks)]
        y_tick_labels = [f"{tick:0.2f}" for tick in np.linspace(-self.padding_m/2,self.length_m+self.padding_m/2, n_ticks)]

        plt.xticks(x_ticks, x_tick_labels)
        plt.yticks(y_ticks, y_tick_labels)

        plt.colorbar()
        plt.savefig(f"current_state_{time.perf_counter()}.png")
        return True

class WaypointBHV:
    """ 
        Receive high level waypoints, compute secondary waypoints
    """
    def __init__(self, goal, grid): 
        self.goal_xy = [goal[0], goal[1]]
        self.goal_angle = goal[2]
        self.end_flag = goal[4]
        self.grid = grid 

    def get_path(self, current_position):
        path = self.a_star_search(current_position, self.goal_xy)
        if path == None:
            print("No path found!")
            return path, path
        else:
            path_by_idx = [self.grid.get_coords(*pair) for pair in path]
            return path, path_by_idx


    def a_star_search(self, start_m, end_m):
        def heuristic(a, b):
            return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        start_x, start_y = self.grid.get_idcs(*start_m)
        end_x, end_y = self.grid.get_idcs(*end_m)
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
    
class WaypointController:
    def __init__(self):
        # Initialize the node
        rospy.init_node('waypoint_controller')

        # Set the parameters
        self.length_m = 2.5 #meters
        self.width_m = 3.75 #meters
        self.step_size_m = 0.05 #meters
        self.obst_r = 0.2 #meters
        self.wall_r = 0.00 #meters
        self.capture_radius = 0.2 #meters
        self.speed = 0.1 #meters/s
        
        # Initialize the grid and waypoint behaviors
        self.padding = 1 #meter
        self.grid = Grid(self.length_m, self.width_m, self.padding, self.step_size_m)
        self.grid.update_grid(self.obst_r, self.wall_r)
        self.last_heading = 0
        self.last_orientation = 0

        #waypoint as (x,y,end_heading,start_flag,end_flag)
        wpt1 = (2.4, 0.48, 0, "wpt_1", "wpt_2")
        wpt2 = (2.93, 0.48, 0, "wpt_2", "aligning_aed")
        wpt3 = (0.09, 2.32, 180, "wpt_3", "unpacking_aed")

        wptbhv_1 = WaypointBHV(wpt1, self.grid)
        wptbhv_2 = WaypointBHV(wpt2, self.grid)
        wptbhv_3 = WaypointBHV(wpt3, self.grid)

        self.waypoint_bhvs = {wpt1[3]:wptbhv_1,
                         wpt2[3]:wptbhv_2,
                         wpt3[3]:wptbhv_3}
        
        self.pose_init = False 

        self.current_position = [0,0]

        self.last_replan = 0

        # Create the publishers and subscribers
        self.pose_sub = rospy.Subscriber(
            '/uwb_pose', PoseStamped, self.pose_callback)
        
        self.obstacles_sub = rospy.Subscriber(
            '/mr/obstacles', String, self.obstacles_callback)
        
        self.system_state_sub = rospy.Subscriber(
            '/system_state', String, self.system_state_callback)
        
        self.system_state_pub = rospy.Publisher('/system_state', String, queue_size=10)
        
        self.ctrl_pub = rospy.Publisher('/to_fw/ctrl', String, queue_size=10)

        # Set the initial system state
        self.system_state = 'init'
        self.replan = True 
        self.grid.update_grid(self.obst_r, self.wall_r)
        self.last_ctrl_pub_t = 0
        self.new_state = False
        self.valid_keys = [key for key in self.waypoint_bhvs.keys()]

        # Run the node
        while not rospy.is_shutdown():
            #If we initialized our pose and know where to form plans from
            if self.pose_init:
                #Check to see if the system state is associated to one of our behaviors/waypoints
                if self.system_state in self.valid_keys:
                    if self.new_state or self.replan:
                        #If this is the first time we entered this state or we need to replan, we form a plan
                        self.bhv = self.waypoint_bhvs[self.system_state]
                        self.bhv.grid.update_grid(self.obst_r, self.wall_r)
                        self.cplan_by_idx, self.cplan_by_xy = self.bhv.get_path(self.current_position)
                        self.new_state = False
                        self.replan = False
                        self.last_replan = time.perf_counter()
                        self.grid.show_grid(self.current_position, self.bhv.goal_xy, self.cplan_by_idx)
                    else:
                        #If we are not in a new state, but we are in one of our states, and we do not need to replan
                        #We simply monitor the plan and pop off captured points - the obstacles topic will tell us if we need to replan
                        if self.cplan_by_xy != None and len(self.cplan_by_xy) > 0:
                            if self.dist(self.current_position, self.cplan_by_xy[0]) < self.capture_radius:
                                print("Captured a point!!!")
                                #If a point is in the capture radius, then 
                                self.cplan_by_xy.pop(0)
                                self.cplan_by_idx.pop(0)  
                                #post a control message every 0.5 seconds this is for smoothing
                            if time.perf_counter() > self.last_ctrl_pub_t+0.2:
                                ctrl_msg = self.get_ctrl_string(self.current_position, self.cplan_by_xy)
                                self.publish_ctrl_msg(ctrl_msg)
                                self.last_ctrl_pub_t = time.perf_counter()
                        else:
                            #If no plan was found, just go straight to the goal - (for bugs?)
                            if time.perf_counter() > self.last_ctrl_pub_t+0.2:
                                ctrl_msg = self.get_ctrl_string(self.current_position, [self.bhv.goal_xy])
                                self.publish_ctrl_msg(ctrl_msg)
                                self.last_ctrl_pub_t = time.perf_counter()
                            print("No plan found")

                    #If we are at the waypoints current target, we raise its end flag and change the system state
                    #else we propose a new heading and wait
                    if self.dist(self.current_position, self.bhv.goal_xy) < self.capture_radius:
                        print(f"Captured goal {self.bhv.goal_xy}")
                        self.system_state = self.bhv.end_flag
                        self.new_state = True
                        self.system_state_pub.publish(self.bhv.end_flag)
                        ctrl_msg = f"$CTRL,0,{self.bhv.goal_angle},0,*"
                        self.publish_ctrl_msg(ctrl_msg)
                        #wait for two seconds before we transition to a new internal state
                        #if it isn't an internal state it doesn't effect anything
                        time.sleep(2)
                    if time.perf_counter() > self.last_replan+2:
                        self.replan = True
            else:
                print("Not localized")

    def dist(self, pos1, pos2):
        #distance between two points fo
        return math.sqrt((pos1[0]-pos2[0])**2+(pos1[1]-pos2[1])**2)
    
    def get_ctrl_string(self, current_position, target_set):
        #For a current position and the proposed path, construct a control message
        cx,cy = current_position
        ctrl_msg = ""
        if len(target_set) > 6:
            ortn = (np.mean([math.atan2((target_set[idx][1]-cy),(target_set[idx][0]-cx)) for idx in range(0,7)])*180/np.pi)%360
            ctrl_msg = f"$CTRL,0,{ortn},{self.speed},*"
        else:
            x,y = target_set[0]
            ortn = (math.atan2((y-cy),(x-cx))*180/np.pi)%360
            ctrl_msg = f"$CTRL,0,{ortn},{self.speed},*"
        return ctrl_msg

    def pose_callback(self, msg):
        # Check if the current position is within the capture radius of the next waypoint
        self.current_position = [msg.pose.position.x, msg.pose.position.y]
        self.pose_init = True
        print(f"Updating point: {self.current_position}")
        

    def obstacles_callback(self, msg):
        # Parse the obstacle coordinates from the message
        obstacles = []
        obstacles_str = msg.data.split(';')
        for obstacle_str in obstacles_str:
            if obstacle_str:
                obstacle = obstacle_str.split(',')
                obstacles.append([float(obstacle[0]), float(obstacle[1])])
        # Update the grid and replan the path
        for obstacle in obstacles:
            self.grid.insert_obstacle(obstacle[0], obstacle[1])
        self.replan = True

    def system_state_callback(self, msg):
        # Update the system state
        self.system_state = msg.data
        self.new_state = True

    def publish_ctrl_msg(self, ctrl_msg):
        # Publish the control message
        print(f"- {ctrl_msg}")
        self.ctrl_pub.publish(ctrl_msg)


if __name__ == '__main__':
    try:
        controller = WaypointController()
    except rospy.ROSInterruptException:
        pass
