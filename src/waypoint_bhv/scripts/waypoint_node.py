#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import math 


import numpy as np
from scipy.ndimage.filters import convolve
import matplotlib.pyplot as plt

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
            print(f"IP: {self.initial_position} -> {self.goal.lead_point}")
            path = self.a_star_search(self.initial_position, self.goal.lead_point)
            print(f"Lead point {self.goal.lead_point}")
            if path == None:
                print("No path found!")
                return path, path
            else:
                goal_by_idx = self.grid.get_idcs(*self.goal.point)
                path.append([goal_by_idx[0], goal_by_idx[1]])
                return path, [self.grid.get_coords(*pair) for pair in path]
        else:
            path = self.a_star_search(self.initial_position, self.goal.point)
            if path == None:
                print("No path found!")
                return path, path
            else:
                return path, [self.grid.get_coords(*pair) for pair in path]
            
    def simplify_path(path_by_idx, path_by_wpt):
        crosses = [[[0,1],[0,-1]], [[1,0],[-1,0]],[[1,1],[-1,-1]]]

        for elem in path_by_idx:
            pass

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
        self.length_m = 2.5
        self.width_m = 3.75
        self.step_size_m = 0.05
        self.obst_r = 0.2
        self.wall_r = 0.00
        self.capture_radius = 0.05
        self.speed = 0.1
        self.compass_setup_offset = 0
        # Initialize the grid and waypoint behaviors
        self.padding = 1
        self.grid = Grid(self.length_m, self.width_m, self.padding, self.step_size_m)
        self.grid.update_grid(self.obst_r, self.wall_r)
        initial_1 = [3.2, 1.4] #arbitrary
        self.pos_1_init = False
        goal_1 = [2.94, 0.48]
        goal_1_lead_angle = 0
        goal_1_lead_length = 0.75
        self.last_heading = 0
        self.last_orientation = 0
        initial_2 = [3, 0.1] #arbitrary
        self.pos_2_init = False
        goal_2 = [0.62, 2.32]
        goal_2_lead_angle = 180
        goal_2_lead_length = 0.5
        self.wpt1 = WaypointBHV(initial_1, Waypoint(goal_1, goal_1_lead_angle, goal_1_lead_length), self.grid)
        self.wpt2 = WaypointBHV(initial_2, Waypoint(goal_2, goal_2_lead_angle, goal_2_lead_length), self.grid)
        self.plan1_by_idx, self.plan_1 = self.wpt1.get_path()
        self.plan2_by_idx, self.plan_2 = self.wpt2.get_path()
        self.current_position = initial_1
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
        self.system_state = ''
        self.grid.update_grid(self.obst_r, self.wall_r)

        # Run the node
        rospy.spin()

    def pose_callback(self, msg):
        # Check if the current position is within the capture radius of the next waypoint
        self.current_position = [msg.pose.position.x, msg.pose.position.y]
        print(f"Updating point: {self.current_position}")
        #self.wpt1.grid.show_grid(initial_point=self.current_position,target=self.wpt1.goal.point,path=self.plan1_by_idx)
        #If we are in the wpt 1 state
        if self.system_state == 'wpt1':
            self.wpt1.initial_position = self.current_position
            self.grid.update_grid(self.obst_r, self.wall_r)
            self.plan1_by_idx, self.plan_1 = self.wpt1.get_path(True)
            #Observe the total distance to the actual target
            distance_to_target = (
                    (self.current_position[0] - self.wpt1.goal.point[0])**2 + (self.current_position[1] - self.wpt1.goal.point[1])**2)**0.5
            #If the plan is not nonetype
            if self.plan_1:
                #remove all the points within the capture radius
                print(f"Plan: {self.plan_1}")
                while True:
                    print("Popping elements")
                    if self.plan_1:
                        next_waypoint = self.plan_1[0]
                        distance_to_waypoint = (
                            (self.current_position[0] - next_waypoint[0])**2 + (self.current_position[1] - next_waypoint[1])**2)**0.5
                        if distance_to_waypoint < self.capture_radius:
                            self.plan_1.pop(0)
                        else:
                            break
                    else:
                        print("Broke out")
                        break
                #If we still have a plan
                if self.plan_1:
                    #publish the next waypoint
                    next_waypoint = self.plan_1[0]
                    self.publish_ctrl_msg(next_waypoint)
                elif distance_to_target < self.capture_radius:
                    #If we are near the final target
                    self.system_state = 'aligning'
                    self.system_state_pub.publish('aligning')
                    msg = "$CTRL,0,{self.wpt1.goal.approach_angle},0,*"
                    self.ctrl_pub(msg)

                else:
                    print("No plan available!!")
            else:
                print("The plan is nonetype on entry")
                    
        elif self.system_state == 'wpt2':
            self.wpt2.initial_position = self.current_position
            self.grid.update_grid(self.obst_r, self.wall_r)
            self.plan2_by_idx, self.plan_2 = self.wpt2.get_path(True)
            #Observe the total distance to the actual target
            distance_to_target = (
                    (self.current_position[0] - self.wpt2.goal.point[0])**2 + (self.current_position[1] - self.wpt1.goal.point[1])**2)**0.5
            #If the plan is not nonetype
            if self.plan_2:
                #remove all the points within the capture radius
                print(f"Plan: {self.plan_2}")
                while True:
                    if self.plan_1:
                        next_waypoint = self.plan_2[0]
                        distance_to_waypoint = (
                            (self.current_position[0] - next_waypoint[0])**2 + (self.current_position[1] - next_waypoint[1])**2)**0.5
                        if distance_to_waypoint < self.capture_radius:
                            self.plan_2.pop(0)
                        else:
                            break
                    else:
                        break
                #If we still have a plan
                if self.plan_2:
                    #publish the next waypoint
                    next_waypoint = self.plan_2[0]
                    self.publish_ctrl_msg(next_waypoint)
                elif distance_to_target < self.capture_radius:
                    #If we are near the final target
                    self.system_state = 'unpacking_aed'
                    self.system_state_pub.publish('unpacking_aed')
                    msg = "$CTRL,0,{self.wpt2.goal.approach_angle},0,*"
                    self.ctrl_pub(msg)
                else:
                    print("No plan available!!")
            else:
                print("The plan is nonetype on entry")

    def obstacles_callback(self, msg):
        # Parse the obstacle coordinates from the message
        print("In obstacle callback")

        obstacles = []
        obstacles_str = msg.data.split(';')
        for obstacle_str in obstacles_str:
            if obstacle_str:
                obstacle = obstacle_str.split(',')
                obstacles.append([float(obstacle[0]), float(obstacle[1])])

        # Update the grid and replan the path
        for obstacle in obstacles:
            self.grid.insert_obstacle(obstacle[0], obstacle[1])
        self.grid.update_grid(self.obst_r, self.wall_r)
        if self.system_state == 'wpt1':
            print("In obstacle callback 2")
            self.plan1_by_idx, self.plan_1
            if self.plan_1:
                next_waypoint = self.plan_1[0]
                self.publish_ctrl_msg(next_waypoint)
        elif self.system_state == 'wpt2':
            _, self.plan_2 = self.wpt2.get_path()
            if self.plan_2:
                next_waypoint = self.plan_2[0]
                self.publish_ctrl_msg(next_waypoint)

    def system_state_callback(self, msg):
        # Update the system state
        self.system_state = msg.data
        if self.system_state == 'wpt1':
            print("In system state callback")
            self.plan1_by_idx, self.plan_1
            if self.plan_1:
                next_waypoint = self.plan_1[0]
                self.publish_ctrl_msg(next_waypoint)
        elif self.system_state == 'wpt2':
            _, self.plan_2 = self.wpt2.get_path()
            if self.plan_2:
                next_waypoint = self.plan_2[0]
                self.publish_ctrl_msg(next_waypoint)

    def publish_ctrl_msg(self, next_waypoint):
        # Publish the control message
        print(f"Publishing control message")
        hdg = 0
        vel = self.speed
        x, y = next_waypoint
        cx, cy = self.current_position
        print(f"current wpt: {cx}x{cy}")
        print(f"next wpt: {x}x{y}")
        ortn = (math.atan2((y-cy),(x-cx))*180/math.pi)%360
        self.last_orientation = ortn
        print(f"Cartesian angle: {ortn}")
        ctrl_msg = f"$CTRL, {hdg}, {ortn}, {vel},*"
        print(f"Publishing: {ctrl_msg}")
        self.ctrl_pub.publish(ctrl_msg)


if __name__ == '__main__':
    try:
        controller = WaypointController()
    except rospy.ROSInterruptException:
        pass
