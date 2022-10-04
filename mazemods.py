#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Yulin Li
@email: yline@connect.ust.hk
"""

# This script provide useful member functions when interacting with the maze
# import this class in your search.py

from argparse import Action
from cgi import print_directory
from fileinput import close
from gettext import find
from importlib.resources import path
from re import X
import numpy as np
import matplotlib.pyplot as plt
import MazeObs
import operator
from matplotlib import colors

import math

class Grid:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.f = 0
        self.g = 0
        self.h = 0
        self.parent = None

    def init_grid(self, parent, end, heuristic):
        self.parent = parent
        if parent is not None:
            self.g = parent.g + 1
        else:
            self.g = 1
        
        self.h = heuristic([self.x, self.y], [end.x, end.y])    
        #self.h = abs(self.x - end.x) + abs(self.y - end.y)
        self.f = self.g + self.h


class Maze:
    n_maze = [22, 36, 37]
    m_maze = [10, 18, 37]

    def __init__(self, mazeType, xI, xG):
        if mazeType == "big":
            self.n = self.n_maze[2]
            self.m = self.m_maze[2]
            self.O = MazeObs.O_maze[2][0]
        elif mazeType == "medium":
            self.n = self.n_maze[1]
            self.m = self.m_maze[1]
            self.O = MazeObs.O_maze[1][0]
        else:
            self.n = self.n_maze[0]
            self.m = self.m_maze[0]
            self.O = MazeObs.O_maze[0][0]
        
        self.xI = xI
        self.xG = xG
            
        self.start = Grid(xI[0],xI[1])
        #self.x = self.xI
        self.end = Grid(xG[0],xG[1])
        #self.showMaze()
        
        self.path = []
        #self.action_space = [(-1, 0),(1, 0),(0, -1),(0, 1),(-1,1),(1,1),(1,-1),(-1,-1)]
        self.action_space = [(-1, 0),(1, 0),(0, -1),(0, 1)]
        
        
    def makeMaze(self):
        # Make corresponding grid values of the maze
        # Initialize to lists of 1. for RGB color index = white
        self.gridvals = [[[1. for i in range(3)] for col in range(self.n)] for row in range(self.m)]
        #print(self.gridvals)
        # Iterate through each obstacle
        for l in range(len(self.O)):
            # Find boundaries of current obstacle
            west, east = [self.O[l][0], self.O[l][1]]
            south, north = [self.O[l][2], self.O[l][3]]
            # Iterate through each cell of obstacle (clunky, but works)
            for i in range(west,east+1):
                for j in range(south,north+1):
                    self.gridvals[j][i] = [0.,0.,0.] # Change entry to RGB black

    def showMaze(self):
        self.makeMaze()
        fig, ax = plt.subplots()  # make a figure + axes
        ax.imshow(self.gridvals)  # Plot it
        ax.invert_yaxis()  # Needed so that bottom left is (0,0)
        #plt.scatter(self.xI[0], self.xI[1], c="red")

    def find_min_f(self, open_list=[]):
        temp_x = open_list[0]
        for x in open_list:
            if x.f < temp_x.f:
                temp_x = x
        return temp_x
    
    def gen_action(self, heuristic):
        open_list = []
        close_list = []
        open_list.append(self.start)
        planning_step =0 
        
        while len(open_list) > 0:
        
            curr_grid = self.find_min_f(open_list)
            open_list.remove(curr_grid)
            close_list.append(curr_grid)
            
            for action in self.action_space:
                
                curr_pos = [curr_grid.x, curr_grid.y]
                grid_new_pos = [curr_grid.x+action[0], curr_grid.y+action[1]]
                collision = self.collisionCheck(curr_pos,action)
                
                for close_grid in close_list:
                    
                    #print(f"index is {index}, closed agent is {closed_set[index]} and x new is {x_new}")
                    close_grid_pos = [close_grid.x, close_grid.y]
                    if grid_new_pos == close_grid_pos:
                        closed = True
                        break
                    else:
                        closed = False
           
                if (collision):
                    #print("potential action causes collision")
                    pass
                elif (closed):
                    pass
                    #print("stuck in the loop")
                else:
                    
                    planning_step += 1
                    grid_new = Grid(grid_new_pos[0], grid_new_pos[1])
                    if grid_new not in open_list:
                        
                        grid_new.init_grid(curr_grid, self.end, heuristic)
                        print(grid_new.h)
                        open_list.append(grid_new)
                    for grid in open_list:
                        if (grid.x == self.end.x) and (grid.y == self.end.y):
                            return grid, planning_step
                        
        return None, planning_step
                    
    # search on the graph and extract the control action sequences
    # this is where actually your A* search algorithm is implemented
    # return the actions list
    # your code here:
    ''''''
    def closedCheck(self, state, closed_set):
        
        if state in closed_set:
            return True
        else:
            return False
    
    def collisionCheck(self,x,u, inputCheck = True):
        # Checks for collisions given position x, control u, obstacle list O
        #print(u)
        if (inputCheck):
            if u != (-1, 0) and u != (1, 0) and u != (0, -1) and u != (0, 1) : #and u !=(-1,1) and u!=(1,1) and u!=(1,-1) and u!=(-1,-1):
                print('collision_check error: Invalid input u!')
                return
        nextx = [x[i] + u[i] for i in range(len(x))]
        for l in range(len(self.O)):
            # Find boundaries of current obstacle
            west, east = [self.O[l][0], self.O[l][1]]
            south, north = [self.O[l][2], self.O[l][3]]
            # Check if nextx is contained in obstacle boundaries
            if west <= nextx[0] <= east and south <= nextx[1] <= north:
                return True
        # If we iterate through whole list and don't trigger the "if", then no collisions
        return False

    # Makes a piece of data with obstacles stored as black RGB values,
    # free space stored as white RGB values, and path stored as increasing hue of
    # yellow RGB values

    def makePath(self, xI, xG, path):
        # Obtain the grid populated with obstacles and free space RGB values first
        self.makeMaze()
        self.gridpath = self.gridvals.copy()
        L = len(path)
        # Iterate through the path to plot as increasing shades of yellow
        for l in range(L-1):
            self.gridpath[path[l][1]][path[l][0]] = [1., 1., 1-l/(L-1)] # white-->yellow
        self.gridpath[xI[1]][xI[0]] = [0., 0., 1.] # Initial node (plotted as blue)
        self.gridpath[xG[1]][xG[0]] = [0., 1., 0.] # Goal node (plotted as green)



    def showPath(self, xI, xG, path):
        # Plots the path
        self.makePath(xI, xG, path)
        fig, ax = plt.subplots(1, 1)  # make a figure + axes
        ax.imshow(self.gridpath)  # Plot it
        ax.invert_yaxis()  # Needed so that bottom left is (0,0)

    '''
    def getPathFromActions(self, xI):
    # extract path from start
    #return the paths node list
    # your code here:
        path = []
    '''
    
    def getCostOfActions(self, xI, actions):
        # evaluate along your path to get the cost
        # If any collisions, cost is 999999, else cost is one for each action
        # your code here:
        collision = self.collisionCheck(xI,actions, inputCheck=False)
        if collision == True:
            cost = 999999
        else:
            cost = math.sqrt(((actions[0])**2)+((actions[1])**2) )
        
        return cost
            

    
    
    
    
    
    
