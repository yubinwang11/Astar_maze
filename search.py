#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Yulin Li
@email: yline@connect.ust.hk
"""

# Please do not distribute or publish solutions to this
# exercise. You are free to use these problems for educational purposes.

from multiprocessing import managers
from os import path
from unittest import result
import numpy as np
import matplotlib.pyplot as plt
from mazemods import Grid, Maze
import cmath

import math
import time


class AStarSearch:
# For testing, choose whatever aviable start and goal nodes.
    #self.xI = (34, 1)
    #self.xG = (1, 15)

    # A heuristic function estimates the cost from the current state to the nearest goal. When use this, the A* algorithm
    # degrade to Dijk algorithm
    def __init__(self, heuristic, maze):

        self.heuristic = heuristic
        self.maze = maze
        
        #self.xI = (34, 1)
        #self.xG = (1, 15)

    def nullHeuristic(state,goal):

        return 0

    # Manhattan heuristic function

    def manhattanHeuristic(state, goal):
        # your code here:
        manhattan_heu = abs(abs(goal[0]-state[0]) + abs(goal[1]-state[1]))
        
        return manhattan_heu

    # Euclidean heuristic function

    def euclideanHeuristic(state, goal):
        # your code here:
        euclidean_heu = abs(math.sqrt(((goal[0]-state[0])**2)+((goal[1]-state[1])**2) ))
        
        return euclidean_heu
    
    # A* algorithm wrapper
    def aStarSearch(self, xI, xG):
        
        result_grid, planning_step = maze_.gen_action(AStarSearch.euclideanHeuristic)
        path = []
        action_seq = []
        while result_grid is not None:
            #path.append([result_grid.x, result_grid.y])
            path.insert(0,[result_grid.x, result_grid.y])
            result_grid = result_grid.parent
            
        print(path)
        for i in range(len(path)-1):
            action = [path[i+1][0] - path[i][0], path[i+1][1] - path[i][1]]
            action_seq.append(action)
        
        total_cost = 0
        curr_x = list(xI)
        for action in action_seq:
            #print(action)
            cost = maze_.getCostOfActions(curr_x, action) 
            total_cost += cost
            curr_x[0] += action[0]
            curr_x[1] += action[1]
        
        print(f"the total cost is {total_cost}, total planning step is {planning_step}")
        maze_.showPath(xI, xG, path)
        #maze_.makePath(xI, xG, path)
            
        
        # your code here:
    

if __name__ == '__main__':
    start_time = time.time()
    # run the first line alone to see the maze environment
    xI = (35, 1)
    xG = (1,35)
    
    maze_ = Maze("big", xI, xG)
    
    # your code here
    search_ = AStarSearch(AStarSearch.manhattanHeuristic, maze_) 
    search_.aStarSearch(xI, xG)
    plt.savefig('Astar_big',dpi=600,format='eps')
    #plt.show()
    end_time = time.time()
    computation_time = end_time - start_time
    print(f"total time is {computation_time}")
    
    
