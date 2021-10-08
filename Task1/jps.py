from scipy.spatial import KDTree
import numpy as np
import random
import math
import time


class Node(object):
    def __init__(self, i, j, g, h, parent, direction):
        self.i = i
        self.j = j
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
        self.direction = direction


class Grid(object):
    def __init__(self, i, j):
        self.i = i
        self.j = j


class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y




class GridMap(object):
    def __init__(self, grid_size):
        self.grid_size = grid_size
        self.min_x = -4500
        self.max_x = 4500
        self.min_y = -3000
        self.max_y = 3000
        self.map_top_right = Point(4500,3000)
        self.map_buttom_left = Point(-4500, -3000)
        self.row = (self.max_y-self.min_y)//grid_size
        self.col = (self.max_x-self.min_y)//grid_size
        self.map = [[0 for j in range(self.col)] for i in range(self.row)]
        self.obstacle_list = []

    def transfer2MapCoor(self, x, y):
        i = (x-self.min_x)//self.grid_size
        j = (y-self.min_y)//self.grid_size
        return Grid(i, j)

    def loadObstacles(self, vision):
        for blue_robot in vision.blue_robot:
            if blue_robot.visible and blue_robot.id > 0:
                self.obstacle_list.append(Point(blue_robot.x,blue_robot.y))
        for yellow_robot in vision.yellow_robot:
            self.obstacle_list.append((Point(yellow_robot.x,yellow_robot.y)))

#0 means free, 1 means occupied
    def setOccupiedGrid(self):
        for obstacle in self.obstacle_list:
            obstacle_buttom_left_map_coor = self.transfer2MapCoor(obstacle.x-self.grid_size, obstacle.y-self.grid_size)
            obstacle_top_right_map_coor = self.transfer2MapCoor(obstacle.x+self.grid_size, obstacle.y+self.grid_size)
            for i in range(obstacle_buttom_left_map_coor.i, obstacle_top_right_map_coor.i+1):
                for j in range(obstacle_buttom_left_map_coor.j, obstacle_top_right_map_coor.j+1):
                    self.map[i][j] = 1

class Jps(object):
    def __init__(self, start_x, start_y, des_x, des_y, grid_size):
        self.grid_map = GridMap(grid_size)
        self.start_grid = self.grid_map.transfer2MapCoor(start_x, start_y)
        self.des_grid = self.grid_map.transfer2MapCoor(des_x, des_y)
        self.open_list = []
        self.close_list = []
        self.path = []

    def calH(self, current_grid):
        return math.sqrt((current_grid.i-self.des_grid.i)**2+(current_grid.j-self.des_grid.j)**2)

    def calG(self, current_grid, last_grid):
        return math.sqrt((current_grid.i-last_grid.i)**2+(current_grid.j-last_grid.j)**2)

    def findMinFInOpenList(self):
        min_node = self.open_list[0]
        for node in self.open_list:
            if min_node.f > node.f:
                min_node = node
        return min_node

    def jpsSearcher(self):
        start_node = Node(self.start_grid.i, self.start_grid.j, 0, self.calH(self.start_grid), -1. -1)
        self.open_list.append(start_node)

        while True:
            node = self.findMinFInOpenList()
            self.close_list.append(node)
            if(self.calH(node)<=1):
                break
            del self.open_list[self.open_list.index(node)]






























