import math
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs, Debug_Msg, Debug_Arc
import time


class Node(object):
    def __init__(self, i, j, g, h, parent, direction):
        self.i = i
        self.j = j
        self.grid = Grid(i, j)
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
        self.robot_size = 100
        self.map_top_right = Point(4500, 3000)
        self.map_buttom_left = Point(-4500, -3000)
        self.row = (self.max_y - self.min_y) // grid_size
        self.col = (self.max_x - self.min_x) // grid_size
        self.map = [[0 for j in range(0, self.col)] for i in range(0, self.row)]
        self.obstacle_list = []
        self.obstacle_x = []
        self.obstacle_y = []

    def transfer2MapCoor(self, x, y):
        i = (x - self.min_x) // self.grid_size
        j = (y - self.min_y) // self.grid_size
        return Grid(i, j)

    # add obstacles in to list
    def loadObstacles(self, vision):
        for blue_robot in vision.blue_robot:
            if blue_robot.visible and blue_robot.id > 0:
                self.obstacle_list.append(Point(blue_robot.x, blue_robot.y))
        for yellow_robot in vision.yellow_robot:
            self.obstacle_list.append((Point(yellow_robot.x, yellow_robot.y)))

    # 0 means free, 1 means occupied
    def setOccupiedGrid(self):
        debugger = Debugger()

        for obstacle in self.obstacle_list:
            obstacle_buttom_left_map_coor = self.transfer2MapCoor(obstacle.x - 3 * self.robot_size,
                                                                  obstacle.y - 3 * self.robot_size)
            obstacle_top_right_map_coor = self.transfer2MapCoor(obstacle.x + 3 * self.robot_size,
                                                                obstacle.y + 3 * self.robot_size)
            for i in range((int)(obstacle_buttom_left_map_coor.i), (int)(obstacle_top_right_map_coor.i + 1)):
                for j in range((int)(obstacle_buttom_left_map_coor.j), (int)(obstacle_top_right_map_coor.j + 1)):
                    if i >= 0 and i < self.col and j >= 0 and j < self.row:
                        self.map[j][i] = 1
                        self.obstacle_x.append(i*self.grid_size+self.min_x+self.grid_size//2)
                        self.obstacle_y.append(j*self.grid_size+self.min_y+self.grid_size//2)

class Jps(object):
    def __init__(self, start_x, start_y, des_x, des_y, grid_size):
        self.grid_map = GridMap(grid_size)
        self.start_grid = self.grid_map.transfer2MapCoor(int(start_x), int(start_y))
        self.des_grid = self.grid_map.transfer2MapCoor(int(des_x), int(des_y))
        self.start_node = Node(self.start_grid.i, self.start_grid.j, 0, 999999, -1, Grid(-1, -1))
        self.neighbors = []
        self.open_list = []
        self.close_list = []
        self.path_x = []
        self.path_y = []

    # cal heuristic function
    def calH(self, current_node):
        return math.sqrt((current_node.i - self.des_grid.i) ** 2 + (current_node.j - self.des_grid.j) ** 2)

    # cal G value
    def calG(self, current_grid, last_grid):
        return math.sqrt((current_grid.i - last_grid.i) ** 2 + (current_grid.j - last_grid.j) ** 2)

    def findMinFInOpenList(self):
        min_node = self.open_list[0]
        for node in self.open_list:
            if min_node.f > node.f:
                min_node = node
        return min_node

    def calDirection(self, current_grid, last_grid):
        return Grid(current_grid.i - last_grid.i, current_grid.j - last_grid.j)

    def isOccupied(self, grid):
        if grid.i < 0 or grid.i >= self.grid_map.col or grid.j < 0 or grid.j >= self.grid_map.row:
            return True
        if self.grid_map.map[grid.j][grid.i] == 1:
            return True
        else:
            return False

    def isForcedNeighbor(self, neighbor, current_node):
        # diagonal condition
        if current_node.i < 0 or current_node.i >= self.grid_map.col or current_node.j < 0 or current_node.j >= self.grid_map.row:
            return False
        if neighbor.i < 0 or neighbor.i >= self.grid_map.col or neighbor.j < 0 or neighbor.j >= self.grid_map.row:
            return False
        direction = current_node.direction
        if direction.i != 0 and direction.j != 0:
            if self.grid_map.map[(neighbor.j + current_node.j - direction.j) // 2][
                (neighbor.i + current_node.i - direction.i) // 2] == 1:
                    if self.grid_map.map[neighbor.j][neighbor.i] == 0:
                        if self.grid_map.map[neighbor.j][neighbor.i] != 9:
                            self.grid_map.map[current_node.j][current_node.i] = 9
                            return True
            return False
        # vertical condition
        if direction.i == 0:
            if neighbor.i >= 0 and neighbor.i < self.grid_map.row:
                if self.grid_map.map[current_node.j][neighbor.i] == 1:
                    if self.grid_map.map[neighbor.j][neighbor.i] == 0:
                        if self.grid_map.map[neighbor.j][neighbor.i] != 9:
                            self.grid_map.map[current_node.j][current_node.i] = 9
                            return True
            return False
        # horizontal condition
        if direction.j == 0:
            if neighbor.j >= 0 and neighbor.j < self.grid_map.row:
                if self.grid_map.map[neighbor.j][current_node.i] == 1:
                    if self.grid_map.map[neighbor.j][neighbor.i] == 0:
                        if self.grid_map.map[neighbor.j][neighbor.i] != 9:
                            self.grid_map.map[current_node.j][current_node.i] = 9
                            return True
            return False

    def getNeighbors(self, current_node):
        self.neighbors = []
        if current_node.i == self.start_grid.i and current_node.j == self.start_grid.j:
            self.neighbors.append(Node(current_node.i - 1, current_node.j,
                                       self.calG(Grid(current_node.i - 1, current_node.j), self.start_grid), 0,
                                       self.start_node,
                                       self.calDirection(Grid(current_node.i - 1, current_node.j), self.start_grid)))
            self.neighbors.append(Node(current_node.i + 1, current_node.j,
                                       self.calG(Grid(current_node.i + 1, current_node.j), self.start_grid), 0,
                                       self.start_node,
                                       self.calDirection(Grid(current_node.i + 1, current_node.j), self.start_grid)))
            self.neighbors.append(Node(current_node.i, current_node.j - 1,
                                       self.calG(Grid(current_node.i, current_node.j - 1), self.start_grid), 0,
                                       self.start_node,
                                       self.calDirection(Grid(current_node.i, current_node.j - 1), self.start_grid)))
            self.neighbors.append(Node(current_node.i, current_node.j + 1,
                                       self.calG(Grid(current_node.i, current_node.j + 1), self.start_grid), 0,
                                       self.start_node,
                                       self.calDirection(Grid(current_node.i, current_node.j + 1), self.start_grid)))
            self.neighbors.append(Node(current_node.i - 1, current_node.j - 1,
                                       self.calG(Grid(current_node.i - 1, current_node.j - 1), self.start_grid), 0,
                                       self.start_node, self.calDirection(Grid(current_node.i - 1, current_node.j - 1),
                                                                          self.start_grid)))
            self.neighbors.append(Node(current_node.i - 1, current_node.j + 1,
                                       self.calG(Grid(current_node.i - 1, current_node.j + 1), self.start_grid), 0,
                                       self.start_node, self.calDirection(Grid(current_node.i - 1, current_node.j + 1),
                                                                          self.start_grid)))
            self.neighbors.append(Node(current_node.i + 1, current_node.j + 1,
                                       self.calG(Grid(current_node.i + 1, current_node.j + 1), self.start_grid), 0,
                                       self.start_node, self.calDirection(Grid(current_node.i + 1, current_node.j + 1),
                                                                          self.start_grid)))
            self.neighbors.append(Node(current_node.i + 1, current_node.j - 1,
                                       self.calG(Grid(current_node.i + 1, current_node.j - 1), self.start_grid), 0,
                                       self.start_node, self.calDirection(Grid(current_node.i + 1, current_node.j - 1),
                                                                          self.start_grid)))
            return
        else:
            direction = current_node.direction
            # diagonal condition
            if direction.i != 0 and direction.j != 0:
                if not self.isOccupied(Grid(current_node.i + direction.i, current_node.j)):
                    temp = Node(current_node.i + direction.i, current_node.j,
                                current_node.g + self.calG(Grid(current_node.i + direction.i, current_node.j),
                                                           current_node.grid), 0, current_node,
                                self.calDirection(Grid(current_node.i + direction.i, current_node.j),
                                                  current_node.grid))
                    self.neighbors.append(temp)
                if not self.isOccupied(Grid(current_node.i, current_node.j + direction.j)):
                    temp = Node(current_node.i, current_node.j + direction.j,
                                current_node.g + self.calG(Grid(current_node.i, current_node.j + direction.j),
                                                           current_node.grid), 0, current_node,
                                self.calDirection(Grid(current_node.i, current_node.j + direction.j),
                                                  current_node.grid))
                    self.neighbors.append(temp)
                if not self.isOccupied(Grid(current_node.i + direction.i, current_node.j + direction.j)):
                    temp = Node(current_node.i + direction.i, current_node.j + direction.j, current_node.g + self.calG(
                        Grid(current_node.i + direction.i, current_node.j + direction.j), current_node.grid), 0,
                                current_node,
                                self.calDirection(Grid(current_node.i + direction.i, current_node.j + direction.j),
                                                  current_node.grid))
                    self.neighbors.append(temp)
                if self.isForcedNeighbor(Grid(current_node.i + direction.i, current_node.j - direction.j),
                                         current_node):
                    temp = Node(current_node.i + direction.i, current_node.j - direction.j,
                                current_node.g + math.sqrt(2), 0, current_node, Grid(direction.i, -direction.j))
                    self.neighbors.append(temp)
                if self.isForcedNeighbor(Grid(current_node.i - direction.i, current_node.j + direction.j),
                                         current_node):
                    temp = Node(current_node.i - direction.i, current_node.j + direction.j,
                                current_node.g + math.sqrt(2),
                                0, current_node, Grid(-direction.i, direction.j))
                    self.neighbors.append(temp)
                return
            else:
                if not self.isOccupied(Grid(current_node.i + direction.i, current_node.j + direction.j)):
                    temp = Node(current_node.i + direction.i, current_node.j + direction.j, current_node.g + 1, 0,
                                current_node, direction)
                    self.neighbors.append(temp)
                # vertical condition
                if direction.i == 0:
                    if self.isForcedNeighbor(Grid(current_node.i - 1, current_node.j + direction.j), current_node):
                        temp = Node(current_node.i - 1, current_node.j + direction.j, current_node.g + math.sqrt(2), 0,
                                    current_node, Grid(-1, direction.j))
                        self.neighbors.append(temp)
                    if self.isForcedNeighbor(Grid(current_node.i + 1, current_node.j + direction.j), current_node):
                        temp = Node(current_node.i + 1, current_node.j + direction.j, current_node.g + math.sqrt(2), 0,
                                    current_node, Grid(1, direction.j))
                        self.neighbors.append(temp)
                    return
                # horizon condition
                if direction.j == 0:
                    if self.isForcedNeighbor(Grid(current_node.i + direction.i, current_node.j - 1), current_node):
                        temp = Node(current_node.i + direction.i, current_node.j - 1, current_node.g + math.sqrt(2), 0,
                                    current_node, Grid(direction.i, -1))
                        self.neighbors.append(temp)
                    if self.isForcedNeighbor(Grid(current_node.i + direction.i, current_node.j + 1), current_node):
                        temp = Node(current_node.i + direction.i, current_node.j + 1, current_node.g + math.sqrt(2), 0,
                                    current_node, Grid(direction.i, 1))
                        self.neighbors.append(temp)
                    return

    def findJumpPoint(self, current_node):
        if self.calH(current_node) == 0:
            return current_node
        if current_node.i < 0 or current_node.i >= self.grid_map.col or current_node.j < 0 or current_node.j >= self.grid_map.row:
            return None
        if self.grid_map.map[current_node.j][current_node.i] == 1:
            return None
        direction = current_node.direction
        # diagonal condition
        if direction.i != 0 and direction.j != 0:
            if self.isForcedNeighbor(Grid(current_node.i - direction.i, current_node.j + direction.j), current_node) \
                    or \
                    self.isForcedNeighbor(Grid(current_node.i + direction.i, current_node.j + direction.j),
                                          current_node):
                return current_node
            if self.findJumpPoint(
                    Node(current_node.i + direction.i, current_node.j, current_node.g + 1, 0, current_node,
                         Grid(direction.i, 0))):
                return current_node
            if self.findJumpPoint(
                    Node(current_node.i, current_node.j + direction.j, 1 + current_node.g, 0, current_node,
                         Grid(0, direction.j))):
                return current_node
        else:
            # vertical condition
            if self.isForcedNeighbor(Grid(current_node.i - 1, current_node.j + direction.j), current_node) \
                    or \
                    self.isForcedNeighbor(Grid(current_node.i + 1, current_node.j + direction.j), current_node):
                return current_node
            # horizontal condition
            if self.isForcedNeighbor(Grid(current_node.i + direction.i, current_node.j - 1), current_node) \
                    or \
                    self.isForcedNeighbor(Grid(current_node.i + direction.i, current_node.j + 1), current_node):
                return current_node
        next_node = Node(current_node.i + direction.i, current_node.j + direction.j, current_node.g + math.sqrt(2), 0,
                         current_node, Grid(direction.i, direction.j))
        return self.findJumpPoint(next_node)

    def generateBestPath(self):
        if len(self.close_list) == 0 or (self.close_list[-1].i != self.des_grid.i and self.close_list[-1].j != self.des_grid.j):
            print('Path not found')
            return None
        self.path_x = [
            int(self.close_list[-1].i * self.grid_map.grid_size + self.grid_map.min_x + self.grid_map.grid_size // 2)]
        self.path_y = [
            int(self.close_list[-1].j * self.grid_map.grid_size + self.grid_map.min_y + self.grid_map.grid_size // 2)]
        ptr = self.close_list[-1]
        index = self.close_list.index(ptr)
        while self.close_list[index].parent != -1:
            ptr = self.close_list[index].parent
            index = self.close_list.index(ptr)
            self.path_x.append(int(self.close_list[self.close_list.index(
                ptr)].i * self.grid_map.grid_size + self.grid_map.min_x + self.grid_map.grid_size // 2))
            self.path_y.append(int(self.close_list[self.close_list.index(
                ptr)].j * self.grid_map.grid_size + self.grid_map.min_y + self.grid_map.grid_size // 2))
        self.reversePathList()
        self.replan()
        self.simplifyPath()
        print("Path found")

    def reversePathList(self):
        for i in range(0,len(self.path_x)//2+1):
            temp_x = self.path_x[-i-1]
            self.path_x[-i-1] = self.path_x[i]
            self.path_x[i] = temp_x
            temp_y = self.path_y[-i - 1]
            self.path_y[-i - 1] = self.path_y[i]
            self.path_y[i] = temp_y

    def replan(self):
        if len(self.path_y) == 2:
            if not (self.path_x[-1] == self.des_grid.i * self.grid_map.grid_size + self.grid_map.min_x + self.grid_map.grid_size//2 and self.path_y[-1] == self.des_grid.j * self.grid_map.grid_size + self.grid_map.min_y + self.grid_map.grid_size//2):
                temp = self.path_x[0]
                self.path_x[0] = self.path_x[1]
                self.path_x[1] = temp
                temp = self.path_y[0]
                self.path_y[0] = self.path_y[1]
                self.path_y[1] = temp
                return
        for i in range(1,len(self.path_x)-1):
            if self.path_y[i] == self.path_y[i-1] and self.path_y[i+1] == self.path_y[i] and (self.path_x[i] - self.path_x[i-1])*(self.path_x[i+1]- self.path_x[i])<0\
                or self.path_x[i] == self.path_x[i-1] and self.path_x[i+1] == self.path_x[i] and (self.path_y[i] - self.path_y[i-1])*(self.path_y[i+1] - self.path_y[i])<0\
                    or abs(self.path_x[i]-self.path_x[i-1])==abs(self.path_y[i]-self.path_y[i-1]) and abs(self.path_x[i+1]-self.path_x[i])==abs(self.path_y[i]-self.path_y[i+1]) and (self.path_y[i] - self.path_y[i-1])*(self.path_y[i+1] - self.path_y[i])<0:
                temp = self.path_x[i]
                self.path_x[i] = self.path_x[i+1]
                self.path_x[i+1] = temp
                temp = self.path_y[i]
                self.path_y[i] = self.path_y[i+1]
                self.path_y[i+1] = temp
            if abs(self.path_x[i-1] - self.path_x[i]) != abs(self.path_y[i-1] - self.path_y[i]) and self.path_x[i-1] - self.path_x[i] != 0 and self.path_y[i-1] - self.path_y[i] != 0:
                temp = self.path_x[i]
                self.path_x[i] = self.path_x[i+1]
                self.path_x[i+1] = temp
                temp = self.path_y[i]
                self.path_y[i] = self.path_y[i+1]
                self.path_y[i+1] = temp

    def simplifyPath(self):
        path_x = [self.path_x[0]]
        path_y = [self.path_y[0]]
        for i in range(1,len(self.path_x)-1):
            if self.path_y[i+1] == self.path_y[i] and self.path_y[i] == self.path_y[i-1]:
                continue
            else:
                if self.path_y[i+1]-self.path_y[i] != 0 and self.path_y[i-1]-self.path_y[i] !=0:
                    if (self.path_x[i+1]-self.path_x[i])/(self.path_y[i+1]-self.path_y[i]) == (self.path_x[i-1]-self.path_x[i])/(self.path_y[i-1]-self.path_y[i]):
                        continue
                path_x.append(self.path_x[i])
                path_y.append(self.path_y[i])
        path_x.append(self.path_x[-1])
        path_y.append(self.path_y[-1])
        self.path_x = path_x
        self.path_y = path_y

    def jpsSearcher(self, vision):
        self.grid_map.loadObstacles(vision)
        self.grid_map.setOccupiedGrid()
        self.open_list.append(self.start_node)
        time_start = time.time()
        while len(self.open_list) > 0:
            node = self.findMinFInOpenList()
            self.close_list.append(node)
            if self.calH(node) == 0:
                break
            if time.time() - time_start > 0.05:
                print("path not found")
                break
            self.getNeighbors(node)
            for neighbor in self.neighbors:
                jump_point = self.findJumpPoint(neighbor)
                if not jump_point:
                    continue
                jump_point.parent = node
                jump_point.h = self.calH(jump_point)
                jump_point.f = jump_point.h + jump_point.g
                self.open_list.append(jump_point)
            del self.open_list[self.open_list.index(node)]
        self.generateBestPath()
        return self.path_x, self.path_y
