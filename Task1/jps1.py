import math


class Node(object):
    def __init__(self, i, j, g, h, parent, direction):
        self.i = i
        self.j = j
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
        self.direction = direction  # (1,1) means up right, (-1,1) means up left, (-1,1)means low right, (-1,-1)means low left


class Grid(object):
    def __init__(self, i, j):
        self.i = i
        self.j = j
        self.g = 0
        self.parent = Grid(-1, -1)

    def setParent(self, parent):
        self.parent = parent

    def setG(self, g):
        self.g = g


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
        self.map_top_right = Point(4500, 3000)
        self.map_buttom_left = Point(-4500, -3000)
        self.row = (self.max_y - self.min_y) // grid_size
        self.col = (self.max_x - self.min_y) // grid_size
        self.map = [[0 for j in range(self.col)] for i in range(self.row)]
        self.obstacle_list = []

    def transfer2MapCoor(self, x, y):
        i = (x - self.min_x) // self.grid_size
        j = (y - self.min_y) // self.grid_size
        return Grid(i, j)

    def loadObstacles(self, vision):
        for blue_robot in vision.blue_robot:
            if blue_robot.visible and blue_robot.id > 0:
                self.obstacle_list.append(Point(blue_robot.x, blue_robot.y))
        for yellow_robot in vision.yellow_robot:
            self.obstacle_list.append((Point(yellow_robot.x, yellow_robot.y)))

    # 0 means free, 1 means occupied
    def setOccupiedGrid(self):
        for obstacle in self.obstacle_list:
            obstacle_buttom_left_map_coor = self.transfer2MapCoor(obstacle.x - self.grid_size,
                                                                  obstacle.y - self.grid_size)
            obstacle_top_right_map_coor = self.transfer2MapCoor(obstacle.x + self.grid_size,
                                                                obstacle.y + self.grid_size)
            for i in range(obstacle_buttom_left_map_coor.i, obstacle_top_right_map_coor.i + 1):
                for j in range(obstacle_buttom_left_map_coor.j, obstacle_top_right_map_coor.j + 1):
                    self.map[i][j] = 1


class Jps(object):
    def __init__(self, start_x, start_y, des_x, des_y, grid_size):
        self.grid_map = GridMap(grid_size)
        self.start_grid = self.grid_map.transfer2MapCoor(start_x, start_y)
        self.des_grid = self.grid_map.transfer2MapCoor(des_x, des_y)
        self.neighbors = []
        self.open_list = []
        self.close_list = []
        self.path_x = []
        self.path_y = []

    def calH(self, current_grid):
        return math.sqrt((current_grid.i - self.des_grid.i) ** 2 + (current_grid.j - self.des_grid.j) ** 2)

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
        if self.grid_map.map[grid.i][grid.j] == 0:
            return False
        else:
            return True

    def isForcedNeighbor(self, neighbor, current_grid, last_grid, direction):
        # diagonal condition
        if direction.i != 0 and direction.j != 0:
            if self.grid_map.map[(neighbor.i + last_grid.i) / 2][(neighbor.j + last_grid.j) / 2] == 1:
                return True
            else:
                return False
        # vertical condition
        if direction.i == 0:
            if self.grid_map.map[neighbor.i][current_grid.j] == 1:
                return True
            else:
                return False
        # horizontal condition
        if direction.j == 0:
            if self.grid_map.map[current_grid.i][neighbor.j] == 1:
                return True
            else:
                return False

    def getNeighbors(self, current_node, last_node):
        self.neighbors = []
        if current_node.i == self.start_grid.i and current_node.j == self.start_grid.j:
            self.neighbors.append(Grid(current_node.i - 1, current_node.j))
            self.neighbors.append(Grid(current_node.i + 1, current_node.j))
            self.neighbors.append(Grid(current_node.i, current_node.j - 1))
            self.neighbors.append(Grid(current_node.i, current_node.j + 1))
            self.neighbors.append(Grid(current_node.i - 1, current_node.j - 1))
            self.neighbors.append(Grid(current_node.i - 1, current_node.j + 1))
            self.neighbors.append(Grid(current_node.i + 1, current_node.j + 1))
            self.neighbors.append(Grid(current_node.i + 1, current_node.j - 1))
            for neighbor in self.neighbors:
                neighbor.setParent(Grid(current_node.i, current_node.j))
                neighbor.setG(self.calG(neighbor, self.start_grid))
            return
        else:
            direction = self.calDirection(Grid(current_node.i, current_node.j), Grid(last_node.i, last_node.j))
            # diagonal condition
            if direction.i != 0 and direction.j != 0:
                if not self.isOccupied(Grid(current_node.i + direction.i, current_node.j)):
                    temp = Grid(current_node.i + direction.i, current_node.j)
                    temp.setParent(Grid(current_node.i, current_node.j))
                    temp.setG(current_node.g + self.calG(temp, Grid(current_node.i, current_node.j)))
                    self.neighbors.append(temp)
                if not self.isOccupied(Grid(current_node.i, current_node.j + direction.j)):
                    temp = Grid(current_node.i + direction.i, current_node.j)
                    temp.setParent(Grid(current_node.i, current_node.j + direction.j))
                    temp.setG(current_node.g + self.calG(temp, Grid(current_node.i, current_node.j)))
                    self.neighbors.append(temp)
                if not self.isOccupied(Grid(current_node.i + direction.i, current_node.j + direction.j)):
                    temp = Grid(current_node.i + direction.i, current_node.j + direction.j)
                    temp.setParent(Grid(current_node.i, current_node.j))
                    temp.setG(current_node.g + self.calG(temp, Grid(current_node.i, current_node.j)))
                    self.neighbors.append(temp)
                if self.isForcedNeighbor(Grid(current_node.i + direction.i, current_node.j - direction.j),
                                         Grid(current_node.i, current_node.j), Grid(last_node.i, last_node.j),
                                         direction):
                    temp = Grid(current_node.i + direction.i, current_node.j - direction.j)
                    temp.setParent(Grid(current_node.i, current_node.j))
                    temp.setG(current_node.g + self.calG(temp, Grid(current_node.i, current_node.j)))
                    self.neighbors.append(temp)
                if self.isForcedNeighbor(Grid(current_node.i - direction.i, current_node.j + direction.j),
                                         Grid(current_node.i, current_node.j), Grid(last_node.i, last_node.j),
                                         direction):
                    temp = Grid(current_node.i - direction.i, current_node.j + direction.j)
                    temp.setParent(Grid(current_node.i, current_node.j))
                    temp.setG(current_node.g + self.calG(temp, Grid(current_node.i, current_node.j)))
                    self.neighbors.append(temp)
                return
            else:
                if not self.isOccupied(Grid(current_node.i + direction.i, current_node.j + direction.j)):
                    temp = Grid(current_node.i + direction.i, current_node.j + direction.j)
                    temp.setParent(Grid(current_node.i, current_node.j))
                    temp.setG(current_node.g + self.calG(temp, Grid(current_node.i, current_node.j)))
                    self.neighbors.append(temp)
                # vertical condition
                if direction.i == 0:
                    if self.isForcedNeighbor(Grid(current_node.i - 1, current_node.j + direction.j),
                                             Grid(current_node.i, current_node.j), Grid(last_node.i, last_node.j)):
                        temp = Grid(current_node.i - 1, current_node.j + direction.j)
                        temp.setParent(Grid(current_node.i, current_node.j))
                        temp.setG(current_node.g + self.calG(temp, Grid(current_node.i, current_node.j)))
                        self.neighbors.append(temp)
                    if self.isForcedNeighbor(Grid(current_node.i + 1, current_node.j + direction.j),
                                             Grid(current_node.i, current_node.j), Grid(last_node.i, last_node.j)):
                        temp = Grid(current_node.i + 1, current_node.j + direction.j)
                        temp.setParent(Grid(current_node.i, current_node.j))
                        temp.setG(current_node.g + self.calG(temp, Grid(current_node.i, current_node.j)))
                        self.neighbors.append(temp)
                    return
                # horizon condition
                if direction.j == 0:
                    if self.isForcedNeighbor(Grid(current_node.i + direction.i, current_node.j - 1),
                                             Grid(current_node.i, current_node.j), Grid(last_node.i, last_node.j)):
                        temp = Grid(current_node.i + direction.i, current_node.j - 1)
                        temp.setParent(Grid(current_node.i, current_node.j))
                        temp.setG(current_node.g + self.calG(temp, Grid(current_node.i, current_node.j)))
                        self.neighbors.append(temp)
                    if self.isForcedNeighbor(Grid(current_node.i + direction.i, current_node.j + 1),
                                             Grid(current_node.i, current_node.j), Grid(last_node.i, last_node.j)):
                        temp = Grid(current_node.i + direction.i, current_node.j + 1)
                        temp.setParent(Grid(current_node.i, current_node.j))
                        temp.setG(current_node.g + self.calG(temp, Grid(current_node.i, current_node.j)))
                        self.neighbors.append(temp)
                    return
        return

    def findJumpPoint(self, current_grid, last_grid):
        if self.calH(current_grid) <= 1:
            return current_grid
        current_grid.setG(last_grid.g + self.calG(current_grid, last_grid))
        direction = self.calDirection(current_grid, last_grid)

        if self.grid_map.map[current_grid.i][
            current_grid.j] == 1 or current_grid.i < 0 or current_grid.i >= self.grid_map.col or current_grid.j < 0 or current_grid.j > self.grid_map.row:
            return None

        # diagonal condition
        if direction.i != 0 and direction.j != 0:
            if self.isForcedNeighbor(Grid(current_grid.i - direction.i, current_grid.j + direction.j), current_grid,
                                     last_grid, direction) \
                    or \
                    self.isForcedNeighbor(Grid(current_grid.i + direction.i, current_grid.j - direction.j),
                                          current_grid, last_grid, direction):
                return current_grid
            if self.findJumpPoint(Grid(current_grid.i + direction.i, current_grid.i),
                                  Grid(current_grid.i, current_grid.j)):
                return current_grid
            if self.findJumpPoint(Grid(current_grid.i, current_grid.i + direction.j), current_grid):
                return current_grid

        else:
            # vertical condition
            if self.isForcedNeighbor(Grid(current_grid.i - 1, current_grid.j + direction.j), current_grid, last_grid,
                                     direction) \
                    or \
                    self.isForcedNeighbor(Grid(current_grid.i + 1, current_grid.j + direction.j), current_grid,
                                          last_grid, direction):
                return current_grid
            # horizontal condition
            if self.isForcedNeighbor(Grid(current_grid.i + direction.i, current_grid.j - 1), current_grid, last_grid,
                                     direction) \
                    or \
                    self.isForcedNeighbor(Grid(current_grid.i + direction.i, current_grid.j - 1), current_grid,
                                          last_grid, direction):
                return current_grid

        return self.findJumpPoint(Grid(current_grid.i + direction.i, current_grid.j + direction.j), current_grid)

    def generateBestPath(self):
        if len(self.close_list) == 0:
            print('Path not found')
            return None
        return self.close_list

    def jpsSearcher(self):
        start_node = Node(self.start_grid.i, self.start_grid.j, 0, self.calH(self.start_grid), -1, Grid(0, 0))
        self.open_list.append(start_node)

        while len(self.open_list) > 0:
            node = self.findMinFInOpenList()
            self.close_list.append(node)
            if self.calH(node) <= 1:
                break

            if node.i == start_node.i and node.j == start_node.j:
                self.getNeighbors(start_node, start_node)
            else:
                self.getNeighbors(node, Grid(node.i - node.direction.i, node.j - node.direction.j))

            for neighbor in self.neighbors:
                jump_point = self.findJumpPoint(neighbor, neighbor.parent)
                jump_point_h = self.calH(Grid(jump_point.i, jump_point.j))
                jump_node = Node(jump_point.i, jump_point.j, jump_point.g, jump_point_h, node,
                                 Grid(jump_point.i - jump_point.parent.i, jump_point.j - jump_point.parent.j))
                self.open_list.append(jump_node)

            del self.open_list[self.open_list.index(node)]

        for element in self.close_list:
            self.path_x.append(element.i * self.grid_map.grid_size + self.grid_map.min_x)
            self.path_y.append(element.j * self.grid_map.grid_size + self.grid_map.min_y)
        return self.path_x, self.path_y
