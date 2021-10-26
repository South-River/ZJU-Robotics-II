from action import Action
from vision import Vision
import time
import math

def calEuclideanDistance(point_A, point_B):
    return math.sqrt((point_A.x - point_B.x) ** 2 + (point_A.y - point_B.y) ** 2)

class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y

class RobotAction():
    def __init__(self, path_x, path_y):
        self.v_max = 1500
        self.mu = 0.5
        self.lamda = 2
        self.k_1 = 0.3
        self.k_2 = 3
        self.rho = 0
        self.alpha = 0
        self.beta = 0
        self.kappa = 0
        self.path_x = path_x
        self.path_y = path_y

    def move2DesPoint(self, vision, action):
        if len(self.path_x) <=0:
            return
        if calEuclideanDistance(Point(self.path_x[-1],self.path_y[-1]),Point(vision.my_robot.x,vision.my_robot.y))<=100:
            action.sendCommand(vx = 0, vy = 0, vw = 0)
            return 1
        self.move2NextNode(vision, action, Point(self.path_x[1], self.path_y[1]))

    def move2NextNode(self, vision, action, node_coor):
        robot_coor = Point(vision.my_robot.x, vision.my_robot.y)
        self.calRho(robot_coor, node_coor)
        self.calBeta(robot_coor, node_coor)
        self.calAlpha(robot_coor, node_coor, vision.my_robot.orientation)
        self.calKappa()
        v = self.v_max / (1 + self.mu * abs(self.kappa) ** self.lamda)
        w = v * self.kappa
        action.sendCommand(vx = v, vy = 0, vw = w)

    def calKappa(self):
        self.kappa = (self.k_2 * (self.alpha - math.atan(-self.k_1 * self.beta)) + (1 + self.k_1/(1 + (self.k_1 * self.beta) ** 2)) * math.sin(self.alpha)) / self.rho

    def calRho(self, robot_coor, node_coor):
        self.rho = calEuclideanDistance(robot_coor, node_coor)

    def calBeta(self, robot_coor, node_coor):
        self.beta = math.atan2(node_coor.y-robot_coor.y, node_coor.x-robot_coor.x)
        goal_angle = math.atan2(self.path_y[1] - self.path_y[0], self.path_x[1] - self.path_x[0])
        self.beta -= goal_angle
        while self.beta >= math.pi:
            self.beta -= 2 * math.pi
        while self.beta <= -math.pi:
            self.beta += 2 * math.pi

    def calAlpha(self, robot_coor, node_coor, orientation):
        self.alpha = math.atan2(node_coor.y-robot_coor.y,node_coor.x-robot_coor.x) - orientation

        while self.alpha >= math.pi:
            self.alpha -= 2 * math.pi
        while self.alpha <= -math.pi:
            self.alpha += 2 * math.pi


class RobotAction_backward():
    def __init__(self, path_x, path_y):
        self.v_max = 1500
        self.mu = 0.5
        self.lamda = 2
        self.k_1 = 0.3
        self.k_2 = 3
        self.rho = 0
        self.alpha = 0
        self.beta = 0
        self.kappa = 0
        self.path_x = path_x
        self.path_y = path_y

    def move2DesPoint(self, vision, action):
        if len(self.path_x) <=0:
            return
        if calEuclideanDistance(Point(self.path_x[-1],self.path_y[-1]),Point(vision.my_robot.x,vision.my_robot.y))<=100:
            action.sendCommand(vx = 0, vy = 0, vw = 0)
            return 1
        self.move2NextNode(vision, action, Point(self.path_x[1], self.path_y[1]))

    def move2NextNode(self, vision, action, node_coor):
        robot_coor = Point(vision.my_robot.x, vision.my_robot.y)
        self.calRho(robot_coor, node_coor)
        self.calBeta(robot_coor, node_coor)
        self.calAlpha(robot_coor, node_coor, vision.my_robot.orientation)
        self.calKappa()
        v = self.v_max / (1 + self.mu * abs(self.kappa) ** self.lamda)
        w = v * self.kappa
        action.sendCommand(vx = -v, vy = 0, vw = w)

    def calKappa(self):
        self.kappa = (self.k_2 * (self.alpha - math.atan(-self.k_1 * self.beta)) + (1 + self.k_1/(1 + (self.k_1 * self.beta) ** 2)) * math.sin(self.alpha)) / self.rho

    def calRho(self, robot_coor, node_coor):
        self.rho = calEuclideanDistance(robot_coor, node_coor)

    def calBeta(self, robot_coor, node_coor):
        self.beta = math.atan2(node_coor.y-robot_coor.y, node_coor.x-robot_coor.x)
        goal_angle = math.atan2(self.path_y[1] - self.path_y[0], self.path_x[1] - self.path_x[0])
        self.beta -= goal_angle
        while self.beta >= math.pi:
            self.beta -= 2 * math.pi
        while self.beta <= -math.pi:
            self.beta += 2 * math.pi

    def calAlpha(self, robot_coor, node_coor, orientation):
        self.alpha = math.atan2(node_coor.y-robot_coor.y,node_coor.x-robot_coor.x) - orientation - math.pi

        while self.alpha >= math.pi:
            self.alpha -= 2 * math.pi
        while self.alpha <= -math.pi:
            self.alpha += 2 * math.pi