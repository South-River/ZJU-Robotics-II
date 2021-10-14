from vision import Vision
from action import Action
from debug import Debugger
from jps import Jps
from feed_back_control import RobotAction
from feed_back_control import RobotAction_backward
import time
import math

if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()
    action.sendCommand(vx = 0, vy = 0, vw = 0)
    start_start_x, start_start_y = 2400, 1500
    end_end_x, end_end_y = -2400, -1500
    grid_size = 200
    for i in range(5):
        if i%2 ==0:
            goal_x, goal_y = end_end_x,end_end_y
        else:
            goal_x, goal_y = start_start_x, start_start_y
        while True:
            start_x, start_y = vision.my_robot.x, vision.my_robot.y
            jps = Jps(start_x, start_y, goal_x, goal_y, grid_size)
            time_start = time.time()
            path_x, path_y = jps.jpsSearcher(vision)
            time_end = time.time()
            debugger.draw_all(path_x, path_y, vision.my_robot.x, vision.my_robot.y, jps.grid_map.obstacle_x, jps.grid_map.obstacle_y)
            if i%2 == 0:
                move_robot = RobotAction(path_x,path_y)
            else:
                move_robot = RobotAction_backward(path_x,path_y)
            if move_robot.move2DesPoint(vision, action) == 1:
                action.sendCommand(vx = 0, vy = 0, vw = 0)
                break

