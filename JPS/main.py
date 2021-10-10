from vision import Vision
from action import Action
from debug import Debugger
from jps import Jps
from zero_blue_action import zero_blue_action
import time
import math

if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()

    while True:
        time.sleep(1)

        for i in range(10000):
            start_x, start_y = vision.my_robot.x, vision.my_robot.y
            goal_x, goal_y = -2400, -1500
            grid_size = 200
            jps = Jps(start_x, start_y, goal_x, goal_y, grid_size)

            time_start = time.time()
            path_x, path_y = jps.jpsSearcher(vision)
            time_end = time.time()
            print("JPS cost:", time_end - time_start)
            print(path_x)
            print(path_y)

            action.sendCommand(vx=0, vy=0, vw=0)
            debugger.draw_all(path_x, path_y, vision.my_robot.x, vision.my_robot.y)

            time.sleep(1)