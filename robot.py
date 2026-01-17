# sr_board.py  (SR / Webots entry point)

from sr.robot3 import Robot
from robot_controller import Controller


robot = Robot()
controller = Controller(robot)
controller.run()
