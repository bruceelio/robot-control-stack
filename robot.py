# robot.py  (Webots entry point; otherwise enter via main.py)

from sr.robot3 import Robot
from robot_controller import Controller

robot = Robot()
controller = Controller(robot)
controller.run()