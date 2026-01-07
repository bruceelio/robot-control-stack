from sr.robot3 import Robot

robot = Robot()

#Go forward and backward at 50% power for 1 second 10 times, ending at the starting point
for i in range(10):
	robot.motor_board.motors[0].power = 0.5
	robot.motor_board.motors[1].power = 0.5
	robot.sleep(1)
	robot.motor_board.motors[0].power = -0.5
	robot.motor_board.motors[1].power = -0.5
	robot.sleep(1)