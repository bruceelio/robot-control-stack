from sr.robot3 import Robot

#Create the robot object
robot = Robot()

#set the left motor to 50% power and right motor to 0% power
robot.motor_board.motors[0].power = 0.5
robot.motor_board.motors[1].power = 0
#Robot power goes off at end of code, so wait for 1 second
robot.sleep(1)