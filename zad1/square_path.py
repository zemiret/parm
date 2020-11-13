"""Sample Webots controller for the square path benchmark."""

from controller import Robot
import math

# Get pointer to the robot.
robot = Robot()

# Get pointer to each wheel of our robot.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

rightWheelSensor = robot.getPositionSensor('right wheel sensor')
rightWheelSensor.enable(1)

diam_mm = 195
diam_dm = diam_mm / 100
wheel_dist_mm = 300
wheel_dist_dm = wheel_dist_mm / 100
turn_dist_dm = wheel_dist_dm * math.pi / 4
side_dist_dm = 20
wheel_dist_modifier = wheel_dist_dm / 2

robot.step(1)
for i in range(0, 4):
    leftWheel.setPosition(1000)
    rightWheel.setPosition(1000)
   
    turn_dist_modifier = turn_dist_dm * i
    wheel_modifier = 0 if i == 0 else diam_dm
    next_stop_dm = side_dist_dm * (i + 1) + wheel_dist_modifier + wheel_modifier - turn_dist_modifier

    if i == 2:
        next_stop_dm += wheel_modifier * 3 / 2
    
    if i == 3: # account for not turning
        next_stop_dm += (wheel_dist_modifier * 2)
    
    while rightWheelSensor.getValue() < next_stop_dm:
        robot.step(1)

    if i == 3:  # it's faster without turning at the end
        leftWheel.setVelocity(0)
        rightWheel.setVelocity(0)
        break

    leftWheel.setPosition(1000)
    rightWheel.setPosition(-1000)

    turn_stop_dm = rightWheelSensor.getValue() - turn_dist_dm
    while rightWheelSensor.getValue() >= turn_stop_dm:
        robot.step(1)
        
    if i == 0:
        robot.step(1)

