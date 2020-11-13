"""Sample Webots controller for the wall following benchmark."""

from controller import Robot

def getDistance(sensor):
    """
    Return the distance of an obstacle for a sensor.

    The value returned by the getValue() method of the distance sensors
    corresponds to a physical value (here we have a sonar, so it is the
    strength of the sonar ray). This function makes a conversion to a
    distance value in meters.
    """
    return ((1000 - sensor.getValue()) / 1000) * 5

def is_read_valid(dist):
    return dist < 5.0


# Maximum speed for the velocity value of the wheels.
# Don't change this value.
MAX_SPEED = 5.24
MIN_SPEED = 3.0

# Get pointer to the robot.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Get pointer to the robot wheels motors.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

# We will use the velocity parameter of the wheels, so we need to
# set the target position to infinity.
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))

# Get and enable the distance sensors.
so0 = robot.getDistanceSensor("so0")
so1 = robot.getDistanceSensor("so1")
so2 = robot.getDistanceSensor("so2")
so3 = robot.getDistanceSensor("so3")
so4 = robot.getDistanceSensor("so4")
so5 = robot.getDistanceSensor("so5")
# so6 = robot.getDistanceSensor("so6")
# so7 = robot.getDistanceSensor("so7")
# so8 = robot.getDistanceSensor("so8")
# so9 = robot.getDistanceSensor("so9")
# so10 = robot.getDistanceSensor("so10")
# so11 = robot.getDistanceSensor("so11")
so12 = robot.getDistanceSensor("so12")
so13 = robot.getDistanceSensor("so13")
so14 = robot.getDistanceSensor("so14")
so15 = robot.getDistanceSensor("so15")

so0.enable(timestep)
so1.enable(timestep)
so2.enable(timestep)
so3.enable(timestep)
so4.enable(timestep)
so5.enable(timestep)
so12.enable(timestep)
so13.enable(timestep)
so14.enable(timestep)
so15.enable(timestep)

# Move forward until we are 50 cm away from the wall.
leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(MAX_SPEED)
while robot.step(timestep) != -1:
    if getDistance(so3) < 0.5:
        break

# Rotate clockwise until the wall is to our left.
leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(-MAX_SPEED)
while robot.step(timestep) != -1:
    # Rotate until there is a wall to our left, and nothing in front of us.
    if getDistance(so0) < 1:
        break


# Main loop.
integral = 0.0
prev_skew = 0
prev_skew_dist_so0 = 0
prev_skew_dist_so15 = 0
tau_p = 30.0
tau_d = 15.0
tau_i = 0.0

left_vel = MAX_SPEED
right_vel = MAX_SPEED

invalidated = False

while robot.step(timestep) != -1:
    so0_dist = getDistance(so0)
    so15_dist = getDistance(so15)

    so1_dist = getDistance(so1)
    so2_dist = getDistance(so2)
    so3_dist = getDistance(so3)
    so4_dist = getDistance(so4)
    so5_dist = getDistance(so5)

    so12_dist = getDistance(so12)
    so13_dist = getDistance(so13)
    so14_dist = getDistance(so14)

    if so3_dist < 0.51 or so4_dist < 0.51:
        # practically front to the wall
        ## print('so3 valid')
        print('Front to the wall: ', so3_dist, so4_dist)
        invalidated = True

        leftWheel.setVelocity(MAX_SPEED)
        rightWheel.setVelocity(-MAX_SPEED)

        while so3_dist < 2 or so4_dist < 2:
            so3_dist = getDistance(so3)
            so4_dist = getDistance(so4)
            print('stepping here: ', so3_dist, so4_dist)
            robot.step(timestep)


#     if not is_read_valid(so0_dist) and is_read_valid(so15_dist): # left turn (I hope at least)
#         # print('LEFT TURN!')
#         left_vel = (MAX_SPEED * 3/4)
#         right_vel = (MAX_SPEED)
#         leftWheel.setVelocity(left_vel)
#         rightWheel.setVelocity(right_vel)
#         while not is_read_valid(so0_dist):
#             robot.step(timestep)
# 
#         # print('BACK AT IT AGAIN')

#    if is_read_valid(so0_dist) and is_read_valid(so15_dist):
    # print(so0_dist, so15_dist)

    if 0.45 < so0_dist < 0.55 and 0.45 < so15_dist < 0.55:
        # keep current path
        if invalidated:
            left_vel = MAX_SPEED
            right_vel = MAX_SPEED
            invalidated = False

        cur_skew = so15_dist - so0_dist # > 0 => going towards wall
        diff = (cur_skew - prev_skew) / timestep # > the bigger abs value, the steeper the error func

        steer = tau_p * cur_skew + tau_d * diff
        prev_skew = cur_skew            

        left_vel = max(min(MAX_SPEED, left_vel + steer), MIN_SPEED)
        right_vel = max(min(MAX_SPEED, right_vel - steer), MIN_SPEED)
#    elif not is_read_valid(so0_dist):
    else:
        # TODO: Idea. if both are valid, choose the bigger value
        if is_read_valid(so0_dist):
            if invalidated:
                left_vel = MAX_SPEED
                right_vel = MAX_SPEED
                invalidated = False

            cur_skew = 0.5 - so0_dist # > 0 = need to turn away from the wall
            diff = (cur_skew - prev_skew_dist_so0) / timestep # > the bigger abs value, the steeper the error func

            steer = tau_p * cur_skew + tau_d * diff
            prev_skew_dist_so0 = cur_skew            
            
            # print('STEER: ', steer)

            left_vel = max(min(MAX_SPEED, left_vel + steer), MIN_SPEED)
            right_vel = max(min(MAX_SPEED, right_vel - steer), MIN_SPEED)

        elif is_read_valid(so15_dist):
            if invalidated:
                left_vel = MAX_SPEED
                right_vel = MAX_SPEED
                invalidated = False

            cur_skew = 0.5 - so0_dist # > 0 = need to turn away from the wall
            diff = (cur_skew - prev_skew_dist_so15) / timestep # > the bigger abs value, the steeper the error func

            steer = tau_p * cur_skew + tau_d * diff
            prev_skew_dist_so15 = cur_skew            

            left_vel = max(min(MAX_SPEED, left_vel + steer), MIN_SPEED)
            right_vel = max(min(MAX_SPEED, right_vel - steer), MIN_SPEED)

        ## print('So0 invalid')

#        elif is_read_valid(so3_dist):
#            # print('so3 valid')
#            left_vel = MAX_SPEED
#            right_vel = MAX_SPEED * 0.01
#        elif is_read_valid(so2_dist):
#            # print('so2 valid')
#            left_vel = MAX_SPEED
#            right_vel = MAX_SPEED * 0.01
        elif so1_dist < 1.0:
            invalidated = True
            # # print('so1 valid')
            left_vel = MAX_SPEED
            # right_vel = MAX_SPEED * 0.01
            right_vel = -MAX_SPEED

            while not is_read_valid(so0_dist):
                so0_dist = getDistance(so0)
                robot.step(timestep)


#         elif is_read_valid(so15_dist):
#             # print('so15 valid')
#             left_vel = MAX_SPEED * 0.01 
#             right_vel = MAX_SPEED
# 
#         elif is_read_valid(so12_dist):
#             # print('so12 valid')
#             left_vel = 0.01
#             right_vel = MAX_SPEED
#         elif is_read_valid(so13_dist):
#             # print('so13 valid')
#             left_vel = MAX_SPEED * 0.01 
#             right_vel = MAX_SPEED
#         elif is_read_valid(so14_dist):
#             # print('so14 valid')
#             left_vel = MAX_SPEED * 0.01 
#             right_vel = MAX_SPEED
        else:
            # nothing valid. Probably turn left
            # print('so0 invalid, nothing valid')
            invalidated = True
            left_vel = MAX_SPEED * 0.4
            right_vel = MAX_SPEED

#    elif not is_read_valid(so15_dist):
#        invalidated = True
#        # print('SO15 invalid')
#        so12_dist = getDistance(so12)
#        so13_dist = getDistance(so13)
#        so14_dist = getDistance(so14)
#
#        if is_read_valid(so12_dist):
#            # print('so12 valid')
#            left_vel = MAX_SPEED * 0.01
#            right_vel = MAX_SPEED
#        elif is_read_valid(so13_dist):
#            # print('so13 valid')
#            left_vel = MAX_SPEED * 0.01
#            right_vel = MAX_SPEED
#        elif is_read_valid(so14_dist):
#            # print('so14 valid')
#            left_vel = MAX_SPEED * 0.01
#            right_vel = MAX_SPEED

    leftWheel.setVelocity(left_vel)
    rightWheel.setVelocity(right_vel)


    # Too close to the wall, we need to turn right.
#    if getDistance(so0) < 0.4:
#        leftWheel.setVelocity(left_vel + steer)
#        rightWheel.setVelocity(right_vel - steer)
#
#        leftWheel.setVelocity(MAX_SPEED)
#        rightWheel.setVelocity(MAX_SPEED * 0.9)
#
#    # Too far from the wall, we need to turn left.
#    elif getDistance(so0) > 0.6:
#        leftWheel.setVelocity(left_vel - steer)
#        rightWheel.setVelocity(right_vel + steer)
#
#        leftWheel.setVelocity(MAX_SPEED * 0.9)
#        rightWheel.setVelocity(MAX_SPEED)
#
#    # We are in the right direction.
#    else:
#        left_vel = MAX_SPEED
#        right_vel = MAX_SPEED
#
#        leftWheel.setVelocity(MAX_SPEED)
#        rightWheel.setVelocity(MAX_SPEED)

# Stop the robot when we are done.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
