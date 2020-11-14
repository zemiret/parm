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
# prev_skew_dist_so0 = 0
# prev_skew_dist_so15 = 0
# tau_p = 30.0
# tau_d = 15.0
# tau_i = 0.0

parallel_skew = 0.0
so0_skew = 0.0
so1_skew = 0.0
so2_skew = 0.0
so3_skew = 0.0
so4_skew = 0.0
FRONT_VALID_READ = 0.7
WALL_DIST = 0.5

left_vel = MAX_SPEED
right_vel = MAX_SPEED

invalidated = False

def pd_steer(left_vel, right_vel, cur_skew, prev_skew, tau_p, tau_d):
    diff = (cur_skew - prev_skew) / timestep 
    steer = tau_p * cur_skew + tau_d * diff

    left_vel = max(min(MAX_SPEED, left_vel + steer), -MAX_SPEED)
    right_vel = max(min(MAX_SPEED, right_vel - steer), -MAX_SPEED)

    return left_vel, right_vel


while robot.step(timestep) != -1:
    so15_dist = getDistance(so15)
    so0_dist = getDistance(so0)

    so1_dist = getDistance(so1)
    so2_dist = getDistance(so2)
    so3_dist = getDistance(so3)
    so4_dist = getDistance(so4)
    so5_dist = getDistance(so5)

    so12_dist = getDistance(so12)
    so13_dist = getDistance(so13)
    so14_dist = getDistance(so14)

    if invalidated:
        left_vel = MAX_SPEED
        right_vel = MAX_SPEED
        invalidated = False

#    if so3_dist < 0.51 or so4_dist < 0.51:
#        # practically front to the wall
#        ## print('so3 valid')
#        print('Front to the wall: ', so3_dist, so4_dist)
#        invalidated = True
#
#        leftWheel.setVelocity(MAX_SPEED)
#        rightWheel.setVelocity(-MAX_SPEED)
#
#        while so3_dist < 2 or so4_dist < 2:
#            so3_dist = getDistance(so3)
#            so4_dist = getDistance(so4)
#            print('stepping here: ', so3_dist, so4_dist)
#            robot.step(timestep)

    cur_par_skew = so15_dist - so0_dist
    if is_read_valid(so0_dist) and is_read_valid(so15_dist)\
            and (0.4 < so0_dist < 0.8 or 0.4 < so15_dist < 0.8)\
            and -0.03 < cur_par_skew < 0.03:
        # keep current path
#        cur_skew = so15_dist - so0_dist # > 0 => going towards wall
        left_vel, right_vel = pd_steer(left_vel, right_vel, cur_par_skew, parallel_skew, 0.3, 6.0)
        parallel_skew = cur_par_skew

        so0_skew = 0
        so3_skew = 0
        so2_skew = 0
    else:
#        if so4_dist < FRONT_VALID_READ:
#            print('so4', so4_dist)
#            cur_skew = WALL_DIST - so4_dist
#            left_vel, right_vel = pd_steer(left_vel, right_vel, cur_skew, so4_skew, 10.0, 5.0)
#            so4_skew = cur_skew
        if so3_dist < FRONT_VALID_READ * 1.2:
            print('so3', so3_dist)
            cur_skew = WALL_DIST - so3_dist
            left_vel, right_vel = pd_steer(left_vel, right_vel, cur_skew, so3_skew, 5.0, 5.0)
            so3_skew = cur_skew

            parallel_skew = 0
            so0_skew = 0
        if so2_dist < FRONT_VALID_READ * 1.2:
            print('so2', so2_dist)
            cur_skew = WALL_DIST - so2_dist
            left_vel, right_vel = pd_steer(left_vel, right_vel, cur_skew, so2_skew, 3.0, 3.0)
            so2_skew = cur_skew

            parallel_skew = 0
            so3_skew = 0
            so0_skew = 0
        elif so1_dist < FRONT_VALID_READ:
            print('so1', so1_dist)
            cur_skew = WALL_DIST - so1_dist
            left_vel, right_vel = pd_steer(left_vel, right_vel, cur_skew, so1_skew, 1.0, 0.5)
            so1_skew = cur_skew

            
        elif so0_dist < FRONT_VALID_READ:
            print('so0', so0_dist)
            cur_skew = WALL_DIST - so0_dist
            left_vel, right_vel = pd_steer(left_vel, right_vel, cur_skew, so0_skew, 0.01, 5.0)
            so0_skew = cur_skew

            parallel_skew = 0
            so3_skew = 0
            so2_skew = 0
        else:
            # No front read at all (in range). Probably turn left
            left_vel = MAX_SPEED * 0.5675
            right_vel = MAX_SPEED

    print('LEFT, RIGHT: ', left_vel, right_vel)

    leftWheel.setVelocity(left_vel)
    rightWheel.setVelocity(right_vel)

# Stop the robot when we are done.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
