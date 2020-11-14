"""Sample Webots controller for the wall following benchmark."""

from controller import Robot
import math

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

cos40 = math.cos(math.pi / 9 * 2)
cos60 = math.cos(math.pi / 3)
cos80 = math.cos(math.pi / 9 * 4)

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

parallel_skew = 0.0
skews = [0 for _ in range(16)]

FRONT_VALID_READ = 0.6
WALL_DIST = 0.5

left_vel = MAX_SPEED
right_vel = MAX_SPEED

invalidated = False

so15_dist = 0
so0_dist = 0

so1_dist = 0
so2_dist = 0
so3_dist = 0
so4_dist = 0
so5_dist = 0

so12_dist = 0
so13_dist = 0
so14_dist = 0

def pd_steer(left_vel, right_vel, cur_skew, prev_skew, tau_p, tau_d):
    diff = (cur_skew - prev_skew) / timestep 
    steer = tau_p * cur_skew + tau_d * diff

    left_vel = max(min(MAX_SPEED, left_vel + steer), -MAX_SPEED)
    right_vel = max(min(MAX_SPEED, right_vel - steer), -MAX_SPEED)

    return left_vel, right_vel


def clear_other_skews(arr, idx):
    for i in range(len(arr)):
        if i != idx:
            arr[i] = 0

def front_dist():
    if so0_dist < FRONT_VALID_READ:
        return so0_dist
    elif so3_dist < FRONT_VALID_READ:
        return so3_dist * cos80
    elif so2_dist < FRONT_VALID_READ:
        return so2_dist * cos60
    elif so1_dist < FRONT_VALID_READ:
        return so1_dist * cos40
    else:
        print("TOTALY INVALID FRONT")
        return 0.5

def rear_dist():
    if so15_dist < FRONT_VALID_READ:
        return so15_dist
    elif so14_dist < FRONT_VALID_READ:
        return so14_dist * cos40
    elif so13_dist < FRONT_VALID_READ:
        return so13_dist * cos60
    elif so0_dist < FRONT_VALID_READ:
        return so0_dist 
    elif so1_dist < FRONT_VALID_READ:
        return so1_dist * cos40
    elif so2_dist < FRONT_VALID_READ:
        return so2_dist * cos60
    else:
        print("TOTALY INVALID REAR")
        return 0.5


tau_p = 0.8
tau_d = 0.4
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
        clear_other_skews(skews, -1)

    if so4_dist < FRONT_VALID_READ:
        print('so4', so3_dist)
        cur_skew = WALL_DIST - so4_dist
        left_vel, right_vel = pd_steer(left_vel, right_vel, cur_skew, skews[4], 5.0, 5.0)
        skews[4] = cur_skew
        clear_other_skews(skews, 4)

    elif so3_dist < FRONT_VALID_READ:
        print('so3', so3_dist)
        cur_skew = WALL_DIST - so3_dist
        left_vel, right_vel = pd_steer(left_vel, right_vel, cur_skew, skews[3], 5.0, 5.0)
        skews[3] = cur_skew
        clear_other_skews(skews, 3)

#    elif so2_dist < FRONT_VALID_READ :
#        print('so2', so2_dist)
#        cur_skew = WALL_DIST - so2_dist
#        left_vel, right_vel = pd_steer(left_vel, right_vel, cur_skew, skews[2], 3.0, 3.0)
#        skews[2] = cur_skew
#        clear_other_skews(skews, 2)
    else:
        if so0_dist < FRONT_VALID_READ or so1_dist < FRONT_VALID_READ\
                or so2_dist < FRONT_VALID_READ\
                or so3_dist < FRONT_VALID_READ\
                or so4_dist < FRONT_VALID_READ:

            front = front_dist()
            rear = rear_dist()

            dist_skew = 0.5 - min(front, rear)
            angle_skew = rear - front

            if dist_skew > 0 and angle_skew > 0 or dist_skew < 0 and angle_skew < 0:
                steer = tau_p * dist_skew + tau_d * angle_skew
                left_vel = max(min(MAX_SPEED, left_vel + steer), -MAX_SPEED)
                right_vel = max(min(MAX_SPEED, right_vel - steer), -MAX_SPEED)
                clear_other_skews(skews, -1)


#        if so0_dist < FRONT_VALID_READ  and so15_dist < FRONT_VALID_READ :
#            dist_skew = 0.5 - min(so0_dist, so15_dist)
#            angle_skew = so15_dist - so0_dist
#            steer = tau_p * dist_skew + tau_d * angle_skew
#            left_vel = max(min(MAX_SPEED, left_vel + steer), -MAX_SPEED)
#            right_vel = max(min(MAX_SPEED, right_vel - steer), -MAX_SPEED)
#            clear_other_skews(skews, -1)
#
#        elif so0_dist < FRONT_VALID_READ :
#            cur_skew = WALL_DIST - so0_dist
#            left_vel, right_vel = pd_steer(left_vel, right_vel, cur_skew, skews[0], 0.004, 5.0)
#            skews[0] = cur_skew
#            clear_other_skews(skews, 0)
#
#        elif so15_dist < FRONT_VALID_READ :
#            cur_skew = WALL_DIST - so15_dist
#            left_vel, right_vel = pd_steer(left_vel, right_vel, cur_skew, skews[15], 0.004, 5.0)
#            skews[15] = cur_skew
#            clear_other_skews(skews, 15)
#        elif so1_dist < FRONT_VALID_READ :
#            print('so1', so1_dist)
#            cur_skew = WALL_DIST - so1_dist
#            left_vel, right_vel = pd_steer(left_vel, right_vel, cur_skew, skews[1], 0.5, 0.5)
#            skews[1] = cur_skew
#            clear_other_skews(skews, 1)
        else:
            # No front read at all (in range). Probably turn left
            left_vel = MAX_SPEED * 0.51
            right_vel = MAX_SPEED
            invalidated = True

    print('LEFT, RIGHT: ', left_vel, right_vel)

    leftWheel.setVelocity(left_vel)
    rightWheel.setVelocity(right_vel)

# Stop the robot when we are done.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
