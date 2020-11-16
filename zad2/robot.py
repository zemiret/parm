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

cos10 = math.cos(math.pi / 18)
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

# Constants used to tune everything

FRONT_VALID_READ = 0.55
FRONT_READ_VALID_MODIFIER = 1.0
WALL_DIST = 0.4

so4_tau_p = 3.0
so4_tau_d = 3.0

so3_tau_p = 3.0
so3_tau_d = 3.0

angle_skew_reaction = 0.3

left_turn_velocity_prop = 0.53

tau_p = 0.4
tau_d = 0.3


# Move forward until we are 50 cm away from the wall.
leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(MAX_SPEED)
while robot.step(timestep) != -1:
    if getDistance(so3) < WALL_DIST * cos10:
        break

# Rotate clockwise until the wall is to our left.
leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(-MAX_SPEED)
while robot.step(timestep) != -1:
    # Rotate until there is a wall to our left, and nothing in front of us.
    if getDistance(so0) < 1:
        break


skews = [0 for _ in range(16)]

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
        return WALL_DIST

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
        return WALL_DIST


left_vel = MAX_SPEED
right_vel = MAX_SPEED
invalidated = False
front_turn = False

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

    if so4_dist < FRONT_VALID_READ * FRONT_READ_VALID_MODIFIER:
        cur_skew = WALL_DIST - so4_dist
        left_vel, right_vel = pd_steer(left_vel, right_vel, cur_skew, skews[4], so4_tau_p, so4_tau_d)
        skews[4] = cur_skew
        clear_other_skews(skews, 4)
        front_turn = True

    elif so3_dist < FRONT_VALID_READ * FRONT_READ_VALID_MODIFIER:
        cur_skew = WALL_DIST - so3_dist
        left_vel, right_vel = pd_steer(left_vel, right_vel, cur_skew, skews[3], so3_tau_p, so3_tau_d)
        skews[3] = cur_skew
        clear_other_skews(skews, 3)
        front_turn = True

    else:
        if front_turn:
            left_vel = MAX_SPEED
            right_vel = MAX_SPEED
            front_turn = False
            clear_other_skews(skews, -1)

        if so0_dist < FRONT_VALID_READ or so1_dist < FRONT_VALID_READ\
                or so2_dist < FRONT_VALID_READ\
                or so3_dist < FRONT_VALID_READ\
                or so4_dist < FRONT_VALID_READ:

            front = front_dist()
            rear = rear_dist()

            dist_skew = WALL_DIST - (front + rear) / 2
            angle_skew = rear - front

            if dist_skew > 0 and angle_skew >= -angle_skew_reaction or dist_skew < 0 and angle_skew <= angle_skew_reaction:
                steer = tau_p * dist_skew + tau_d * angle_skew
                left_vel = max(min(MAX_SPEED, left_vel + steer), -MAX_SPEED)
                right_vel = max(min(MAX_SPEED, right_vel - steer), -MAX_SPEED)
                clear_other_skews(skews, -1)

        else:
            # No front read at all (in range). Probably turn left
            left_vel = MAX_SPEED * left_turn_velocity_prop
            right_vel = MAX_SPEED
            invalidated = True

    leftWheel.setVelocity(left_vel)
    rightWheel.setVelocity(right_vel)

# Stop the robot when we are done.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
