"""Sample Webots controller for highway driving benchmark."""

# TODO: IEAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAS
# Try to always keep the left lane. I.E. Some exact distance from the left railing
# If needed, adjust speed, overtake to the right, etc, but always try to navigte back 
# to the left railing

from vehicle import Driver
import time

# name of the available distance sensors
sensorsNames = [
    'front',
    'front right 0',
    'front right 1',
    'front right 2',
    'front left 0',
    'front left 1',
    'front left 2',
    'rear',
    'rear left',
    'rear right',
    'right',
    'left']
sensors = {}

maxSpeed = 80
driver = Driver()

driver.setSteeringAngle(0.0)  # go straight

timestep = 10

# get and enable the distance sensors
for name in sensorsNames:
    sensors[name] = driver.getDistanceSensor('distance sensor ' + name)
    sensors[name].enable(timestep)

# get and enable the GPS
gps = driver.getGPS('gps')
gps.enable(timestep)

# get the camera
camera = driver.getCamera('camera')
# uncomment those lines to enable the camera
camera.enable(50)
camera.recognitionEnable(50)

# LANE_POSITIONS = [5.0, 2.0, -1.0]
LANE_LEFT = 0
LANE_MID = 1
LANE_RIGHT = 2
LANE_WIDTH = 3.0

step = 0
def debug(msg):
    if step % 50 == 0:
        print(msg + '\n\n')

class ReactiveController:
    def __init__(self, driver, gps, sensors):
        self.driver = driver
        self.gps = gps
#        self.target_lane = LANE_RIGHT
        self.sensors = sensors

        self.target_lane = LANE_RIGHT
        self.target_lane_pos = -1.0

        self.x = self.get_x()

        self.angle_p = 0.02
        self.angle_d = 2.0

        self.target_left_dist = self.sensors.max_left * 0.15
        self.left_dist = 0
        self.left_angle_p = -0.0004
        self.left_angle_d = -1.5

        self.downcounter = 0

    def change_lane_left(self):
        if self.target_lane > LANE_LEFT:
            self.target_lane -= 1
            self.target_lane_pos = (self.target_lane_pos + LANE_WIDTH)
            print('Turn left, target: ', self.target_lane_pos)

            self.downcounter = 250

    def change_lane_right(self):
        if self.target_lane < LANE_RIGHT:
            self.target_lane += 1
            self.target_lane_pos = (self.get_x() - LANE_WIDTH)
            print('Turn right, target: ', self.target_lane_pos)

            self.downcounter = 250

    def update_angle(self):
        # TODO: LANE_LEFT PD adjusting target_lane_pos (that should do!)

        if self.downcounter == 0 and self.target_lane == LANE_LEFT:
            left_dist = self.get_left_dist()
            p_dif = left_dist - self.target_left_dist
            d_dif = left_dist - self.left_dist

            steer = self.left_angle_p * p_dif + self.left_angle_d * d_dif
            cur_steer_angle = self.driver.getSteeringAngle()

#            if self.sensors.front_left_read():
#                cur_steer_angle = -cur_steer_angle

            new_angle = max(min(steer + cur_steer_angle, 0.12), -0.12)
#            debug('STEER: ' + str(steer) + ' NEW ANGLE: ' + str(new_angle))
            self.driver.setSteeringAngle(new_angle)

#            print(steer, new_angle)
#            time.sleep(0.2)

            self.left_dist = left_dist # prev = cur
        else:
            cur_x = self.get_x()
            p_dif = cur_x - self.target_lane_pos
            d_dif = cur_x - self.x 

            steer = self.angle_p * p_dif + self.angle_d * d_dif
            cur_steer_angle = self.driver.getSteeringAngle()

            new_angle = max(min(steer + cur_steer_angle, 0.1), -0.1)

            self.driver.setSteeringAngle(new_angle)
            self.x = cur_x # prev = cur

        self.downcounter = max(0, self.downcounter - 1)

    def update_speed(self):
        frontDistance = self.sensors.front.getValue()
        frontRange = self.sensors.front.getMaxValue()

        speed = min(maxSpeed, maxSpeed * frontDistance / (frontRange * 0.8))
        self.driver.setCruisingSpeed(speed)

        if frontDistance < frontRange * 0.2: # very close. Full break, hope for no collision
            self.driver.setBrakeIntensity(1)

        # brake if we need to reduce the speed
        speedDiff = self.driver.getCurrentSpeed() - speed
        if speedDiff > 0:
            self.driver.setBrakeIntensity(min(speedDiff / speed, 1))
        else:
            self.driver.setBrakeIntensity(0)

    def get_x(self):
        return self.gps.getValues()[0]

    def get_left_dist(self):
        return self.sensors.left.getValue()


class SensorManager:
    def __init__(self, sensors):
        self.front = sensors['front']
        self.left = sensors['left']
        self.right = sensors['right']

        self.front_left_0 = sensors['front left 0']
        self.front_right_0 = sensors['front right 0']
        self.front_left_1 = sensors['front left 1']
        self.front_right_1 = sensors['front right 1']
        self.front_left_2 = sensors['front left 2']
        self.front_right_2 = sensors['front right 2']

#        self.rear_left = sensors['rear left']
#        self.rear_right = sensors['rear right']

        self.max_front = self.front.getMaxValue()
        self.max_left = self.left.getMaxValue()
        self.max_right = self.right.getMaxValue()
        self.max_front_left_0 = self.front_left_0.getMaxValue()
        self.max_front_right_0 = self.front_right_0.getMaxValue()
        self.max_front_left_1 = self.front_left_1.getMaxValue()
        self.max_front_right_1 = self.front_right_1.getMaxValue()
        self.max_front_left_2 = self.front_left_2.getMaxValue()
        self.max_front_right_2 = self.front_right_2.getMaxValue()

#        self.max_rear_left = self.rear_left.getMaxValue()
#        self.max_rear_right = self.rear_right.getMaxValue()


    def front_clear(self):
        return self.front.getValue() > (self.max_front * 0.6)

    def can_turn_left(self):
        return self.left.getValue() > (self.max_left / 2) \
                and (self.front_left_0.getValue() < (self.max_front_left_0 * 0.6) \
                or self.front_left_0.getValue() > (self.max_front_left_0 * 0.8)) \
                and self.front_left_1.getValue() > (self.max_front_left_1 * 0.8) \
                and self.front_left_2.getValue() > (self.max_front_left_2 * 0.8) \

    def can_turn_right(self):
        return self.right.getValue() > (self.max_right / 2) \
                and (self.front_right_0.getValue() < (self.max_front_right_0 * 0.6) \
                or self.front_right_0.getValue() > (self.max_front_right_0 * 0.8)) \
                and self.front_right_1.getValue() > (self.max_front_right_1 * 0.8) \
                and self.front_right_2.getValue() > (self.max_front_right_2 * 0.8) \

    def front_left_read(self):
        return self.front.getValue() < self.max_front \
                or self.front_left_0.getValue() < self.max_front_left_0 \
                or self.front_left_1.getValue() < self.max_front_left_1 \
                or self.front_left_2.getValue() < self.max_front_left_2


class DecisiveController:
    def __init__(self, reactive_controller, sensor_manager):
        self.controller = reactive_controller
        self.sensors = sensor_manager
        self.turning = False

        self.clear_count = 0
        self.clear_threshold = 10

    def act(self):
        if self.sensors.front_clear():
#            debug('Front clear, target_lane: ' + str(self.controller.target_lane))
            self.turning = False
            if self.controller.target_lane == LANE_LEFT:
                self.clear_count = 0
                return


        if not self.turning and self.sensors.can_turn_left():
            self.clear_count += 1

            if self.clear_count > self.clear_threshold:
                self.turning = True 
                self.clear_count = 0
                self.controller.change_lane_left()

        elif not self.sensors.front_clear() and self.turning == 0 and self.sensors.can_turn_right():
            self.clear_count += 1

            if self.clear_count > self.clear_threshold:
                self.turning = True 
                self.clear_count = 0
                self.controller.change_lane_right()
        else:
            self.clear_count = 0


sensor_manager = SensorManager(sensors)
reactive_controller = ReactiveController(driver, gps, sensor_manager)
decisive_controller = DecisiveController(reactive_controller, sensor_manager)


speed = maxSpeed
driver.setCruisingSpeed(speed)

for _ in range(250):
    driver.step()


while driver.step() != -1:
    # adjust the speed according to the value returned by the front distance sensor

    reactive_controller.update_angle()
    reactive_controller.update_speed()
    decisive_controller.act()

    step += 1

#    time.sleep(0.01)

