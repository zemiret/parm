"""Sample Webots controller for highway driving benchmark."""

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

LANE_POSITIONS = [5.0, 2.0, -1.0]
LANE_LEFT = 0
LANE_MID = 1
LANE_RIGHT = 2

class ReactiveController:
    def __init__(self, driver, gps, sensor_manager):
        self.driver = driver
        self.gps = gps
        self.target_lane = LANE_RIGHT
        self.lane_positions = LANE_POSITIONS
        self.sensor_manager = sensor_manager

        self.x = self.get_x()

        self.angle_p = 0.03
        self.angle_d = 2.0

    def change_lane_left(self):
        self.target_lane = max(self.target_lane - 1, 0)

    def change_lane_right(self):
        self.target_lane = min(self.target_lane + 1, len(LANE_POSITIONS) - 1)

    def update_angle(self):
        cur_x = self.get_x()
        p_dif = cur_x - self.lane_positions[self.target_lane]
        d_dif = cur_x - self.x 

        steer = self.angle_p * p_dif + self.angle_d * d_dif
        cur_steer_angle = self.driver.getSteeringAngle()

        new_angle = max(min(steer + cur_steer_angle, 0.2), -0.2)

        self.driver.setSteeringAngle(steer + cur_steer_angle)
        self.x = cur_x # prev = cur

    def update_speed(self):
        frontDistance = self.sensor_manager.front.getValue()
        frontRange = self.sensor_manager.front.getMaxValue()

        speed = min(maxSpeed, maxSpeed * frontDistance / (frontRange * 0.5))
        self.driver.setCruisingSpeed(speed)

        if frontDistance < frontRange * 0.15: # very close. Full break, hope for no collision
            self.driver.setBrakeIntensity(1)

        # brake if we need to reduce the speed
        speedDiff = self.driver.getCurrentSpeed() - speed
        if speedDiff > 0:
            self.driver.setBrakeIntensity(min(speedDiff / speed, 1))
        else:
            self.driver.setBrakeIntensity(0)

    def get_lane(self):
        return self.target_lane

    def get_x(self):
        return self.gps.getValues()[0]


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
        return self.front.getValue() > (self.max_front / 2)

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


class DecisiveController:
    def __init__(self, reactive_controller, sensor_manager):
        self.controller = reactive_controller
        self.sensors = sensor_manager
        self.turning = False

        self.clear_count = 0
        self.clear_threshold = 10

    def act(self):
        if self.sensors.front_clear():
            self.clear_count = 0
            self.turning = False
            return

        if not self.turning and self.sensors.can_turn_left():
            self.clear_count += 1

            if self.clear_count > self.clear_threshold:
                self.turning = True 
                self.controller.change_lane_left()

        elif not self.turning and self.sensors.can_turn_right():
            self.clear_count += 1

            if self.clear_count > self.clear_threshold:
                self.turning = True 
                self.controller.change_lane_right()
        else:
            self.clear_count = 0


sensor_manager = SensorManager(sensors)
reactive_controller = ReactiveController(driver, gps, sensor_manager)
decisive_controller = DecisiveController(reactive_controller, sensor_manager)


speed = maxSpeed
driver.setCruisingSpeed(speed)

for _ in range(300):
    driver.step()


while driver.step() != -1:
    # adjust the speed according to the value returned by the front distance sensor

    reactive_controller.update_angle()
    reactive_controller.update_speed()
    decisive_controller.act()

#    time.sleep(0.01)

