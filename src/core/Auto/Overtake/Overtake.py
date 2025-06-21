from src.core.Auto.MotionScheduler import MotionScheduler
import time

class Overtake():
    """This class handles overtaking.
    Args:
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, logging, debugging=False, normal_speed=200, highway_speed=500):
        self.logging = logging
        self.debugging = debugging
        self.normal_speed = normal_speed
        self.highway_speed = highway_speed

        self.angle = 0
        self.speed = 0
        self.state = "finish"
        self.caught_up_at_time = 0
        self.passed_at_time = 0
        self.right_sensor_counter = 0
        
        # Maximum wheel angle for allowing lane return (in degrees)
        self.max_wheel_angle_for_return = 7
        
        self.motionScheduler = MotionScheduler()

        self.motions = {
            "overtake": {
                "move_left": [
                    (-230, self.highway_speed, .4),
                ],
                "move_right": [
                    (230, self.highway_speed, .3),
                ],
            },
            "pass_obstacle": {
                "move_left": [
                    (-230, self.normal_speed, 2.5),
                ],
                "move_right": [
                    (230, self.normal_speed, 3),
                ],
            }
        }

    def run(self, highway, front_sensors, side_sensors, lane_follow_angle=0):
        # print(side_sensors["right"])
        if self.state == "finish":
            self.state = "close_distance"

        if self.state == "close_distance":
            if front_sensors["distance"] <= 60:
                self.state = "change_lane_left"
                print(f'Overtake [{"overtake" if highway else "pass"}]{self.state}')
                self.motionScheduler.set_schedule(self.motions["overtake" if highway else "pass_obstacle"]["move_left"])
        
        elif self.state == "change_lane_left":
            self.angle, self.speed, finished = self.motionScheduler.run()
            # self.angle += angle
            if finished:
                self.state = "catch_up"
                self.caught_up_at_time = time.time()  # Start ignore period
                print(f'Overtake [{"overtake" if highway else "pass"}]{self.state}')
                self.angle, self.speed = None, self.highway_speed if highway else self.normal_speed
        
        elif self.state == "catch_up":
            if side_sensors["right"] < 50:
                self.right_sensor_counter += 1
            else:
                self.right_sensor_counter = 0

            if self.right_sensor_counter >= 3 and time.time() - self.caught_up_at_time > 2:
                self.caught_up_at_time = time.time()
                self.state = "pass"
                self.right_sensor_counter = 0
                print(f'Overtake [{"overtake" if highway else "pass"}]{self.state}')

        elif self.state == "pass":
            if side_sensors["right"] > 50:
                self.right_sensor_counter += 1
            else:
                self.right_sensor_counter = 0

            if self.right_sensor_counter >= 3:
                self.passed_at_time = time.time()
                self.state = "get_distance"
                self.right_sensor_counter = 0
                print(f'Overtake [{"overtake" if highway else "pass"}]{self.state}')

        elif self.state == "get_distance":
            if side_sensors["right"] < 50:
                self.right_sensor_counter += 1
            else:
                self.right_sensor_counter = 0

            if self.right_sensor_counter >= 3:
                self.state = "pass"
                self.right_sensor_counter = 0
                print(f'Overtake [{"overtake" if highway else "pass"}]{self.state}')
            elif time.time() - self.passed_at_time > (self.passed_at_time - self.caught_up_at_time) // 2:
                # Check if wheel angle is acceptable for lane return
                if abs(lane_follow_angle) <= self.max_wheel_angle_for_return * 10:  # lane_follow_angle is in tenths of degrees
                    self.state = "change_lane_right"
                    print(f'Overtake [{"overtake" if highway else "pass"}]{self.state}')
                    self.motionScheduler.set_schedule(self.motions["overtake" if highway else "pass_obstacle"]["move_right"])
                else:
                    if self.debugging:
                        print(f'Lane return delayed - wheel angle too high: {abs(lane_follow_angle)/10}°')

        elif self.state == "change_lane_right":
            self.angle, self.speed, finished = self.motionScheduler.run()
            # self.angle += angle
            if finished:
                self.state = "finish"
                print(f'Overtake [{"overtake" if highway else "pass"}]{self.state}')
                self.angle, self.speed = None, self.highway_speed if highway else self.normal_speed

        return self.angle, self.speed, self.state != "finish"