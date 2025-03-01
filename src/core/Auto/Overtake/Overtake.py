from src.core.Auto.Parking.MotionScheduler import MotionScheduler
import time

class Overtake():
    """This class handles overtaking.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        self.angle = 0
        self.speed = 0
        self.state = "finish"
        self.caught_up_at_time = 0
        self.passed_at_time = 0
        
        self.motionScheduler = MotionScheduler()

        self.motions = {
            "move_left": [
                (-150, 500, 1),
                (150, 500, 1),
            ],
            "move_right": [
                (150, 500, 1),
                (-150, 500, 1),
            ],
        }

    def run(self, angle, front_sensors, side_sensors):
        if self.state == "finish":
            self.state = "close_distance"
            self.angle, self.speed = None, 500

        elif self.state == "close_distance":
            if front_sensors["distance"] <= 60:
                self.state = "change_lane_left"
                self.motionScheduler.set_schedule(self.motions["move_left"])
        
        elif self.state == "change_lane_left":
            self.angle, self.speed, finished = self.motionScheduler.run()
            self.angle += angle
            if finished:
                self.state = "catch_up"
                self.angle, self.speed = None, 500
        
        elif self.state == "catch_up":
            if side_sensors["right"] < 50:
                self.caught_up_at_time = time.time()
                self.state = "pass"

        elif self.state == "pass":
            if side_sensors["right"] > 50:
                self.passed_at_time = time.time()
                self.state = "get_distance"

        elif self.state == "get_distance":
            if side_sensors["right"] < 50:
                self.state = "pass"
            elif time.time() - self.passed_at_time > self.passed_at_time - self.caught_up_at_time: # drive the same amount of time after passing the car to ensure that the we can merge back
                self.state = "change_lane_right"
                self.motionScheduler.set_schedule(self.motions["move_right"])

        elif self.state == "change_lane_right":
            self.angle, self.speed, finished = self.motionScheduler.run()
            self.angle += angle
            if finished:
                self.state = "finish"
                self.angle, self.speed = None, 500

        return self.angle, self.speed, self.state != "finish"