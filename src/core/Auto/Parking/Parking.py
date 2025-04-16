import time
from src.core.Auto.Parking.MotionScheduler import MotionScheduler

class Parking():
    """This thread handles Parking.
    Args:
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, logging, debugging=False):
        self.logging = logging
        self.debugging = debugging

        self.state = "finish"
        self.angle = 0
        self.speed = 0

        self.left_spot_taken = False
        self.right_spot_taken = False
        self.parking_spot = "right"

        self.last_parking_spot_detected = False
        self.last_detection_time = 0
        self.start_evaluate = 0

        self.motions = {
            "park": {
                "left": [
                    (None, 300, 2.3),
                    (-230, -300, 1.7),
                    (230, -300, 2),
                ],
                "right": [
                    (None, 300, 2.3),
                    (230, -300, 1.7),
                    (-230, -300, 2),
                ],
            },
            "unpark": {
                "left": [
                    (0, -300, .4),
                    (230, 300, 1.5),
                    (-230, 300, 1.5),
                ],
                "right": [
                    (0, -300, .4),
                    (-230, 300, 1.5),
                    (230, 300, 1.5),
                ],
            },
            "wait": [
                (0, 0, 3) # wait parked for 3 seconds
            ]
        }

        self.motionScheduler = MotionScheduler()

    def detect_parking_spot(self, parking_spot_detected: bool) -> bool:
        current_time = time.time()
        
        if parking_spot_detected and not self.last_parking_spot_detected:
            self.last_detection_time = current_time
            self.last_parking_spot_detected = True
            return True

        if not parking_spot_detected:
            if current_time - self.last_detection_time >= 0.5:
                self.last_parking_spot_detected = False

        return False

    def run(self, parking_spot_detected, side_sensors):
        parking_spot_detected = self.detect_parking_spot(parking_spot_detected)

        if self.state == "finish":
            self.state = "search_parking_spot"
            self.angle, self.speed = None, 200

        elif self.state == "search_parking_spot":
            if parking_spot_detected:
                self.left_spot_taken = False
                self.right_spot_taken = False
                self.state = "search_and_evaluate"
                self.start_evaluate = time.time()

        elif self.state == "search_and_evaluate":
            if time.time() - self.start_evaluate > 1:
                if self.evaluate_side_sensors(side_sensors):
                    self.state = "search_parking_spot"
                elif parking_spot_detected:
                    if not self.right_spot_taken:
                        self.parking_spot = "right"
                    else:
                        self.parking_spot = "left"

                    self.logging.info(f"Performing parking maneuver in the {self.parking_spot} spot")
                    self.state = "park"
                    self.motionScheduler.set_schedule(self.motions[self.state][self.parking_spot])
            
        elif self.state == "park":
            self.angle, self.speed, finished = self.motionScheduler.run()
            if finished:
                self.state = "wait"
                self.motionScheduler.set_schedule(self.motions[self.state])

        elif self.state == "wait":
            self.angle, self.speed, finished = self.motionScheduler.run()
            if finished:
                self.logging.info(f"Performing unparking maneuver from the {self.parking_spot} spot")
                self.state = "unpark"
                self.motionScheduler.set_schedule(self.motions[self.state][self.parking_spot])


        elif self.state == "unpark":
            self.angle, self.speed, finished = self.motionScheduler.run()
            if finished:
                self.state = "finish"

        self.logging.debug(f"Current state: {self.state}")

        return self.angle, self.speed, self.state != "finish"

    def evaluate_side_sensors(self, side_sensors):
        # Implement logic to evaluate side sensors
        left_spot = side_sensors["left"] < 40.0
        right_spot = side_sensors["right"] < 40.0

        self.left_spot_taken = self.left_spot_taken or left_spot
        self.right_spot_taken = self.right_spot_taken or right_spot

        return self.left_spot_taken and self.right_spot_taken

if __name__ == "__main__":
    import logging
    # Setup logging
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger("ParkingTest")
    
    # Mock queue list (not used in this test)
    queueList = {}
    
    # Initialize the Parking class
    parking_system = Parking(queueList, logger, debugging=True)
    
    # Mock sensor data
    test_cases = [
        (False, {"left": float('inf'), "right": float('inf')}),
        (True, {"left": float('inf'), "right": float('inf')}),
        (False, {"left": 10.0, "right": float('inf')}),
        (False, {"left": float('inf'), "right": float('inf')}),
        (True, {"left": float('inf'), "right": float('inf')}),
        (False, {"left": 10.0, "right": 10.0}),
    ]
    
    running = True
    while running:
        for parkingSpotDetected, side_sensors in test_cases:
            angle, speed, running = parking_system.run(parkingSpotDetected, side_sensors)
            print(f"Angle: {angle}, Speed: {speed}, Running: {running}")
            if not running:
                break
            time.sleep(0.5)