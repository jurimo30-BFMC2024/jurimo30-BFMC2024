import time
from src.core.Auto.Parking.MotionScheduler import MotionScheduler

class Parking():
    """This thread handles Parking.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        self.state = "finish"
        self.angle = 0
        self.speed = 0

        self.left_spot_taken = False
        self.right_spot_taken = False
        self.parking_spot = "right"

        self.motions = {
            "park": {
                "left": [
                    
                ],
                "right": [

                ],
            },
            "unpark": {
                "left": [

                ],
                "right": [
                    
                ],
            },
            "wait": [
                (0, 0, 3) # wait parked for 3 seconds
            ]
        }

        self.motionScheduler = MotionScheduler()

    def run(self, parkingSpotDetected, sideSensors):

        if self.state == "finish":
            self.state = "search_parking_spot"
            self.angle, self.speed = None, 100

        elif self.state == "search_parking_spot":
            if parkingSpotDetected is not None:
                self.left_spot_taken = False
                self.right_spot_taken = False
                self.state = "search_and_evaluate"

        elif self.state == "search_and_evaluate":
            if self.evaluate_side_sensors(sideSensors):
                self.state = "search_parking_spot"
            elif parkingSpotDetected is not None:
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

    def evaluate_side_sensors(self, sideSensors):
        # Implement logic to evaluate side sensors
        left_spot = sideSensors["left"] != 0
        right_spot = sideSensors["right"] != 0

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
        (None, {"left": 0, "right": 0}),  # Searching for a parking spot
        (True, {"left": 0, "right": 0}),  # Parking spot detected
        (None, {"left": 1, "right": 0}),  # Left side detected
        (True, {"left": 0, "right": 0}),  # Both sides detected
        (None, {"left": 1, "right": 1}),  # Both sides detected
        (None, {"left": 1, "right": 1}),  # Both sides detected
    ]
    
    running = True
    while running:
    # for parkingSpotDetected, sideSensors in test_cases:
        angle, speed, running = parking_system.run(True, {"left": 0, "right": 0})
        print(f"Angle: {angle}, Speed: {speed}, Running: {running}")
        time.sleep(0.5)