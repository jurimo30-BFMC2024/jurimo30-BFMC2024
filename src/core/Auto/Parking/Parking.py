

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


    def run(self, parkingSpotDetected, sideSensors):

        if self.state == "finish":
            self.state = "search_parking_spot"
            self.angle, self.speed = 0, 0

        elif self.state == "search_parking_spot":
            if parkingSpotDetected is not None:
                self.left_spot_taken = False
                self.right_spot_taken = False
                self.state = "search_and_evaluate"

        elif self.state == "search_and_evaluate":
            if self.evaluate_side_sensors():
                self.state = "search_parking_spot"
            elif parkingSpotDetected is not None:
                self.state = "park"

        elif self.state == "park":
            self.perform_parking()
            self.state = "unpark"

        elif self.state == "unpark":
            self.perform_unparking()
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

    def perform_parking(self):
        self.logging.info("Performing parking maneuver...")
        # Implement parking logic

    def perform_unparking(self):
        self.logging.info("Performing unparking maneuver...")
        # Implement unparking logic
