import time

class RoundaboutControl():
    def __init__(self, logging, debugging=False):
        self.logging = logging
        self.debugging = debugging
        self.status = -1  # -1: not started, 0: moving forward, 1: turning right, 2: adjusting angle
        self.lastPoint = 0
        self.angle = 0
        self.speed = 0
        self.exiting = False
        self.exitFlag = False  # Initialize exitFlag

    def getControlData(self, angleForRoundabout, navigate, exitFlag):
        roundAbout = True
        self.exitFlag = exitFlag  # Update instance attribute instead of local variable

        if self.status == -1:  # Initial state
            if self.debugging:
                print("Starting roundabout maneuver")
            self.lastPoint = time.time()
            self.status = 0
            self.angle = 0
            self.speed = 150  # Fixed forward speed

        elif self.status == 0:  # Moving forward
            if (time.time() - self.lastPoint) >= 1.8:  # Fixed forward duration
                if self.debugging:
                    print("Switching to right turn")
                self.lastPoint = time.time()
                self.status = 1
                self.angle = 240
                self.speed = 150  # Fixed right turn speed
                self.exitFlag = False  # Update instance attribute instead of local variable

        elif self.status == 1:  # Turning right
            if (time.time() - self.lastPoint) >= 3:  # Fixed right turn duration
                if self.debugging:
                    print("Adjusting angle for roundabout")
                self.status = 2
                self.lastPoint = time.time()
                self.angle = int(angleForRoundabout*10) 
                self.speed = 150

        elif self.status == 2:  # Adjusting 
            self.angle = int(angleForRoundabout*10) 
            self.speed = 150
            if self.exitFlag:  # Use instance attribute
                print("Modul detektovao izlaz")
                if navigate.pop(0) == "Right":
                    print("Izlazim iz kruznog toka")
                    self.angle = 250
                    self.speed = 150
                    self.status = 3
                    self.lastPoint = time.time()
                else:
                    self.status = 2
                    print("Ostajem u kruznom toku")
                self.exitFlag = False  # Reset instance attribute

        elif self.status == 3:  # Exiting roundabout
            if (time.time() - self.lastPoint) >= 3:  # Fixed right turn duration for exit
                if self.debugging:
                    print("Exiting roundabout completely")
                roundAbout = False  # Exit roundabout
                self.status = -1  # Reset to initial state
                self.angle = 0
                self.speed = 0

        return self.angle, self.speed, roundAbout, self.exitFlag  # Return instance attribute
