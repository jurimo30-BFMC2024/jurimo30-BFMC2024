import time

class RoundaboutControl():
    def __init__(self, logging, debugging=False):
        self.logging = logging
        self.debugging = debugging
        self.status = -1  # -1: not started, 0: moving forward, 1: turning right, 2: adjusting angle
        self.lastPoint = 0
        self.angle = 0
        self.speed = 0

    def getControlData(self, angleForRoundabout, navigate, exitFlag):
        roundAbout = True

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

        elif self.status == 1:  # Turning right
            if (time.time() - self.lastPoint) >= 3:  # Fixed right turn duration
                if self.debugging:
                    print("Adjusting angle for roundabout")
                self.status = 2
                self.lastPoint = time.time()
                self.angle = int(angleForRoundabout*10) 
                print(self.angle)
                self.speed = 150

        elif self.status == 2:  # Adjusting 
            self.angle = int(angleForRoundabout*10) 
            print(self.angle)
            self.speed = 150
            if exitFlag and not getattr(self, 'exitHandled', False):  # React only on first True
                if self.debugging:
                    print("Exit detected")
                if navigate:  # Ensure navigate is not empty
                    direction = navigate.pop(0)  # Get and remove the first direction
                    if direction == "Straight":
                        if self.debugging:
                            print("Continuing in roundabout")
                        self.status = 2  # Continue adjusting angle
                    elif direction == "Right":
                        if self.debugging:
                            print("Exiting roundabout to the right")
                        self.status = 3  # Transition to fixed right turn for exit
                        self.lastPoint = time.time()
                        self.angle = 230
                        self.speed = 168
                else:
                    if self.debugging:
                        print("No directions left in navigate")
                self.exitHandled = True  # Mark exit as handled
            elif not exitFlag:
                self.exitHandled = False  # Reset when exitFlag returns to False
                if self.debugging:
                    print("Waiting for exit flag")

        elif self.status == 3:  # Exiting roundabout
            if (time.time() - self.lastPoint) >= 1.5:  # Fixed right turn duration for exit
                if self.debugging:
                    print("Exiting roundabout completely")
                roundAbout = False  # Exit roundabout
                self.status = -1  # Reset to initial state
                self.angle = 0
                self.speed = 0

        return self.angle, self.speed, roundAbout
