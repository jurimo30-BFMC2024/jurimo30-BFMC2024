import time
import math

class RoundaboutControl():
    def __init__(self, logging, debugging=False):
        self.logging = logging
        self.debugging = debugging
        self.status = -3  # -2: not started, -1: fix the car angle, 0: moving forward, 1: turning right, 2: adjusting angle, 3: exiting
        self.lastPoint = 0
        self.angle = 0
        self.speed = 0
        self.exiting = False
        self.exitFlag = False  # Initialize exitFlag
        self.slope_degrees = 0
        self.straighten_time = 0

    def calculate_distance_to_straighten(self, alpha_deg, wheelbase=26, max_steering_angle=25):
        """
        Izračunava dužinu puta (u cm) koju model auta treba da pređe dok se ne ispravi.
    
        Parametri:
        alpha_deg (float): Ugao između auta i zaustavne linije u stepenima (-90 do 90).
        speed_cm_s (float): Brzina kretanja auta napred u cm/s.
        wheelbase (float): Međuosovinsko rastojanje u cm (default 26 cm).
        max_steering_angle (float): Maksimalni ugao skretanja točkova u stepenima (default 25°).
    
        Returns:
        float: Dužina puta u cm koja je potrebna da se auto ispravi.
        """
        # Konvertujemo uglove u radijane
        alpha_rad = math.radians(alpha_deg)
        steering_angle_rad = math.radians(max_steering_angle)
    
        # Poluprečnik kružne putanje (aproksimacija)
        R = wheelbase / math.tan(steering_angle_rad)
    
        # Dužina luka potrebna da se auto ispravi
        distance = R * abs(alpha_rad)

        #po brzini 168, ovaj manevar ce trajati distance/168 sekundi
        #jbg izracunaj sam dole


        return distance

    def getControlData(self, angleForRoundabout, navigate, exitFlag, stopLine):
        roundAbout = True
        self.exitFlag = exitFlag  # Update instance attribute instead of local variable
        if stopLine[0]:
            self.slope_degrees = stopLine[1]

        if self.status == -3:
            if self.debugging:
                print("Starting roundabout maneuver")
            self.speed = 0
            self.status = -2
            self.angle = 0
            self.lastPoint = time.time()
            
        elif self.status == -2:  # Initial state
            if (time.time() - self.lastPoint) >= 0.2:
                straighten_distance = self.calculate_distance_to_straighten(self.slope_degrees)
                # Assume speed is 300 cm/s — calculate duration
                self.straighten_time = (straighten_distance / 30)
                print("krecem sa ispravljanjem")
                # print(f'straighten_distance: {straighten_distance}, self.straighten_time: {self.straighten_time}')

                if self.slope_degrees < 0:
                    self.angle = -250
                elif self.slope_degrees > 0:
                    self.angle = 250
                else:
                    self.angle = 0
                self.speed = 300
                self.lastPoint = time.time()
                self.status = -1
                # print("angle, speed, slope", self.angle, self.speed, self.slope_degrees)

        elif self.status == -1: # fix the car angle
            if (time.time() - self.lastPoint) >= self.straighten_time:
                if self.debugging:
                    print("Starting roundabout maneuver speed")
                self.lastPoint = time.time()
                self.status = 0
                self.angle = 0
                self.speed = 150  # Fixed forward speed

        elif self.status == 0:  # Moving forward
            if (time.time() - self.lastPoint) >= 1.2 - self.straighten_time:  # Fixed forward duration
                if self.debugging:
                    print("Switching to right turn")
                self.lastPoint = time.time()
                self.status = 1
                self.angle = 250
                self.speed = 150  # Fixed right turn speed
                self.exitFlag = False  # Update instance attribute instead of local variable

        elif self.status == 1:  # Turning right
            if (time.time() - self.lastPoint) >= 3.4:  # Fixed right turn duration
                if self.debugging:
                    print("Adjusting angle for roundabout")
                self.status = 2
                self.lastPoint = time.time()
                self.angle = int(angleForRoundabout*10) 
                self.speed = 200

        elif self.status == 2:  # Adjusting 
            self.angle = int(angleForRoundabout*10) 
            self.speed = 200
            if self.exitFlag:  # Use instance attribute
                print("Modul detektovao izlaz")
                if navigate.pop(0) == "Right":
                    print("Izlazim iz kruznog toka")
                    self.angle = 245
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
                self.status = -3  # Reset to initial state
                self.angle = 0
                self.speed = 0

        return self.angle, self.speed, roundAbout, self.exitFlag  # Return instance attribute
