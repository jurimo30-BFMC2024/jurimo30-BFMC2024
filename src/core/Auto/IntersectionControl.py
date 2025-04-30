from src.core.Auto.MotionScheduler import MotionScheduler
from src.core.Auto.TrafficSignController import TrafficSignController
import time
import math

class IntersectionControl():
    def __init__(self, logging, debugging=False):
        self.logging = logging
        self.debugging = debugging

        self.state = "finish"
        self.direction = None
        self.full_stop_required = True
        self.slope_degrees = 0
        self.traffic_light_present = False

        self.motions = {
            "Right": [
                (0, 168, 1.7),
                (240, 168, 5.3),
            ],
            "Left": [
                (0, 168, 3.7),
                (-190, 168, 6.0),
            ],
            "Straight": [
                (0, 168, 9.0)
            ],
            "Wait": [
                (0, 0, 3.0)
            ]
        }

        self.motionScheduler = MotionScheduler()

        self.status = -2
        self.lastPoint = 0
        self.navPoint = 0
        self.smer = "None"
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
    
    def createCorrectedMotion(self):
        straighten_distance = self.calculate_distance_to_straighten(self.slope_degrees)
        straighten_time = (straighten_distance / 30)

        # Determine the angle to correct the direction based on slope sign
        if self.slope_degrees < 0:
            angle = -250
        elif self.slope_degrees > 0:
            angle = 250
        else:
            angle = 0

        # Create the correction motion (angle, speed, duration)
        correction_motion = (angle, 300, straighten_time)

        # Get a copy of the motion sequence for the current direction
        motion = self.motions[self.direction].copy()

        # Adjust the first motion's duration to account for the correction time
        angle, speed, t = motion[0]
        motion[0] = (angle, speed, t - straighten_time)

        # Insert the correction motion at the beginning
        motion.insert(0, correction_motion)

        return motion
    
    def setCourse(self, sign: str, direction: str, traffic_light_present: bool):
        if self.state != "finish":
            raise RuntimeError(f"Previous intersection course not finished: {self.state}")
        
        if direction not in ["Right", "Left", "Straight"]:
            raise ValueError(f"Unsuported direction requested: {direction}")
        
        if sign not in ["priority", "stop"]:
            raise ValueError(f"Unknown sign received: {sign}")
        
        self.full_stop_required = (sign == "stop")
        self.direction = direction
        self.traffic_light_present = traffic_light_present
        
        if self.full_stop_required and self.traffic_light_present:
            raise RuntimeError("Undefined state requested, both stop and traffic light detected")
    

    def getControlData(self, stop_line_present: bool, stop_line_slope: float, trafficLights: TrafficSignController):
        if stop_line_present:
            self.slope_degrees = stop_line_slope

        if self.state == "finish":
            if self.full_stop_required:
                self.motionScheduler.set_schedule(self.motions["Wait"])
                self.state = "stop"
            elif self.traffic_light_present:
                self.state = "traffic_light"
            else:
                self.motionScheduler.set_schedule(self.createCorrectedMotion())
                self.state = "maneuver"

        if self.state == "stop":
            angle, speed, finished = self.motionScheduler.run()
            if finished:
                self.motionScheduler.set_schedule(self.createCorrectedMotion())
                self.state = "maneuver"

        elif self.state == "traffic_light":
            angle, speed = 0, 0
            if trafficLights.get_active() == "green":
                self.motionScheduler.set_schedule(self.createCorrectedMotion())
                self.state = "maneuver"

        elif self.state == "maneuver":
            angle, speed, finished = self.motionScheduler.run()
            if finished:
                self.state = "finish"

        return angle, speed, self.state != "finish"