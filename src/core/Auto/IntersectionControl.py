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
                (240, 168, 6.1),
            ],
            "Left": [
                (0, 168, 3.7),
                (-190, 168, 6.3),
            ],
            "Straight": [
                (20, 168, 9.7)
            ],
            "Wait": [
                (0, 0, 3.0)
            ]
        }

        self.motionScheduler = MotionScheduler()
        self.approach_counter = 0
        
        # Traffic light timeout handling
        self.traffic_light_wait_start = None
        self.traffic_light_timeout = 3.0  # 3 seconds timeout
    
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
        # Konvertujemo uglole u radijane
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
        straighten_time = (straighten_distance / 16.8)

        # Determine the angle to correct the direction based on slope sign
        if self.slope_degrees < 0:
            angle = -250
        elif self.slope_degrees > 0:
            angle = 250
        else:
            angle = 0

        # Create the correction motion (angle, speed, duration)
        correction_motion = (angle, 200, straighten_time)

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
        
        if not traffic_light_present and sign not in ["priority", "stop"]:
            raise ValueError(f"Unknown sign received: {sign}")
        
        self.full_stop_required = (sign == "stop")
        self.direction = direction
        self.traffic_light_present = traffic_light_present
        
        if self.full_stop_required and self.traffic_light_present:
            raise RuntimeError("Ambiguous state: both STOP and traffic light detected")

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
            
            # Start timing when we first enter traffic_light state
            if self.traffic_light_wait_start is None:
                self.traffic_light_wait_start = time.time()
            
            current_time = time.time()
            wait_duration = current_time - self.traffic_light_wait_start
            
            # Check if green light or timeout exceeded
            if trafficLights.get_active() == "green" or wait_duration >= self.traffic_light_timeout:
                if wait_duration >= self.traffic_light_timeout:
                    print(f"Traffic light timeout reached ({self.traffic_light_timeout}s), proceeding as green")
                
                self.state = "approach_stop_line"
                self.approach_counter = 0
                self.traffic_light_wait_start = None  # Reset timer

        elif self.state == "approach_stop_line":
            angle, speed = 0, 200  # go forward until stop line is detected
            if stop_line_present:
                self.approach_counter += 1
                if self.approach_counter >= 5:
                    self.motionScheduler.set_schedule(self.createCorrectedMotion())
                    self.state = "maneuver"

        elif self.state == "maneuver":
            angle, speed, finished = self.motionScheduler.run()
            if finished:
                self.state = "finish"

        return angle, speed, self.state != "finish"