from typing import Tuple, Optional
from src.core.Auto.IntersectionControl import IntersectionControl
from src.core.Auto.Crosswalk import CrosswalkController
from src.core.Auto.TrafficSignController import TrafficSignController

class IntersectionCrosswalkController:
    """
    Controller that handles combined intersection and crosswalk scenarios.
    State machine transitions:
    finish -> intersection -> crosswalk -> intersection_finish -> finish
    """

    def __init__(self, intersection_controller: IntersectionControl, crosswalk_controller: CrosswalkController) -> None:
        self.intersection = intersection_controller
        self.crosswalk = crosswalk_controller
        self.state: str = "finish" 

    def setCourse(self, sign: str, direction: str, traffic_light_present: bool) -> bool:
        """Initialize the controller with intersection parameters"""
        if self.state != "finish":
            raise RuntimeError(f"Cannot set course in current state: {self.state}")
        
        self.intersection.setCourse(sign, direction, traffic_light_present)
        self.state = "intersection"

    def getControlData(self, 
                      stop_line_present: bool, 
                      stop_line_slope: float, 
                      trafficLights: TrafficSignController,
                      stephanie_position: Optional[Tuple[float, float, float, float]],
                      crosswalk_sign_present: bool) -> Tuple[float, float, bool]:
        """
        Main control loop that manages state transitions and delegates control
        
        Returns:
            Tuple[float, float, bool]: (steering_angle, speed, is_running)
        """
        
        if self.state == "intersection":
            angle, speed, running = self.intersection.getControlData(
                stop_line_present, stop_line_slope, trafficLights
            )
            
            # Check for crosswalk during intersection
            if stop_line_present and crosswalk_sign_present:
                self.state = "crosswalk"
                angle, speed = angle, 0

            if not running:
                self.state = "finish"

        elif self.state == "crosswalk":
            angle, speed, finished = self.crosswalk.control(stephanie_position, forward_enabled=True)
            
            if finished:
                self.state = "finish"

        # elif self.state == "intersection_finish":
        #     angle, speed, running = self.intersection.getControlData(
        #         stop_line_present, stop_line_slope, trafficLights
        #     )
            
        #     if not running:
        #         self.state = "finish"

        elif self.state == "finish":
            raise RuntimeError("Cannot get control data in finish state")
        
        return angle, speed, self.state != "finish"