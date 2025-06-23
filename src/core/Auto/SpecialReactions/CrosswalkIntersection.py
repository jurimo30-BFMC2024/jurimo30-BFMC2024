from typing import Tuple, Optional
from src.core.Auto.IntersectionControl import IntersectionControl
from src.core.Auto.Crosswalk import CrosswalkController
from src.core.Auto.TrafficSignController import TrafficSignController

class CrosswalkIntersectionController:
    """
    Controller that handles combined crosswalk and intersection scenarios.
    State machine transitions:
    finish -> crosswalk -> intersection -> finish
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
        self.state = "crosswalk"

    def getControlData(self, 
                      stop_line_present: bool, 
                      stop_line_slope: float, 
                      trafficLights: TrafficSignController,
                      stephanie_position: Optional[Tuple[float, float, float, float]]) -> Tuple[float, float, bool]:
        """
        Main control loop that manages state transitions and delegates control
        
        Returns:
            Tuple[float, float, bool]: (steering_angle, speed, is_running)
        """
        
        if self.state == "crosswalk":
            angle, speed, finished = self.crosswalk.control(stephanie_position, forward_enabled=False)
            
            if finished:
                self.state = "intersection"

        elif self.state == "intersection":
            angle, speed, running = self.intersection.getControlData(
                stop_line_present, stop_line_slope, trafficLights
            )
            
            if not running:
                self.state = "finish"

        elif self.state == "finish":
            raise RuntimeError("Cannot get control data in finish state")

        return angle, speed, self.state != "finish"
