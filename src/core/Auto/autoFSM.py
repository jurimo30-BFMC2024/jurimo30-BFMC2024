if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.core.Auto.Parking.Parking import Parking
from src.core.Auto.Overtake.Overtake import Overtake
from src.core.Core.ControlModeThread.ControlModeThread import ControlModeThread
from src.core.Auto.LaneFollow.LaneFollow import LaneFollow
from src.core.Auto.SpeedControl import SpeedControl
from src.core.Auto.IntersectionControl import IntersectionControl
from src.core.Auto.RoundaboutControl import RoundaboutControl
from src.utils.messages.allMessages import (
    LaneDetect,
    CoreSteerMotor,
    CoreSpeedMotor,
    IntersectionDetect,
    IntersectionDetect2,
    ObjectDetection,
    SideSensors,
    FrontSensors,
    ParkingSpotDetect,
    RoundAboutAngle
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.core.Auto.pathPlanning.pathPlanning import PathPlanner
from src.core.Auto.TrafficSignController import TrafficSignController
import time
from enum import Enum, auto

"""
    - stopline staviti u jedan stopline subscriber
"""

class autoFSMState(Enum):
    DRIVE = auto()
    OVERTAKE = auto()
    PARKING = auto()
    INTERSECTION = auto()
    ROUNDABOUT = auto()
    CROSSWALK = auto()
    HIGHWAY = auto()

class autoFSM(ControlModeThread):
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        
        self.steerMotorSender = messageHandlerSender(self.queuesList, CoreSteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, CoreSpeedMotor)

        self.subscribe()
        super().__init__()

    def start(self):
        self.planer = PathPlanner(start=43, goal=10, mode="pacman")
        self.laneFollowData = LaneFollow(self.logging, False)
        self.speedControler = SpeedControl(self.logging, False)
        self.intersectionController = IntersectionControl(self.logging, self.debugging)
        self.parkingController = Parking(self.logging, self.debugging)
        self.overtakeController = Overtake(self.logging, self.debugging)
        self.roundaboutController = RoundaboutControl(self.logging, self.debugging)

        self.laneDetectSubscriber.empty()
        self.intersectionDetectSubscriber.empty()
        self.intersectionDetectSubscriber2.empty()
        self.objectDetectionSubscriber.empty()
        self.sideSensorSubscriber.empty()
        self.frontSensorSubscriber.empty()
        self.parkingSpotDetectionSubscriber.empty()
        self.roundaboutAngleSubscriber.empty()

        self.oldAngle = 0
        self.oldSpeed = 0
        self.steerMotorSender.send("0")
        self.speedMotorSender.send("0")
        #self.navigateCommand = self.planer.planPath()
        self.navigateCommand = ["Straight", "Straight", "Straight", "Right"]

        print(self.navigateCommand)
        self.traffic_signs = TrafficSignController([
            "stop", "crosswalk", "highway_entrance", "highway_exit",
            "one_way", "no_entry", "parking", "priority",
            "round_about", "round_about2"
        ])

        self.traffic_light_states = TrafficSignController([
            "red", "green", "yellow", "red_yellow"
        ])

        self.sign_car_detected = False
        self.obstacle_start_time = None
        self.stephanie = False
        self.roundaboutExitFlag = False
        self.intersectionSign = "None"

        self.state = autoFSMState.DRIVE

        super().start()
    
    def stop(self):
        super().stop()

    def loop(self):
        error_angle = self.laneDetectSubscriber.receiveWithBlock()
        stop_line_present, stop_line_slope = self.intersectionDetectSubscriber.receiveWithBlock() # stopLine je sad tuple (intersection(bool), slope_degrees (float))
        lowDistance = self.intersectionDetectSubscriber2.receiveWithBlock()

        while self.objectDetectionSubscriber.isDataInPipe():
            object = self.objectDetectionSubscriber.receive()

            if object == "stefanija":
                self.stephanie = True
            elif object == "exit":
                self.roundaboutExitFlag = True
            elif object == "car":
                self.sign_car_detected = True
            elif object in self.traffic_light_states:
                self.traffic_light_states.set_active(object)
            elif object in self.traffic_signs:
                self.traffic_signs.set_active(object)
            else:
                raise ValueError(f'Unknown object detected: {object}')
          
            print(f"Preuzet je objekat {object}")

        parking_spot_detected = self.parkingSpotDetectionSubscriber.receive() != None

        #ulaz obrade sa ESP
        front_sensors = self.frontSensorSubscriber.receiveWithBlock()
        side_sensors = self.sideSensorSubscriber.receiveWithBlock()

        # Receive roundabout-related messages
        roundabout_angle = self.roundaboutAngleSubscriber.receiveWithBlock()  # Corrected to RoundAboutAngle
        traffic_light_present = self.traffic_light_states.get_active() != None
        obstacle = front_sensors["distance"] <= 80 and self.sign_car_detected
        angle = self.laneFollowData.getControlData(self.state == autoFSMState.HIGHWAY, lowDistance, error_angle) # calculate steering angle from lane follow data

        if not obstacle:
            self.obstacle_start_time = None  # Reset if obstacle is not present

        if not self._running.is_set():
            return
        
        ##############################         SET STATES            #############################

        
        if self.state == autoFSMState.HIGHWAY:
            if self.traffic_signs.get_active() == "highway_exit" or stop_line_present or lowDistance:
                if self.debugging:
                    print("Izlazak sa auto puta")
                self.traffic_signs.clear()
                self.state = autoFSMState.DRIVE
        
        if self.state == autoFSMState.DRIVE:
            if self.traffic_signs.get_active() == "parking":
                self.traffic_signs.clear()
                self.state = autoFSMState.PARKING
            
            elif stop_line_present and (self.traffic_signs.get_active() in ["stop", "priority"] or traffic_light_present):
                if self.debugging:
                    print("Krecemo sa raskrsnicom")
                if self.traffic_signs.get_active() == "stop":
                    self.intersectionSign = "stop"
                if self.traffic_signs.get_active() == "priority":
                    self.intersectionSign = "priority"
                if traffic_light_present:
                    self.intersectionSign = "None"
                self.traffic_signs.clear()
                self.state = autoFSMState.INTERSECTION

            elif stop_line_present and self.traffic_signs.get_active() in ["round_about", "round_about2"]:
                if self.debugging:
                    print("Entering roundabout")
                self.traffic_signs.clear()
                self.state = autoFSMState.ROUNDABOUT

            elif stop_line_present and self.traffic_signs.get_active() == "crosswalk" and self.stephanie:
                self.crosswalkStart = time.time()
                self.state = autoFSMState.CROSSWALK
                        
            elif self.traffic_signs.get_active() == "highway_entrance":
                if self.debugging:
                    print("Ulazak na autoput")
                self.traffic_signs.clear()
                self.state = autoFSMState.HIGHWAY
            
            elif obstacle and self.oldSpeed == 0:
                if self.obstacle_start_time is None:
                    self.obstacle_start_time = time.time()
                
                if time.time() - self.obstacle_start_time >= 1:
                    print("Pass static obstacle start")
                    self.sign_car_detected = False
                    self.state = autoFSMState.OVERTAKE

        ##############################         FSM            #############################

        if self.state == autoFSMState.PARKING:
            park_angle, speed, finished = self.parkingController.run(parking_spot_detected, side_sensors)
            if park_angle is not None:
                angle = park_angle

            if finished:
                self.state = autoFSMState.DRIVE
            
        elif self.state == autoFSMState.INTERSECTION:
            angle, speed, finished = self.intersectionController.getControlData(
                navigate=self.navigateCommand,
                sign=self.intersectionSign,
                trafficLights=self.traffic_light_states,
                trafficLightFlag=traffic_light_present,
                stop_line_present=stop_line_present,
                stop_line_slope=stop_line_slope
            )
            
            if finished:
                self.state = autoFSMState.DRIVE

        elif self.state == autoFSMState.OVERTAKE:
            overtake_angle, speed, finished = self.overtakeController.run(False, front_sensors, side_sensors)
            if overtake_angle is not None:
                angle = overtake_angle

            if finished:
                self.state = autoFSMState.DRIVE

        elif self.state == autoFSMState.CROSSWALK:
            angle = 0
            speed = 0
            if time.time() - self.crosswalkStart >= 3:
                self.stephanie = False
                self.traffic_signs.clear()
                self.state = autoFSMState.DRIVE

        elif self.state == autoFSMState.ROUNDABOUT:
            angle, speed, finished, self.roundaboutExitFlag  = self.roundaboutController.getControlData(
                angleForRoundabout=roundabout_angle,  # Use the received angle
                navigate=self.navigateCommand,
                exitFlag=self.roundaboutExitFlag,
                stop_line_present=stop_line_present,
                stop_line_slope=stop_line_slope
            )

            if finished:
                self.state = autoFSMState.DRIVE

        elif self.state == autoFSMState.DRIVE or self.state == autoFSMState.HIGHWAY:
            no_active_sign = self.traffic_signs.get_active() is None and self.traffic_light_states.get_active() is None
            stephanie_crossing = self.stephanie and self.traffic_signs.get_active() != "crosswalk"

            speed = self.speedControler.getControlData(
                angle=angle,
                stopLine=stop_line_present,
                lowDistance=lowDistance,
                highway=self.state == autoFSMState.HIGHWAY,
                frontDistance=front_sensors["distance"],
                enable_emergency_stop=no_active_sign,
                car_in_front=self.sign_car_detected,
                stephanie_in_front=stephanie_crossing
            )

        ############################ Sending data ##############################

        if angle != self.oldAngle:
            self.steerMotorSender.send(f"{angle}")
            self.oldAngle = angle
            # if self.debugging:
            #     self.logging.info(f"New steering angle: {angle}")

        if speed != self.oldSpeed:
            self.speedMotorSender.send(f"{speed}")
            self.oldSpeed = speed
            # if self.debugging:
            #     self.logging.info(f"New speed: {speed}")
        
        # time.sleep(0.05)

    def getTime(self):
        return round(time.time()*1000)

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.laneDetectSubscriber = messageHandlerSubscriber(self.queuesList, LaneDetect, "LastOnly", True)
        self.intersectionDetectSubscriber = messageHandlerSubscriber(self.queuesList, IntersectionDetect, "LastOnly", True)
        self.intersectionDetectSubscriber2 = messageHandlerSubscriber(self.queuesList, IntersectionDetect2, "LastOnly", True)
        self.objectDetectionSubscriber = messageHandlerSubscriber(self.queuesList, ObjectDetection, "FIFO", True)
        self.sideSensorSubscriber = messageHandlerSubscriber(self.queuesList, SideSensors, "LastOnly", True)
        self.frontSensorSubscriber = messageHandlerSubscriber(self.queuesList, FrontSensors, "LastOnly", True)
        self.parkingSpotDetectionSubscriber = messageHandlerSubscriber(self.queuesList, ParkingSpotDetect, "LastOnly", True)
        self.roundaboutAngleSubscriber = messageHandlerSubscriber(self.queuesList, RoundAboutAngle, "LastOnly", True)  # Corrected to RoundAboutAngle