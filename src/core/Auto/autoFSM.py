if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.core.Auto.Parking.Parking import Parking
from src.core.Auto.Overtake.Overtake import Overtake
from src.core.Core.ControlModeThread.ControlModeThread import ControlModeThread
from src.core.Auto.LaneFollow.LaneFollow import LaneFollower as LaneFollowController
from src.core.Auto.SpeedControl import SpeedControl
from src.core.Auto.IntersectionControl import IntersectionControl
from src.core.Auto.RoundaboutControl import RoundaboutControl
from src.utils.messages.allMessages import (
    LaneDetect,
    CoreSteerMotor,
    CoreSpeedMotor,
    StopLineDetect,
    ObjectDetection,
    TrafficSignsDetection,
    SideSensors,
    FrontSensors,
    ParkingSpotDetect,
    RoundAboutAngle
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.core.Auto.pathPlanning.pathPlanning import PathPlanner
from src.core.Auto.Localization.Localization import Localization
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
        self.planer = PathPlanner(start=10, goal=43, mode="pacman")
        self.laneFollowContrler = LaneFollowController(512, 270, self.logging, False)
        self.speedControler = SpeedControl(self.logging, False)
        self.intersectionController = IntersectionControl(self.logging, self.debugging)
        self.parkingController = Parking(self.logging, self.debugging)
        self.overtakeController = Overtake(self.logging, self.debugging)
        self.roundaboutController = RoundaboutControl(self.logging, self.debugging)

        self.laneDetectSubscriber.empty()
        self.stopLineDetectionSubscriber.empty()
        self.objectDetectionSubscriber.empty()
        self.trafficSignsSubscriber.empty()
        self.sideSensorSubscriber.empty()
        self.frontSensorSubscriber.empty()
        self.parkingSpotDetectionSubscriber.empty()
        self.roundaboutAngleSubscriber.empty()

        self.oldAngle = 0
        self.oldSpeed = 0
        self.steerMotorSender.send("0")
        self.speedMotorSender.send("0")
        self.navigateCommand, segmentsData = self.planer.planPath()
        self.localization = Localization(segmentsData)
        self.localization.start_new_segment()

        print(self.navigateCommand)
        self.traffic_signs = TrafficSignController([
            "stop", "crosswalk", "highway_entrance", "highway_exit",
            "one_way", "no_entry", "parking", "priority",
            "round_about", "round_about2"
        ])

        self.traffic_light_states = TrafficSignController([
            "red", "green", "yellow", "red_yellow"
        ])

        self.sign_car_position = False
        self.obstacle_start_time = None
        self.stephanie_position = False
        self.roundaboutExit_position = False
        self.intersectionSign = "None"

        self.state = autoFSMState.DRIVE

        super().start()
    
    def stop(self):
        super().stop()

    def loop(self):
        self.leftX, self.rightX = self.laneDetectSubscriber.receiveWithBlock()
        stop_line_present, stop_line_distance, stop_line_angle = self.stopLineDetectionSubscriber.receiveWithBlock() # stopLine je sad tuple (intersection(bool), slope_degrees (float))
        stop_line_present_close = stop_line_present and stop_line_distance < 130

        while self.objectDetectionSubscriber.isDataInPipe():
            detected_objects_dict = self.objectDetectionSubscriber.receive() # This is now a dict
            object_name = detected_objects_dict.get("name")
            object_position = detected_objects_dict.get("position")

            if object_name == "stefanija":
                self.stephanie_position = object_position
                if self.debugging: print(f"Stephanie present: {self.stephanie_position}")
            elif object_name == "exit":
                self.roundaboutExit_position = object_position
                if self.debugging: print(f"Roundabout Exit present: {self.roundaboutExit_position}")
            elif object_name == "car":
                self.sign_car_position = object_position
                if self.debugging: print(f"Car present: {self.sign_car_position}")
            else:
                raise ValueError(f'Unknown object detected: {detected_objects_dict}')
            
            if self.debugging:
                print(f"Preuzet je objekat: {object_name}, Prisutan: {object_position}")

        while self.trafficSignsSubscriber.isDataInPipe():
            sign = self.trafficSignsSubscriber.receive() # This is a string (sign name)
            if sign in self.traffic_light_states:
                self.traffic_light_states.set_active(sign)
                if self.debugging: print(f"Traffic light detected: {sign}")
            elif sign in self.traffic_signs:
                self.traffic_signs.set_active(sign)
                if self.debugging: print(f"Traffic sign detected: {sign}")
            else:
                 raise ValueError(f'Unknown sign detected: {sign}')
            
            if self.debugging:
                print(f"Preuzet je saobracajni znak: {sign}")

        parking_spot_detected = self.parkingSpotDetectionSubscriber.receive() != None

        #ulaz obrade sa ESP
        front_sensors = self.frontSensorSubscriber.receiveWithBlock()
        side_sensors = self.sideSensorSubscriber.receiveWithBlock()

        # Receive roundabout-related messages
        roundabout_angle = self.roundaboutAngleSubscriber.receiveWithBlock()  # Corrected to RoundAboutAngle
        traffic_light_present = self.traffic_light_states.get_active() != None

        obstacle = front_sensors["distance"] <= 80 and self.sign_car_position
        angle = self.laneFollowContrler.process_following(self.leftX, self.rightX) # calculate steering angle from lane follow data

        if not obstacle:
            self.obstacle_start_time = None  # Reset if obstacle is not present

        if not self._running.is_set():
            return
        
        ##############################         SET STATES            #############################

        
        if self.state == autoFSMState.HIGHWAY:
            if self.traffic_signs.get_active() == "highway_exit" or stop_line_present:
                if self.debugging:
                    print("Izlazak sa auto puta")
                self.traffic_signs.clear()
                self.laneFollowContrler.set_pid_highway(False)
                self.state = autoFSMState.DRIVE

        
        if self.state == autoFSMState.DRIVE:
            if self.traffic_signs.get_active() == "parking":
                self.traffic_signs.clear()
                self.state = autoFSMState.PARKING
            
            elif stop_line_present_close and (self.traffic_signs.get_active() in ["stop", "priority"] or traffic_light_present):
                if self.debugging:
                    print("Krecemo sa raskrsnicom")
                
                self.intersectionController.setCourse(
                    sign=self.traffic_signs.get_active(), 
                    direction=self.navigateCommand.pop(0),
                    traffic_light_present=traffic_light_present
                )
                self.traffic_signs.clear()
                self.localization.update_speed_error()
                # print("Speed error:", self.localization.speed_error)
                self.state = autoFSMState.INTERSECTION

            elif stop_line_present_close and self.traffic_signs.get_active() in ["round_about", "round_about2"]:
                if self.debugging:
                    print("Entering roundabout")
                self.traffic_signs.clear()
                self.localization.update_speed_error()
                # print("Speed error:", self.localization.speed_error)
                self.state = autoFSMState.ROUNDABOUT


            elif stop_line_present and self.traffic_signs.get_active() == "crosswalk" and self.stephanie_position:
                self.crosswalkStart = time.time()
                self.state = autoFSMState.CROSSWALK
                        
            elif self.traffic_signs.get_active() == "highway_entrance":
                if self.debugging:
                    print("Ulazak na autoput")
                self.traffic_signs.clear()
                self.laneFollowContrler.set_pid_highway(True)
                self.state = autoFSMState.HIGHWAY
            
            elif obstacle and self.oldSpeed == 0:
                if self.obstacle_start_time is None:
                    self.obstacle_start_time = time.time()
                
                if time.time() - self.obstacle_start_time >= 1:
                    print("Pass static obstacle start")
                    self.sign_car_position = False
                    self.state = autoFSMState.OVERTAKE

        ##############################         FSM            #############################

        if self.state == autoFSMState.PARKING:
            park_angle, speed, module_running = self.parkingController.run(parking_spot_detected, side_sensors)
            if park_angle is not None:
                angle = park_angle

            if not module_running:
                self.state = autoFSMState.DRIVE
            
        elif self.state == autoFSMState.INTERSECTION:
            angle, speed, module_running = self.intersectionController.getControlData(
                stop_line_present=stop_line_present_close,
                stop_line_slope=stop_line_angle,
                trafficLights=self.traffic_light_states
            )
            
            if not module_running:
                self.localization.start_new_segment()
                self.state = autoFSMState.DRIVE

        elif self.state == autoFSMState.OVERTAKE:
            overtake_angle, speed, module_running = self.overtakeController.run(False, front_sensors, side_sensors)
            if overtake_angle is not None:
                angle = overtake_angle

            if not module_running:
                self.state = autoFSMState.DRIVE

        elif self.state == autoFSMState.CROSSWALK:
            angle = 0
            speed = 0
            if time.time() - self.crosswalkStart >= 3:
                self.stephanie_position = False
                self.traffic_signs.clear()
                self.state = autoFSMState.DRIVE

        elif self.state == autoFSMState.ROUNDABOUT:
            angle, speed, module_running, self.roundaboutExit_position  = self.roundaboutController.getControlData(
                angleForRoundabout=roundabout_angle,  # Use the received angle
                navigate=self.navigateCommand,
                exitFlag=self.roundaboutExit_position,
                stop_line_present=stop_line_present,
                stop_line_slope=stop_line_angle
            )

            if not module_running:
                self.localization.start_new_segment()
                self.state = autoFSMState.DRIVE

        elif self.state == autoFSMState.DRIVE or self.state == autoFSMState.HIGHWAY:
            no_active_sign = self.traffic_signs.get_active() is None and self.traffic_light_states.get_active() is None
            stephanie_crossing = self.stephanie_position and self.traffic_signs.get_active() != "crosswalk"

            speed = self.speedControler.getControlData(
                angle=angle,
                stopLine=stop_line_present_close,
                lowDistance=stop_line_present,
                highway=self.state == autoFSMState.HIGHWAY,
                frontDistance=front_sensors["distance"],
                enable_emergency_stop=no_active_sign,
                car_in_front=self.sign_car_position,
                stephanie_in_front=stephanie_crossing
            )

            self.localization.update_position(speed / 10)
            print(f"Distance[est]: {self.localization.total_distance:.2f}, Speed[avg]: {self.localization.average_target_speed:.2f}, Position: {self.localization.get_location()}")
            

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
        self.stopLineDetectionSubscriber = messageHandlerSubscriber(self.queuesList, StopLineDetect, "LastOnly", True)
        self.objectDetectionSubscriber = messageHandlerSubscriber(self.queuesList, ObjectDetection, "FIFO", True)
        self.trafficSignsSubscriber = messageHandlerSubscriber(self.queuesList, TrafficSignsDetection, "FIFO", True)
        self.sideSensorSubscriber = messageHandlerSubscriber(self.queuesList, SideSensors, "LastOnly", True)
        self.frontSensorSubscriber = messageHandlerSubscriber(self.queuesList, FrontSensors, "LastOnly", True)
        self.parkingSpotDetectionSubscriber = messageHandlerSubscriber(self.queuesList, ParkingSpotDetect, "LastOnly", True)
        self.roundaboutAngleSubscriber = messageHandlerSubscriber(self.queuesList, RoundAboutAngle, "LastOnly", True)  # Corrected to RoundAboutAngle