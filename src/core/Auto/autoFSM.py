from src.core.Auto.Parking.Parking import Parking
from src.core.Auto.Overtake.Overtake import Overtake
from src.core.Core.ControlModeThread.ControlModeThread import ControlModeThread
from src.core.Auto.LaneFollow.LaneFollow import LaneFollow
from src.core.Auto.SpeedControl import SpeedControl
from src.core.Auto.IntersectionControl import IntersectionControl as InterCont
from src.utils.messages.allMessages import (
    CoreSteerMotor,
    CoreSpeedMotor,
    IntersectionDetect,
    IntersectionDetect2,
    ObjectDetection,
    SideSensors,
    FrontSensors,
    ParkingSpotDetect,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.core.Auto.pathPlanning.pathPlanning import PathPlanner as pp
import time
import enum

class autoFSM(ControlModeThread):
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.laneFollowData = LaneFollow(self.queuesList, self.logging, False)
        self.speedControler = SpeedControl(self.logging, self.debugging)
        self.interCont = InterCont(queueList, logging, debugging)
        self.parkingController = Parking(queueList, logging, debugging)
        self.overtakeController = Overtake(queueList, logging, debugging)

        self.steerMotorSender = messageHandlerSender(self.queuesList, CoreSteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, CoreSpeedMotor)

        self.planer = pp(10, 7, "pacman")

        self.subscribe()
        super().__init__()

    def start(self):
        self.oldAngle = 0
        self.oldSpeed = 0
        self.steerMotorSender.send("0")
        self.speedMotorSender.send("0")
        self.navigateCommand = self.planer.planPath()

        print(self.navigateCommand)
        self.traffic_signs = {
            "stop": False,
            "crosswalk": False,
            "highway_entrance": False,
            "highway_exit": False,
            "one_way": False,
            "no_entry": False,
            "parking": False,
            "priority": False,
            "round_about": False
        }

        self.obstacle = False
        self.obstacle_start_time = None

        self.intersection = False
        self.crosswalk = False
        self.highway = False
        self.parking = False
        self.overtake = False

        super().start()
    
    def stop(self):
        super().stop()

    def loop(self):
        angle = self.laneFollowData.getControlData()
        stopLine = self.intersectionDetectSubscriber.receiveWithBlock()
        lowDistance = self.intersectionDetectSubscriber2.receiveWithBlock()
        if self.signDetectionSubscriber.isDataInPipe():
            sign = self.signDetectionSubscriber.receive()
            self.traffic_signs[sign] = True
            if self.debugging:
                print(f"Preuzet je znak {sign}")

        parking_spot_detected = self.parkingSpotDetectionSubscriber.receive() != None

        #ulaz obrade sa ESP
        front_sensors = self.frontSensorSubscriber.receiveWithBlock()
        side_sensors = self.sideSensorSubscriber.receiveWithBlock()

        if not self.parking:
            if self.traffic_signs["parking"]:
                self.traffic_signs["parking"] = False
                self.parking = True
            
        frontDistance = front_sensors["distance"]
        #flogovi za znakove znacajne situacije parking, raskrsnica, semafor ....
        if not self.intersection:
            if self.traffic_signs["stop"] or self.traffic_signs["priority"]:
                if stopLine:
                    if self.debugging:
                        print("Krecemo sa raskrsnicom")
                    self.intersection = True
                    if self.traffic_signs["stop"]:
                        self.intersectionSign = "stop"
                    if self.traffic_signs["priority"]:
                        self.intersectionSign = "priority"

        if not self.highway and self.traffic_signs["highway_entrance"]:
            self.highway = True
            self.traffic_signs["highway_entrance"] = False
            if self.debugging:
                print("Ulazak na autoput")
        if self.highway and (self.traffic_signs["highway_exit"]):
            self.highway = False
            self.traffic_signs["highway_entrance"] = False
            self.traffic_signs["highway_exit"] = False
            if self.debugging:
                print("Izlazak sa auto puta")

        self.obstacle = front_sensors["distance"] <= 80

        if self.highway and self.obstacle:
            self.overtake = True
            print("Overtake on highway")
        elif self.obstacle and self.oldSpeed == 0 and not self.highway:
            if self.obstacle_start_time is None:
                self.obstacle_start_time = time.time()
            
           # if time.time() - self.obstacle_start_time >= 1:
                print("Pass static obstacle start")
                self.overtake = True
        else:
            self.obstacle_start_time = None  # Reset if obstacle is not present

        if not self._running.is_set():
            return
        
        #################         FSM            ############
        if self.parking:
            park_angle, speed, self.parking = self.parkingController.run(parking_spot_detected, side_sensors)
            if park_angle is not None:
                angle = park_angle
        elif self.intersection:
            angle, speed, self.intersection = self.interCont.getControlData(self.navigateCommand, self.traffic_signs, self.intersectionSign, self.oldAngle)
            pass
        elif self.overtake:
            overtake_angle, speed, self.overtake = self.overtakeController.run(self.highway, front_sensors, side_sensors)
            if overtake_angle is not None:
                angle = overtake_angle
        else:
            speed = self.speedControler.getControlData(angle, stopLine, lowDistance, self.highway, frontDistance)

        ############ Sending data ##############################

        if angle != self.oldAngle:
            self.steerMotorSender.send(f"{angle}")
            self.oldAngle = angle
            if self.debugging:
                self.logging.info(f"New steering angle: {angle}")

        if speed != self.oldSpeed:
            self.speedMotorSender.send(f"{speed}")
            self.oldSpeed = speed
            if self.debugging:
                self.logging.info(f"New speed: {speed}")
        
        time.sleep(0.05)

    def getTime(self):
        return round(time.time()*1000)

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.intersectionDetectSubscriber = messageHandlerSubscriber(self.queuesList, IntersectionDetect, "LastOnly", True)
        self.intersectionDetectSubscriber2 = messageHandlerSubscriber(self.queuesList, IntersectionDetect2, "LastOnly", True)
        self.signDetectionSubscriber = messageHandlerSubscriber(self.queuesList, ObjectDetection, "FIFO", True)
        self.sideSensorSubscriber = messageHandlerSubscriber(self.queuesList, SideSensors, "LastOnly", True)
        self.frontSensorSubscriber = messageHandlerSubscriber(self.queuesList, FrontSensors, "LastOnly", True)
        self.parkingSpotDetectionSubscriber = messageHandlerSubscriber(self.queuesList, ParkingSpotDetect, "LastOnly", True)
        
