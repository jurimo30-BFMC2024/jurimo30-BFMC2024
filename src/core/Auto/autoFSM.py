from src.core.Auto.Parking.Parking import Parking
from src.core.Auto.Overtake.Overtake import Overtake
from src.core.Core.ControlModeThread.ControlModeThread import ControlModeThread
from src.core.Auto.LaneFollow.LaneFollow import LaneFollow
from src.core.Auto.SpeedControl import SpeedControl
from src.core.Auto.IntersectionControl import IntersectionControl as InterCont
from src.core.Auto.RoundaboutControl import RoundaboutControl
from src.utils.messages.allMessages import (
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
from src.core.Auto.pathPlanning.pathPlanning import PathPlanner as pp
import time
import enum

class autoFSM(ControlModeThread):
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.laneFollowData = LaneFollow(self.queuesList, self.logging, False)
        self.speedControler = SpeedControl(self.logging, False)
        self.interCont = InterCont(queueList, logging, debugging)
        self.parkingController = Parking(queueList, logging, debugging)
        self.overtakeController = Overtake(queueList, logging, debugging)
        self.roundaboutController = RoundaboutControl(logging, debugging)

        self.steerMotorSender = messageHandlerSender(self.queuesList, CoreSteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, CoreSpeedMotor)

        self.planer = pp(43, 10, "pacman")

        self.subscribe()
        super().__init__()

    def start(self):
        self.oldAngle = 0
        self.oldSpeed = 0
        self.steerMotorSender.send("0")
        self.speedMotorSender.send("0")
        #self.navigateCommand = self.planer.planPath()
        self.navigateCommand = ["Straight", "Right", "Right", "Straight", "Right", "Straight", "Straight", "Straight", "Right"]

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
            "round_about": False,
            "round_about2": False
        }

        self.trafficLightStates = {
            "red": False,
            "green": False,
            "yellow": False,
            "red_yellow": False
        }

        self.sign_car_detected = False
        self.obstacle = False
        self.obstacle_start_time = None
        self.trafficLight = False
        self.lowDistance = False

        self.intersection = False
        self.crosswalk = False
        self.highway = False
        self.parking = False
        self.overtake = False
        self.stephanie = False
        self.roundabout = False
        self.roundaboutExitFlag = False

        super().start()
    
    def stop(self):
        super().stop()

    def loop(self):
        angle = self.laneFollowData.getControlData(self.highway, self.lowDistance)
        stopLine = self.intersectionDetectSubscriber.receiveWithBlock()
        self.lowDistance = self.intersectionDetectSubscriber2.receiveWithBlock()
        if self.signDetectionSubscriber.isDataInPipe():
            sign = self.signDetectionSubscriber.receive()

            if sign == "stefanija":
                self.stephanie = True
            elif sign == "exit":
                self.roundaboutExitFlag = True
                print("Exit from roundabout is received in AUTO FSM")
            elif sign == "car":
                self.sign_car_detected = True
            elif sign in self.trafficLightStates:
                for key in self.trafficLightStates:
                    self.trafficLightStates[key] = False
                self.trafficLightStates[sign] = True
            else:
                self.traffic_signs[sign] = True

          
            print(f"Preuzet je znak {sign}")

        parking_spot_detected = self.parkingSpotDetectionSubscriber.receive() != None

        #ulaz obrade sa ESP
        front_sensors = self.frontSensorSubscriber.receiveWithBlock()
        side_sensors = self.sideSensorSubscriber.receiveWithBlock()

        # Receive roundabout-related messages
        roundabout_angle = self.roundaboutAngleSubscriber.receiveWithBlock()  # Corrected to RoundAboutAngle

        if not self.parking and not self.overtake and not self.intersection:
            if self.traffic_signs["parking"]:
                self.traffic_signs["parking"] = False
                self.parking = True
            
        self.trafficLight = any(self.trafficLightStates.values())

        #flogovi za znakove znacajne situacije parking, raskrsnica, semafor ....
        if not self.intersection and not self.parking and not self.overtake:
            if self.traffic_signs["stop"] or self.traffic_signs["priority"] or self.trafficLight:

                if stopLine:
                    if self.debugging:
                        print("Krecemo sa raskrsnicom")
                    self.intersection = True
                    if self.traffic_signs["stop"]:
                        self.intersectionSign = "stop"
                    if self.traffic_signs["priority"]:
                        self.intersectionSign = "priority"
                    
                    if self.trafficLight:
                        self.intersectionSign = "None"
                        

        if not self.highway and self.traffic_signs["highway_entrance"] and not self.parking and not self.overtake and not self.intersection:
            self.highway = True
            self.traffic_signs["highway_entrance"] = False
            if self.debugging:
                print("Ulazak na autoput")
        if self.highway and (self.traffic_signs["highway_exit"] or stopLine or self.lowDistance) and not self.parking and not self.overtake and not self.intersection:
            self.highway = False
            self.traffic_signs["highway_entrance"] = False
            self.traffic_signs["highway_exit"] = False
            if self.debugging:
                print("Izlazak sa auto puta")

        # Latching obstacle if:
        # - distance is low, and
        # - either car or stefanija (when not at crosswalk) is detected
        valid_sign_obstacle = self.sign_car_detected
        self.obstacle = front_sensors["distance"] <= 80 and valid_sign_obstacle

        #if self.highway and self.obstacle and not self.parking and not self.intersection:
            #self.overtake = True
            #print("Overtake on highway")
        if self.obstacle and self.oldSpeed == 0 and not self.highway and not self.parking and not self.intersection:
            if self.obstacle_start_time is None:
                self.obstacle_start_time = time.time()
            
            if time.time() - self.obstacle_start_time >= 1:
                print("Pass static obstacle start")
                self.overtake = True
                self.sign_car_detected = False
        else:
            self.obstacle_start_time = None  # Reset if obstacle is not present

        #temp solution
        if self.traffic_signs["crosswalk"] and self.stephanie and stopLine and not self.crosswalk:
            self.crosswalk = True
            self.crosswalkStart = time.time()
            

        if not self._running.is_set():
            return

        if not self.roundabout and not self.parking and not self.overtake and not self.intersection:
            if self.traffic_signs["round_about"] or self.traffic_signs["round_about2"]:
                if stopLine:
                    if self.debugging:
                        print("Entering roundabout")
                    self.roundabout = True
                    self.traffic_signs["round_about"] = False
                    self.traffic_signs["round_about2"] = False

        #################         FSM            ############
        if self.parking:
            park_angle, speed, self.parking = self.parkingController.run(parking_spot_detected, side_sensors)
            if park_angle is not None:
                angle = park_angle
        elif self.intersection:
            angle, speed, self.intersection = self.interCont.getControlData(self.navigateCommand, self.traffic_signs, self.intersectionSign, self.trafficLightStates, self.trafficLight)
            pass
        elif self.overtake:
            overtake_angle, speed, self.overtake = self.overtakeController.run(self.highway, front_sensors, side_sensors)
            if overtake_angle is not None:
                angle = overtake_angle
        elif self.roundabout:
            angle, speed, self.roundabout, self.roundaboutExitFlag  = self.roundaboutController.getControlData(
                angleForRoundabout=roundabout_angle,  # Use the received angle
                navigate=self.navigateCommand,
                exitFlag=self.roundaboutExitFlag
            )
        elif self.crosswalk:
            angle = 0
            speed = 0
            if time.time() - self.crosswalkStart >= 3:
                self.crosswalk = False
                self.stephanie = False
                self.traffic_signs["crosswalk"] = False
        else:
            speed = self.speedControler.getControlData(angle, stopLine, self.lowDistance, self.highway, front_sensors["distance"], (not(any(self.traffic_signs.values()) or any(self.trafficLightStates.values()))), self.sign_car_detected, (self.stephanie and not self.traffic_signs["crosswalk"]))

        ############ Sending data ##############################

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
        self.roundaboutAngleSubscriber = messageHandlerSubscriber(self.queuesList, RoundAboutAngle, "LastOnly", True)  # Corrected to RoundAboutAngle
