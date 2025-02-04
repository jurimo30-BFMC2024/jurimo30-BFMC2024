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
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
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
        
        self.steerMotorSender = messageHandlerSender(self.queuesList, CoreSteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, CoreSpeedMotor)

        self.subscribe()
        super().__init__()

    def start(self):
        self.oldAngle = 0
        self.oldSpeed = 0
        self.steerMotorSender.send("0")
        self.speedMotorSender.send("0")
        self.navigateCommand = ["Right", "Right", "Straight", "Right", "Left"]
        self.traffic_signs = {
            "stop sign": False,
            "crosswalk sign": False,
            "highway entrance sign": False,
            "highway exit sign": False,
            "one way road sign": False,
            "no-entry road sign": False,
            "parking sign": False,
            "priority sign": False,
            "round-about sign": False
        }

        self.intersection = False
        self.crosswalk = False
        self.highway = False
        self.parking = False

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
            print(f"Preuzet je znak {sign}")

        #ulaz obrade sa ESP
        obstacle = False
        #flogovi za znakove znacajne situacije parking, raskrsnica, semafor ....
        if not self.intersection:
            if self.traffic_signs["stop sign"]:
                if stopLine:
                    print("Krecemo sa raskrsnicom")
                    self.intersection = True
                    self.intersectionSign = "stop sign"
        if not self.highway and self.traffic_signs["highway entrance sign"]:
            self.highway = True
            self.traffic_signs["highway entrance sign"] = False
        if self.highway and self.traffic_signs["highway exit sign"]:
            self.highway = False
            self.traffic_signs["highway entrance sign"] = False


    
        if not self._running.is_set():
            return
        

        #################         FSM            ############
        if self.parking:
            pass
        elif self.intersection:
            angle, speed, self.intersection = self.interCont.getControlData(self.navigateCommand, self.traffic_signs, self.intersectionSign)
            pass
        else:
            speed = self.speedControler.getControlData(angle, stopLine, lowDistance, self.highway, False)

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
        pass