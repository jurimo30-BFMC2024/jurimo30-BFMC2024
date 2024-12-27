from src.core.Core.ControlModeThread.ControlModeThread import ControlModeThread
from src.core.Auto.LaneFollow.LaneFollow import LaneFollow
from src.utils.messages.allMessages import (
    CoreSteerMotor,
    CoreSpeedMotor,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
import time

class autoFSM(ControlModeThread):
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.laneFollowData = LaneFollow(self.queuesList, self.logging, self.debugging)
        
        self.steerMotorSender = messageHandlerSender(self.queuesList, CoreSteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, CoreSpeedMotor)

        self.subscribe()
        super().__init__()

    def start(self):
        self.oldAngle = 0
        self.oldSpeed = 0
        self.steerMotorSender.send("0")
        self.speedMotorSender.send("0")
        super().start()

    def run(self):
        while self._running.is_set():
            angle, speed = self.laneFollowData.getControlData()
            if not self._running.is_set():
                return

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
        pass