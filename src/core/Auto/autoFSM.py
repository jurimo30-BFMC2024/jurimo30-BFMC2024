from LaneFollow.LaneFollow import LaneFollow
from src.utils.messages.allMessages import (
    CoreSteerMotor,
    CoreSpeedMotor,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

class autoFSM():
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.laneFollowData = LaneFollow(self.queuesList, self.logging, self.debugging)
        
        self.steerMotorSender = messageHandlerSender(self.queuesList, CoreSteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, CoreSpeedMotor)

        self.subscribe()

    def run(self):
        angle, speed = self.laneFollowData.getControlData()

        self.steerMotorSender.send(f"{angle}")
        self.speedMotorSender.send(f"{speed}")


    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        pass