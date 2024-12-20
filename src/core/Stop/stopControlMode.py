from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import (
    CoreBrake,
    CoreControl,
    CoreSpeedMotor,
    CoreSteerMotor
)

class stopControlMode():
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        
        self.has_run = False
        self.steerMotorSender = messageHandlerSender(self.queuesList, CoreSteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, CoreSpeedMotor)

        self.subscribe()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        pass

    def reset(self):
        self.has_run = False

    def stop(self):
        if not self.has_run:
            self.speedMotorSender.send("0")
            self.steerMotorSender.send("0")
            self.has_run = True