from src.utils.messages.allMessages import (mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

from src.utils.messages.allMessages import (
    Control,
    SteerMotor,
    SpeedMotor,
    Brake
)

class manualControlMode():
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.subscribe()

    def subscribe(self):
        self.controlSubscriber = messageHandlerSubscriber(self.queuesList, Control, "lastOnly", True)
        self.steerMotorSubscriber = messageHandlerSubscriber(self.queuesList, SteerMotor, "lastOnly", True)
        self.speedMotorSubscriber = messageHandlerSubscriber(self.queuesList, SpeedMotor, "lastOnly", True)
        self.brakeSubscriber = messageHandlerSubscriber(self.queuesList, Brake, "lastOnly", True)

    def run(self):
        if(self.speedMotorSubscriber.isDataInPipe):
            speedRecv = self.speedMotorSubscriber.receive
        if(self.steerMotorSubscriber.isDataInPipe):
            steerRecv = self.steerMotorSubscriber.receive
        if(self.brakeSubscriber.isDataInPipe):
            speedRecv = self.brakeSubscriber.receive
        if(self.controlSubscriber.isDataInPipe):
            speedRecv = self.controlSubscriber.receive