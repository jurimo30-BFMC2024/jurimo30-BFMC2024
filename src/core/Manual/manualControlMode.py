from src.utils.messages.allMessages import (mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

from src.utils.messages.allMessages import (
    Control,
    SteerMotor,
    SpeedMotor,
    Brake,
    CoreBrake,
    CoreControl,
    CoreSpeedMotor,
    CoreSteerMotor
)

class manualControlMode():
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        self.steerMotorSender = messageHandlerSender(self.queuesList, CoreSteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, CoreSpeedMotor)
        self.brakeMotorSender = messageHandlerSender(self.queuesList, CoreBrake)
        self.controlMotorSender = messageHandlerSender(self.queuesList, CoreControl)

        self.subscribe()

    def subscribe(self):
        self.controlSubscriber = messageHandlerSubscriber(self.queuesList, Control, "lastOnly", True)
        self.steerMotorSubscriber = messageHandlerSubscriber(self.queuesList, SteerMotor, "lastOnly", True)
        self.speedMotorSubscriber = messageHandlerSubscriber(self.queuesList, SpeedMotor, "lastOnly", True)
        self.brakeSubscriber = messageHandlerSubscriber(self.queuesList, Brake, "lastOnly", True)

    def run(self):
        if self.speedMotorSubscriber.isDataInPipe():
            speedRecv = self.speedMotorSubscriber.receive()
            self.speedMotorSender.send(speedRecv)
        if self.steerMotorSubscriber.isDataInPipe():
            steerRecv = self.steerMotorSubscriber.receive()
            self.steerMotorSender.send(steerRecv)
        if self.brakeSubscriber.isDataInPipe():
            brakeRecv = self.brakeSubscriber.receive()
            self.brakeMotorSender.send(brakeRecv)
        if self.controlSubscriber.isDataInPipe():
            controlRecv = self.controlSubscriber.receive()
            self.controlMotorSender.send(controlRecv)