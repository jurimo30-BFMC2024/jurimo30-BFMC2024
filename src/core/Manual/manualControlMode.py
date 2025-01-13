from src.core.Core.ControlModeThread.ControlModeThread import ControlModeThread
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
import time

class manualControlMode(ControlModeThread):
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        self.steerMotorSender = messageHandlerSender(self.queuesList, CoreSteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, CoreSpeedMotor)
        self.brakeMotorSender = messageHandlerSender(self.queuesList, CoreBrake)
        self.controlMotorSender = messageHandlerSender(self.queuesList, CoreControl)

        self.steerRecv = 0
        self.speedRecv = 0

        self.subscribe()
        super().__init__()

    def subscribe(self):
        self.controlSubscriber = messageHandlerSubscriber(self.queuesList, Control, "lastOnly", True)
        self.steerMotorSubscriber = messageHandlerSubscriber(self.queuesList, SteerMotor, "lastOnly", True)
        self.speedMotorSubscriber = messageHandlerSubscriber(self.queuesList, SpeedMotor, "lastOnly", True)
        self.brakeSubscriber = messageHandlerSubscriber(self.queuesList, Brake, "lastOnly", True)

    def start(self):
        print(f"stari speed: {self.speedRecv}")
        print(f"stari steer: {self.steerRecv}")
        self.speedMotorSender.send(f"{self.speedRecv}")
        self.steerMotorSender.send(f"{self.steerRecv}")
        super().start()

    def stop(self):
        super().stop()

    def loop(self):
        if self.speedMotorSubscriber.isDataInPipe():
            self.speedRecv = int(self.speedMotorSubscriber.receive())
            self.speedMotorSender.send(f"{self.speedRecv}")
        if self.steerMotorSubscriber.isDataInPipe():
            self.steerRecv = int(self.steerMotorSubscriber.receive())
            self.steerMotorSender.send(f"{self.steerRecv}")
        if self.brakeSubscriber.isDataInPipe():
            brakeRecv = self.brakeSubscriber.receive()
            self.brakeMotorSender.send(brakeRecv)
        if self.controlSubscriber.isDataInPipe():
            controlRecv = self.controlSubscriber.receive()
            self.controlMotorSender.send(controlRecv)
        time.sleep(0.05)