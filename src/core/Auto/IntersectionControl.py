
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.core.Auto.LaneFollow.MovingAverage import MovingAverage as ma
from src.core.Auto.PID import PIDController as pid
import time

class IntersectionControl():
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.status = 0 # 0-nije startovano, 1 - startovano ide napred, 2 - startovano mota
        self.lastPoint = 0

    def getControlData(self):
        angle = 0
        speed = 0
        self.lastStatus = self.status
        intersection = True

        if self.status == 0:
            self.lastPoint = time.time()
            self.status = 1
            angle = 0
            speed = 10
        elif self.status == 1:
            if (time.time() - self.lastPoint) >= 1.5:
                self.status = 2
                self.lastPoint = time.time()
                angle = -23
                speed = 10
        elif self.status == 2:
            if (time.time() - self.lastPoint) > 5:
                self.status = 0
                intersection = False
                angle = 0
                speed = 0
                self.lastPoint = 0
        
        return angle, speed, intersection

