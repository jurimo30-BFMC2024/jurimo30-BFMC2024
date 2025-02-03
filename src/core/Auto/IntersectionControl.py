
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
        self.navPint = 0
        self.smer = "None"

    def getControlData(self, navigate):
        self.lastStatus = self.status
        intersection = True
        speed = 100

        if(smer == "Right"):
            angle = 230
            time1 = 1.2
            time2 = 5.3
        elif(smer == "Left"):
            angle = -230
            time1 = 2.2
            time2 = 6
        elif(smer == "Straight"):
            angle = 0
            time1 = 1
            time2 = 3
            speed = 200

        if self.status == 0:
            self.smer = navigate[self.navPint]
            self.navPint += 1
            self.lastPoint = time.time()
            self.status = 1
            angle = 0
        elif self.status == 1:
            if (time.time() - self.lastPoint) >= time1:
                self.status = 2
                self.lastPoint = time.time()
                angle = angle
        elif self.status == 2:
            if (time.time() - self.lastPoint) > time2:
                self.status = 0
                intersection = False
                angle = 0
                self.lastPoint = 0
        
        return angle, speed, intersection