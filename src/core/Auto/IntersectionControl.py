
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

    def getControlData(self, smer):
        self.lastStatus = self.status
        intersection = True

        if(smer == "Right"):
            angle = 230
            time1 = 1.2
            time2 = 5.3
        else:
            angle = -230
            time1 = 2.2
            time2 = 6

        if self.status == 0:
            self.lastPoint = time.time()
            self.status = 1
            angle = 0
            speed = 100
            print("Raskrsnica krenula")
        elif self.status == 1:
            if (time.time() - self.lastPoint) >= time1:
                print("Krecem da motam")
                self.status = 2
                self.lastPoint = time.time()
                angle = angle
                speed = 100
        elif self.status == 2:
            if (time.time() - self.lastPoint) > time2:
                print("Zavrsena raskrsnica")
                self.status = 0
                intersection = False
                angle = 0
                speed = 0
                self.lastPoint = 0
        
        return angle, speed, intersection