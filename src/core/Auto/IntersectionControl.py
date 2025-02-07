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
        self.status = -1 # 0-nije startovano, 1 - startovano ide napred, 2 - startovano mota
        self.lastPoint = 0
        self.navPoint = 0
        self.smer = "None"

    def getControlData(self, navigate, signs, sign, oldAngle):
        self.lastStatus = self.status
        intersection = True

        if(self.smer == "Right"):
            tangle = 230
            time1 = 1.2
            time2 = 5.3
        elif(self.smer == "Left"):
            tangle = -230
            time1 = 2.2
            time2 = 6
        elif(self.smer == "Straight"):
            tangle = 0
            time1 = 1
            time2 = 4
        else:
            tangle = 0
            time1 = 100
            time2 = 100

        if self.status == -1:
            if self.debugging:
                print("Pokmrenut manevar raskrsnice")
            self.status = 0
            self.lastPoint = time.time()
            self.angle = 0
            self.speed = 0
            if sign == "stop":
                self.time0 = 3
                if self.debugging:
                    print("Cekanje za znak stop")
            elif sign == "priority":
                self.time0 = 0
            else:
                self.time0 = 0


        if self.status == 0:
            if (time.time() - self.lastPoint) >= self.time0:
                if self.debugging:
                    print("Krecem sa algoritmom")
                if len(navigate) != self.navPoint:
                    self.smer = navigate[self.navPoint]
                    if self.debugging:
                        print(f"Smer je {self.smer}")
                else:
                    self.status = -1
                    if self.debugging:
                        print("Izlazak iz opsega, staza je zavrsena")
                self.navPoint += 1
                self.lastPoint = time.time()
                self.status = 1
                self.angle = 0
                self.speed = 100
        elif self.status == 1:
            if (time.time() - self.lastPoint) >= time1:
                if self.debugging:
                    print("Krecem da motam")
                self.status = 2
                self.lastPoint = time.time()
                self.angle = tangle
                self.speed = 100
        elif self.status == 2:
            if (time.time() - self.lastPoint) >= time2:
                if self.debugging:
                    print("kraj")
                self.status = -1
                intersection = False
                self.angle = 0
                self.lastPoint = 0
                self.speed = 100
                signs[sign] = False
        
        return self.angle, self.speed, intersection