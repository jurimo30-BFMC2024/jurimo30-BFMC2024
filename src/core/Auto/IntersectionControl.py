from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.core.Auto.LaneFollow.MovingAverage import MovingAverage as ma
from src.core.Auto.PID import PIDController as pid
import time
import socket

class IntersectionControl():
    def __init__(self, logging, debugging=False):
        self.logging = logging
        self.debugging = debugging
        self.status = -1 # 0-nije startovano, 1 - startovano ide napred, 2 - startovano mota
        self.lastPoint = 0
        self.navPoint = 0
        self.smer = "None"

    def getControlData(self, navigate, signs, sign, trafficLights, trafficLightFlag):
        self.lastStatus = self.status
        intersection = True

        if(self.smer == "Right"):
            tangle = 240
            time1 = 2.0
            time2 = 5.2
        elif(self.smer == "Left"):
            tangle = -190
            time1 = 3.5
            time2 = 5.8
        elif(self.smer == "Straight"):
            tangle = 0
            time1 = 5
            time2 = 4
        else:
            tangle = 0
            time1 = 100
            time2 = 100

        if self.status == -1:
            if self.debugging:
                print("Pokmrenut manevar raskrsnice")
            self.lastPoint = time.time()
            self.angle = 0
            self.speed = 0

            if trafficLightFlag:
                if trafficLights["green"]:
                    self.status = 0
                    trafficLights["green"] = False
                    self.time0 = 0
                else:
                    self.status = -1
            else:
                self.status = 0
                if sign == "stop":
                    self.time0 = 3
                    if self.debugging:
                        print("Cekanje za znak stop")
                elif sign == "priority":
                    self.time0 = 0
                else:
                    self.time0 = 0


        if self.status == 0:
            if ((time.time() - self.lastPoint) >= self.time0) or trafficLightFlag:
                if self.debugging:
                    print("Krecem sa algoritmom")
                if navigate:  # Check if navigate is not empty
                    # self.send_udp_packet(navigate[0])
                    self.smer = navigate.pop(0)  # Use pop to get and remove the first element
                    if self.debugging:
                        print(f"Smer je {self.smer}")
                else:
                    self.status = -1
                    self.speed = 0
                    self.angle = 0
                    if self.debugging:
                        print("Izlazak iz opsega, staza je zavrsena")
                self.lastPoint = time.time()
                self.status = 1
                self.angle = 0
                self.speed = 168
        elif self.status == 1:
            if (time.time() - self.lastPoint) >= time1:
                if self.debugging:
                    print("Krecem da motam")
                self.status = 2
                self.lastPoint = time.time()
                self.angle = tangle
                self.speed = 168
        elif self.status == 2:
            if (time.time() - self.lastPoint) >= time2:
                if self.debugging:
                    print("kraj")
                self.status = -1
                intersection = False
                self.angle = 0
                self.lastPoint = 0
                self.speed = 168
                signs[sign] = False
                trafficLights = {key: False for key in trafficLights}
        
        return self.angle, self.speed, intersection