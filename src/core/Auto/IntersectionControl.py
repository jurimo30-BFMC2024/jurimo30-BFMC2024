from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.core.Auto.LaneFollow.MovingAverage import MovingAverage as ma
from src.core.Auto.PID import PIDController as pid
import time
import math
import socket

class IntersectionControl():
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.status = -2 # 0-nije startovano, 1 - startovano ide napred, 2 - startovano mota
        self.lastPoint = 0
        self.navPoint = 0
        self.smer = "None"
    
    def calculate_distance_to_straighten(self, alpha_deg, wheelbase=26, max_steering_angle=25):
        """
        Izračunava dužinu puta (u cm) koju model auta treba da pređe dok se ne ispravi.
    
        Parametri:
        alpha_deg (float): Ugao između auta i zaustavne linije u stepenima (-90 do 90).
        speed_cm_s (float): Brzina kretanja auta napred u cm/s.
        wheelbase (float): Međuosovinsko rastojanje u cm (default 26 cm).
        max_steering_angle (float): Maksimalni ugao skretanja točkova u stepenima (default 25°).
    
        Returns:
        float: Dužina puta u cm koja je potrebna da se auto ispravi.
        """
        # Konvertujemo uglove u radijane
        alpha_rad = math.radians(alpha_deg)
        steering_angle_rad = math.radians(max_steering_angle)
    
        # Poluprečnik kružne putanje (aproksimacija)
        R = wheelbase / math.tan(steering_angle_rad)
    
        # Dužina luka potrebna da se auto ispravi
        distance = R * abs(alpha_rad)

        #po brzini 168, ovaj manevar ce trajati distance/168 sekundi
        #jbg izracunaj sam dole


        return distance


    def getControlData(self, navigate, signs, sign, trafficLights, trafficLightFlag, stopLine):
        self.lastStatus = self.status
        intersection = True
        slope_degrees = stopLine[1]

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

        if self.status == -2:
            if self.debugging:
                print("Pokmrenut manevar raskrsnice")
            self.lastPoint = time.time()
            self.angle = 0
            self.speed = 0

            if trafficLightFlag:
                if trafficLights["green"]:
                    self.status = -1
                    trafficLights["green"] = False
                    self.time0 = 0
                else:
                    self.status = -2
            else:
                self.status = -1
                if sign == "stop":
                    self.time0 = 3
                    if self.debugging:
                        print("Cekanje za znak stop")
                elif sign == "priority":
                    self.time0 = 0
                else:
                    self.time0 = 0

            straighten_distance = self.calculate_distance_to_straighten(slope_degrees)
            # Assume speed is 168 cm/s — calculate duration
            self.straighten_time = straighten_distance / 168.0

        if self.status == -1:
            if ((time.time() - self.lastPoint) >= self.time0) or trafficLightFlag:
                
                if slope_degrees < 90:
                    self.angle = 250
                elif slope_degrees > 90:
                    self.angle = -250
                else:
                    self.angle = 0
                self.speed = 168
                self.lastPoint = time.time()
                self.status = 0
                

        if self.status == 0:
            if (time.time() - self.lastPoint) >= self.straighten_time:
                if self.debugging:
                    print("Krecem sa algoritmom")
                if navigate:  # Check if navigate is not empty
                    # self.send_udp_packet(navigate[0])
                    self.smer = navigate.pop(0)  # Use pop to get and remove the first element
                    if self.debugging:
                        print(f"Smer je {self.smer}")
                else:
                    self.status = -2
                    self.speed = 0
                    self.angle = 0
                    if self.debugging:
                        print("Izlazak iz opsega, staza je zavrsena")
                self.lastPoint = time.time()
                self.status = 1
                self.angle = 0
                self.speed = 168
        elif self.status == 1:
            if (time.time() - self.lastPoint) >= time1 - self.straighten_time:
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
                self.status = -2
                intersection = False
                self.angle = 0
                self.lastPoint = 0
                self.speed = 168
                signs[sign] = False
                trafficLights = {key: False for key in trafficLights}
        
        return self.angle, self.speed, intersection