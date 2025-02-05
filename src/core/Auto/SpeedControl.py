from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.core.Auto.LaneFollow.MovingAverage import MovingAverage as ma
from src.core.Auto.PID import PIDController as pid

class SpeedControl():
    def __init__(self, logging, debugging=False):
        self.logging = logging
        self.debugging = debugging
        self.avgSpeed = ma(20)
        self.pid = pid(0.5, 0.3, 0)

    def filter(self, angle, alpha = 0.3):
        self.oldAngle = angle * alpha + self.oldAngle * (1 - alpha)
        return self.oldAngle

    def map_value(self, value, in_min=30, in_max=170, out_min=150, out_max=300):
        value = max(min(value, in_max), in_min)
        return out_min + (out_max - out_min) * (value - in_min) / (in_max - in_min)

    def getControlData(self, angle, stopLine, lowDistance, highway, frontDistance):

        if stopLine:
            return 65

        if not highway:
            if abs(angle) < 30:
                speed = 250
            elif abs(angle) > 170:
                print(f"angle: {angle} speed: 100")
                speed = 100
            else:
                speed = self.map_value(angle, 30, 170, 100, 250)
        else:
            if abs(angle) < 70:
                speed = 350
            elif abs(angle) > 170:
                speed = 200
            else:
                speed = self.map_value(angle, 70, 170, 200, 350)


        if lowDistance:
            n = 0
            speed = 50
            while n < 3 and self.avgSpeed.get_average() > 120:
                self.avgSpeed.add(70)
                n += 1

        if frontDistance < 30:
            self.avgSpeed.add(0)
            return 0

        speed = int(self.avgSpeed.filter(speed))

        return speed
