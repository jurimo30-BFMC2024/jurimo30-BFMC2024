from src.core.Auto.LaneFollow.MovingAverage import MovingAverage
from src.core.Auto.PIDController import PIDController

import time

class SpeedControl():
    def __init__(self, logging, debugging=False):
        self.logging = logging
        self.debugging = debugging
        self.avgSpeed = MovingAverage(20)
        self.pid = PIDController(20, 7, 0.5)
        self.followDistance = 50  # Desired following distance in cm
        self.emergencyStopDistance = 35  # Emergency stop threshold in cm
        self.emergency_stop_threshold = 3  # Consecutive readings required
        self.consecutive_emergency = 0
        self.stop = False
        self.waiting_time = 6.0
        self.entered_highway_time = None

        self.right_line_only_nodes = {
            "399", "400", "401", "444", "445", "446",
            "421", "422", "423", "463", "464", "465",
            # Dodaj specifične nodove gde treba pratiti samo desnu liniju
        }

    def filter(self, angle, alpha=0.3):
        self.oldAngle = angle * alpha + self.oldAngle * (1 - alpha)
        return self.oldAngle

    def map_value(self, value, in_min=30, in_max=170, out_min=150, out_max=300):
        value = max(min(value, in_max), in_min)
        return out_min + (out_max - out_min) * (value - in_min) / (in_max - in_min)

    def getControlData(self, angle, stopLine, lowDistance, highway, frontDistance,
                   enable_emergency_stop, car_in_front, stephanie_in_front, current_node: str | None):
        if stopLine:
            return 65


        # Speed calculation based on highway mode and angle
        if not highway:
            if abs(angle) < 30:
                speed = 350
            elif abs(angle) > 145:
                speed = 220
            else:
                speed = self.map_value(angle, 30, 145, 280, 350)
        else:
            if current_node in self.right_line_only_nodes:
                speed = 280
            else:
                if abs(angle) < 30:
                    speed = 480
                elif abs(angle) > 100:
                    speed = 400
                else:
                    speed = self.map_value(angle, 30, 100, 400, 480)

        # Low distance override
        if lowDistance:
            n = 0
            speed = 120
            while n < 3 and self.avgSpeed.get_average() > 120:
                self.avgSpeed.add(70)
                n += 1

        # Emergency stop logic (only if Stephanie or car is in front)
        # print(f"enable_emergency_stop: {enable_emergency_stop}, stephanie_in_front: {stephanie_in_front}, car_in_front: {car_in_front}")
        if enable_emergency_stop and (stephanie_in_front or car_in_front):
            if frontDistance >= 10:
                if not self.stop:
                    if frontDistance < self.emergencyStopDistance:
                        self.consecutive_emergency += 1
                        if self.consecutive_emergency >= self.emergency_stop_threshold:
                            print(f"EMERGENCY STOP [{frontDistance}cm] Triggered after {self.consecutive_emergency} readings")
                            self.stop = True
                            self.consecutive_emergency = 0
                            self.avgSpeed.add(0)
                            return 0
                    else:
                        self.consecutive_emergency = 0
                else:
                    if frontDistance > self.emergencyStopDistance + 10:
                        print(f"EMERGENCY CLEAR [{frontDistance}cm]")
                        self.stop = False
                    else:
                        self.avgSpeed.add(0)
                        return 0
            else:
                if self.debugging:
                    print(f"Ignoring low measurement: {frontDistance}cm")

        # PID speed adjustment (only if car in front)
        pid_adjustment = 0
        if car_in_front and frontDistance <= 80:
            distanceError = self.followDistance - frontDistance
            pid_adjustment = self.pid.compute(distanceError)
        else:
            self.pid.reset()

        follow_speed = speed - pid_adjustment
        final_speed = max(0, min(min(speed, follow_speed), 500))

        if self.debugging:
            print(f"[{frontDistance}cm] PID: {int(pid_adjustment)}, Base: {int(speed)} Final: {int(final_speed)}")

        return int(self.avgSpeed.filter(final_speed))
