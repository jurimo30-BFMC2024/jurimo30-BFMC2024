from src.core.Auto.LaneFollow.MovingAverage import MovingAverage
from src.core.Auto.PID import PIDController as pid

class SpeedControl():
    def __init__(self, logging, debugging=False):
        self.logging = logging
        self.debugging = debugging
        self.avgSpeed = MovingAverage(20)
        self.pid = pid(20, 7, 0.5)
        self.followDistance = 40  # Desired following distance in cm
        self.emergencyStopDistance = 20  # Emergency stop threshold in cm
        self.emergency_stop_threshold = 3  # Consecutive readings required
        self.consecutive_emergency = 0
        self.stop = False

    def filter(self, angle, alpha=0.3):
        self.oldAngle = angle * alpha + self.oldAngle * (1 - alpha)
        return self.oldAngle

    def map_value(self, value, in_min=30, in_max=170, out_min=150, out_max=300):
        value = max(min(value, in_max), in_min)
        return out_min + (out_max - out_min) * (value - in_min) / (in_max - in_min)

    def getControlData(self, angle, stopLine, lowDistance, highway, frontDistance):
        if stopLine:
            return 65

        # Speed calculation based on highway mode and angle
        if not highway:
            if abs(angle) < 30:
                speed = 380
            elif abs(angle) > 145:
                speed = 150
            else:
                speed = self.map_value(angle, 30, 145, 200, 380)
        else:
            if abs(angle) < 70:
                speed = 580
            elif abs(angle) > 170:
                speed = 420
            else:
                speed = self.map_value(angle, 70, 170, 420, 580)

        # Low distance override
        if lowDistance:
            n = 0
            speed = 120
            while n < 3 and self.avgSpeed.get_average() > 120:
                self.avgSpeed.add(70)
                n += 1

        # Emergency stop logic
        if frontDistance >= 10:  # Only consider readings ≥10cm
            if not self.stop:
                # Update consecutive counter
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
                # Check clearance condition
                if frontDistance > self.emergencyStopDistance + 10:
                    print(f"EMERGENCY CLEAR [{frontDistance}cm]")
                    self.stop = False
                else:
                    self.avgSpeed.add(0)
                    return 0
        else:
            # Ignore readings below 10cm
            if self.debugging:
                print(f"Ignoring low measurement: {frontDistance}cm")

        # PID speed adjustment
        pid_adjustment = 0
        if frontDistance <= 80:
            distanceError = self.followDistance - frontDistance
            pid_adjustment = self.pid.update(distanceError)
        else:
            self.pid.reset()
            
        follow_speed = speed - pid_adjustment
        final_speed = max(0, min(min(speed, follow_speed), 600))
        
        if self.debugging:
            print(f"[{frontDistance}cm] PID: {int(pid_adjustment)}, Base: {int(speed)} Final: {int(final_speed)}")

        return int(self.avgSpeed.filter(final_speed))