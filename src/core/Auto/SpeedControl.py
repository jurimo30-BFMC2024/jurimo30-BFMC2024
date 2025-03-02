from src.core.Auto.LaneFollow.MovingAverage import MovingAverage
from src.core.Auto.PID import PIDController as pid

class SpeedControl():
    def __init__(self, logging, debugging=False):
        self.logging = logging
        self.debugging = debugging
        self.avgSpeed = MovingAverage(20)
        self.pid = pid(10, 7, 0.5)
        self.followDistance = 60  # Desired following distance in cm
        self.emergencyStopDistance = 30  # Desired following distance in cm
        self.state = "normal"
        self.stopped = False
        self.oldAngle = 0

    def filter(self, angle, alpha=0.3):
        self.oldAngle = angle * alpha + self.oldAngle * (1 - alpha)
        return self.oldAngle

    def map_value(self, value, in_min=30, in_max=170, out_min=150, out_max=300):
        value = max(min(value, in_max), in_min)
        return out_min + (out_max - out_min) * (value - in_min) / (in_max - in_min)

    def getControlData(self, angle, stopLine, lowDistance, highway, frontDistance):
        # Handle highest priority state transitions first
        if stopLine:
            self.state = "stop_line"
        
        if frontDistance < self.emergencyStopDistance:
            self.state = "emergency_stop"
            self.stopped = True
        elif self.stopped and frontDistance < self.emergencyStopDistance + 10:
            self.state = "holding_stop"
        elif self.stopped:
            self.stopped = False
            self.state = "normal"

        # Process current state
        if self.state == "stop_line":
            self.state = "normal"  # Reset state after stop line
            return 65

        elif self.state == "emergency_stop":
            self.avgSpeed.add(0)
            print("Emergency stop")
            return 0

        elif self.state == "holding_stop":
            print("Holding stop")
            return 0

        elif self.state == "normal" or self.state == "following":
            # Check for transitions to low_speed or following
            if lowDistance:
                self.state = "low_speed"
            elif frontDistance <= 80:
                self.state = "following"
            else:
                self.state = "normal"

        # Compute speed based on current state
        speed = 0
        if self.state == "low_speed":
            speed = 50
            n = 0
            while n < 3 and self.avgSpeed.get_average() > 120:
                self.avgSpeed.add(70)
                n += 1
        else:
            # Calculate base speed based on highway and angle
            if not highway:
                if abs(angle) < 30:
                    speed = 380
                elif abs(angle) > 170:
                    speed = 220
                else:
                    speed = self.map_value(angle, 30, 170, 220, 380)
            else:
                if abs(angle) < 70:
                    speed = 580
                elif abs(angle) > 170:
                    speed = 420
                else:
                    speed = self.map_value(angle, 70, 170, 420, 580)

            # Apply PID adjustment if in following state
            if self.state == "following":
                distanceError = self.followDistance - frontDistance
                pid_adjustment = self.pid.update(distanceError)
                follow_speed = speed - pid_adjustment
                speed = max(0, min(min(speed, follow_speed), 600))
            else:
                self.pid.reset()

        # Filter and return the speed
        filtered_speed = int(self.avgSpeed.filter(speed))
        print(f"Return speed: {filtered_speed}")
        return filtered_speed