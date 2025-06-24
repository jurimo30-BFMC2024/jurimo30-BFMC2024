if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")


from src.core.Auto.LaneFollow.MovingAverage import MovingAverage
from src.core.Auto.PIDController import PIDController

import time

class SpeedControl:
    # Class-level constants for configuration
    AVG_WINDOW = 20
    PID_KP = 20
    PID_KI = 7
    PID_KD = 0.5
    FOLLOW_DISTANCE = 50  # cm
    EMERGENCY_STOP_DISTANCE = 35  # cm
    EMERGENCY_STOP_THRESHOLD = 3  # readings
    LOW_DISTANCE_SPEED = 120
    STOP_LINE_SPEED = 65
    MAX_SPEED = 500

    HIGHWAY_LOWER_SPEED_NODES = {
        "398", "399", "400", "401", "444", "445", "446", "447",
        "421", "422", "423", "463", "464", "465", "466",
        "426", "425", "424", "462", "461", "460",
        "479", "480", "481", "482", "402", "403", "404"
        # Add specific nodes on highway where lower speed is required
    }

    def __init__(self, logging, debugging: bool = False):
        self.logging = logging
        self.debugging = debugging
        self.avgSpeed = MovingAverage(self.AVG_WINDOW)
        self.pid = PIDController(self.PID_KP, self.PID_KI, self.PID_KD)
        self.consecutive_emergency = 0
        self.stop = False

    def map_value(self, value, in_min=30, in_max=170, out_min=150, out_max=300):
        value = max(min(value, in_max), in_min)
        return out_min + (out_max - out_min) * (value - in_min) / (in_max - in_min)

    def _calc_speed_normal(self, angle: float) -> float:
        abs_angle = abs(angle)
        if abs_angle < 30:
            return 350
        elif abs_angle > 145:
            return 220
        else:
            return self.map_value(angle, 30, 145, 280, 350)

    def _calc_speed_highway(self, angle: float, current_node: str | None) -> float:
        if current_node in self.HIGHWAY_LOWER_SPEED_NODES:
            return 280
        abs_angle = abs(angle)
        if abs_angle < 30:
            return 480
        elif abs_angle > 100:
            return 400
        else:
            return self.map_value(angle, 30, 100, 400, 480)

    def getControlData(
        self,
        angle: float,
        stopLine: bool,
        lowDistance: bool,
        highway: bool,
        frontDistance: float,
        enable_emergency_stop: bool,
        car_in_front: bool,
        stephanie_in_front: bool,
        current_node: str | None
    ) -> int:
        if stopLine:
            return self.STOP_LINE_SPEED

        # Speed calculation based on highway mode and angle
        speed = (
            self._calc_speed_highway(angle, current_node)
            if highway else self._calc_speed_normal(angle)
        )

        # Low distance override
        if lowDistance:
            n = 0
            speed = self.LOW_DISTANCE_SPEED
            while n < 3 and self.avgSpeed.get_average() > self.LOW_DISTANCE_SPEED:
                self.avgSpeed.add(70)
                n += 1

        # Emergency stop logic (only if Stephanie or car is in front)
        if enable_emergency_stop:
            if car_in_front:
                if frontDistance >= 10:
                    if not self.stop:
                        if frontDistance < self.EMERGENCY_STOP_DISTANCE:
                            self.consecutive_emergency += 1
                            if self.consecutive_emergency >= self.EMERGENCY_STOP_THRESHOLD:
                                print(f"EMERGENCY STOP [{frontDistance}cm] Triggered after {self.consecutive_emergency} readings")
                                self.stop = True
                                self.consecutive_emergency = 0
                                self.avgSpeed.add(0)
                                return 0
                        else:
                            self.consecutive_emergency = 0
                    else:
                        if frontDistance > self.EMERGENCY_STOP_DISTANCE + 10:
                            print(f"EMERGENCY CLEAR [{frontDistance}cm]")
                            self.stop = False
                        else:
                            self.avgSpeed.add(0)
                            return 0
                elif self.debugging:
                    print(f"Ignoring low measurement: {frontDistance}cm")

            elif stephanie_in_front:
                print(f"Stephanie in front, emergency stop triggered")
                self.avgSpeed.add(0)
                return 0

        # PID speed adjustment (only if car in front)
        pid_adjustment = 0
        if car_in_front and frontDistance <= 80:
            distanceError = self.FOLLOW_DISTANCE - frontDistance
            pid_adjustment = self.pid.compute(distanceError)
        else:
            self.pid.reset()

        follow_speed = speed - pid_adjustment
        final_speed = max(0, min(min(speed, follow_speed), self.MAX_SPEED))

        if self.debugging:
            print(f"[{frontDistance}cm] PID: {int(pid_adjustment)}, Base: {int(speed)} Final: {int(final_speed)}")

        return int(self.avgSpeed.filter(final_speed))

if __name__ == "__main__":
    # Dummy logging function
    def dummy_log(msg):
        print(f"LOG: {msg}")

    # Instantiate SpeedControl with debugging enabled
    sc = SpeedControl(logging=dummy_log, debugging=True)

    # Test 1: Normal drive state
    print("=== Normal Drive State ===")
    speed = sc.getControlData(
        angle=10,
        stopLine=False,
        lowDistance=False,
        highway=False,
        frontDistance=100,
        enable_emergency_stop=False,
        car_in_front=False,
        stephanie_in_front=False,
        current_node=None
    )
    print(f"Speed (normal drive): {speed}")

    # Test 2: Highway state with nodes from HIGHWAY_LOWER_SPEED_NODES
    print("\n=== Highway State (LOWER SPEED NODES) ===")
    for node in list(sc.HIGHWAY_LOWER_SPEED_NODES)[:2]:
        speed = sc.getControlData(
            angle=10,
            stopLine=False,
            lowDistance=False,
            highway=True,
            frontDistance=100,
            enable_emergency_stop=False,
            car_in_front=False,
            stephanie_in_front=False,
            current_node=node
        )
        print(f"Speed (highway, node {node}): {speed}")

    # Test 3: Highway state with nodes outside HIGHWAY_LOWER_SPEED_NODES
    print("\n=== Highway State (OTHER NODES) ===")
    for node in ["999", "888"]:
        speed = sc.getControlData(
            angle=0,
            stopLine=False,
            lowDistance=False,
            highway=True,
            frontDistance=100,
            enable_emergency_stop=False,
            car_in_front=False,
            stephanie_in_front=False,
            current_node=node
        )
        print(f"Speed (highway, node {node}): {speed}")
