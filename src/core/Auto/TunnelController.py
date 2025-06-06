from src.core.Auto.PIDController import PIDController

class TunnelController:
    def __init__(self):
        self.pid = PIDController(kp=0.4, ki=0.01, kd=0.0, kaw = 0, output_limits=(-25, 25))

    # iskreno ne znam dal treba jos nesto da se doda
    def getControlData(self, distance_from_right_wall):
        error = distance_from_right_wall - 10.0
        return self.pid.compute(error)