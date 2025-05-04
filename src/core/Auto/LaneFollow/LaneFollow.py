from src.core.Auto.LaneFollow.MovingAverage import MovingAverage

class LaneFollow():
    """This thread handles LaneFollow.
    Args:
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, logging, debugging=False):
        self.logging = logging
        self.debugging = debugging
        self.avgAngle = MovingAverage(2)
        self.oldAngle = 0
        self.finalAngle = 0

    def filter(self, angle, alpha = 0.3):
        self.oldAngle = angle * alpha + self.oldAngle * (1 - alpha)
        return self.oldAngle
    

    def map_value(self, value, in_min=30, in_max=170, out_min=150, out_max=300):
        value = max(min(value, in_max), in_min)
        return out_min + (out_max - out_min) * (value - in_min) / (in_max - in_min)

    def getControlData(self, highway, stop_line, input_angle):
        angle = int(input_angle * 10)

        self.finalAngle = -angle


        return self.finalAngle