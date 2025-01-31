from src.utils.messages.allMessages import (
    LaneDetect,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.core.Auto.LaneFollow.MovingAverage import MovingAverage as ma
from src.core.Auto.PID import PIDController as pid

class LaneFollow():
    """This thread handles LaneFollow.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.avgAngle = ma(5)
        self.subscribe()
        self.pid = pid(0.5, 0.3, 0)
        self.oldAngle = 0

    def filter(self, angle, alpha = 0.3):
        self.oldAngle = angle * alpha + self.oldAngle * (1 - alpha)
        return self.oldAngle
    

    def map_value(self, value, in_min=30, in_max=170, out_min=150, out_max=300):
        value = max(min(value, in_max), in_min)
        return out_min + (out_max - out_min) * (value - in_min) / (in_max - in_min)

    def getControlData(self):
        angle = int(self.laneDetectSubscriber.receiveWithBlock() * 10)

        self.finalAngle = angle

        if self.finalAngle > 250:
            self.finalAngle = 240
        if self.finalAngle < -250:
            self.finalAngle = -240

        if self.debugging:
            self.logging.info(f"Lane detect out: {self.finalAngle}")

        #self.finalAngle = int(self.avgAngle.filter(self.finalAngle))

        return self.finalAngle

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.laneDetectSubscriber = messageHandlerSubscriber(self.queuesList, LaneDetect, "LastOnly", True)
