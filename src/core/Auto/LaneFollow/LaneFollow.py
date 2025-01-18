from src.utils.messages.allMessages import (
    LaneDetect,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.core.Auto.LaneFollow.MovingAverage import MovingAverage as ma

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
        self.avgSpeed = ma(20)
        self.avgAngle = ma(2)
        self.subscribe()

    def map_value(self, value, in_min=30, in_max=170, out_min=220, out_max=80):
        value = max(min(value, in_max), in_min)
        return out_min + (out_max - out_min) * (value - in_min) / (in_max - in_min)

    def getControlData(self):
        angle = int(self.laneDetectSubscriber.receiveWithBlock() * 10)
        
        if abs(angle) < 30:
            speed = 280
        elif abs(angle) > 170:
            speed = 100
        else:
            speed = self.map_value(angle)

        if angle > 250:
            angle = 240
        if angle < -250:
            angle = -240

        if self.debugging:
            self.logging.info(f"Lane detect out: {angle}")

        speed = int(self.avgSpeed.filter(speed))
        # angle = int(self.avgAngle.filter(angle))

        return angle, speed

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.laneDetectSubscriber = messageHandlerSubscriber(self.queuesList, LaneDetect, "LastOnly", True)
