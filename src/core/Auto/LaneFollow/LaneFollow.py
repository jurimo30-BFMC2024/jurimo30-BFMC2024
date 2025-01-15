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
        self.avgSpeed = ma.MovingAverage(10)
        self.avgAngle = ma.MovingAverage(10)
        self.subscribe()

    def map_value(value, in_min=50, in_max=180, out_min=250, out_max=80):
        value = max(min(value, in_max), in_min)
        return out_min + (out_max - out_min) * (value - in_min) / (in_max - in_min)

    def getControlData(self):
        angle = int(self.laneDetectSubscriber.receiveWithBlock() * 20)
        
        if abs(angle) < 50:
            speed = 250
        elif abs(angle) > 180:
            speed = 80
        else:
            speed = self.map_value(angle)

        if self.debugging:
            self.logging.info(f"Lane detect out: {angle}")

        if angle > 240:
            angle = 240
        if angle < -240:
            angle = -240

        speed = self.avgSpeed(speed)
        angle = self.avgAngle(angle)

        return angle, speed

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.laneDetectSubscriber = messageHandlerSubscriber(self.queuesList, LaneDetect, "LastOnly", True)
