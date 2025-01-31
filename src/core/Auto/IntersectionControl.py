from src.utils.messages.allMessages import (
    LaneDetect,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.core.Auto.LaneFollow.MovingAverage import MovingAverage as ma
from src.core.Auto.PID import PIDController as pid

class IntersectionControl():
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

        self.subscribe()

    def getControlData(self):
        
        return self.finalAngle

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.laneDetectSubscriber = messageHandlerSubscriber(self.queuesList, LaneDetect, "LastOnly", True)
