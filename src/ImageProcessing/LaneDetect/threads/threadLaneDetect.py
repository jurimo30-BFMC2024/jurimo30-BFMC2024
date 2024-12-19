from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
class threadLaneDetect(ThreadWithStop):
    """This thread handles LaneDetect.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        super(threadLaneDetect, self).__init__()
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        # Tvoj rad ovde
        

        self.subscribe()
        

    def run(self):
        while self._running:
            pass

    def subscribe(self):
        """Subscribes to the messages you are interested in"""

