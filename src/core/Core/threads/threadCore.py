from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from Manual.manualControlMode import manualControlMode
class threadCore(ThreadWithStop):
    """This thread handles Core.
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
        super(threadCore, self).__init__()
        self.manualMode = manualControlMode(queueList, logging, debugging)

    def run(self):
        while self._running:
            self.manualMode.run()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        pass
