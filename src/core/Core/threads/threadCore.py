from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (DrivingMode)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from Manual.manualControlMode import manualControlMode
from Stop.stopControlMode import stopControlMode
from core.Auto.autoFSM import autoFSM
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
        self.mode = "stop"
        self.subscribe()
        super(threadCore, self).__init__()
        self.manualMode = manualControlMode(queueList, logging, debugging)
        self.stopMode = stopControlMode(queueList, logging, debugging)
        self.autoMode = autoFSM(queueList, logging, debugging)

    def run(self):
        while self._running:
            mode = self.drivingModeSubscriber.receive()
            if mode is not None:
                if self.debugging:
                    self.logging.info(mode)
                    self.mode = mode

            if(self.mode == "stop"):
                self.stopMode.run()
            elif(self.mode == "manual"):
                self.manualMode.run()
            elif(self.mode == "auto"):
                self.autoMode.run()
            elif(self.mode == "legacy"):
                self.mode = "stop"
                if self.debugging:
                    self.logging.error("Unsupported driving mode")

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.drivingModeSubscriber = messageHandlerSubscriber(self.queuesList, DrivingMode, "LastOnly", True)
        pass
