from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (DrivingMode)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.core.Manual.manualControlMode import manualControlMode
from src.core.Stop.stopControlMode import stopControlMode
from src.core.Auto.autoFSM import autoFSM
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
        self.modes = {
            "stop": stopControlMode(queueList, logging, debugging),
            "manual": manualControlMode(queueList, logging, debugging),
            "auto": autoFSM(queueList, logging, debugging),
        }
        self.mode = "stop"
        self.modes[self.mode].start()

    def run(self):
        while self._running:
            mode = self.drivingModeSubscriber.receiveWithBlock()
            if mode not in self.modes:
                if self.debugging:
                    self.logging.error(f"Unsupported driving mode: {mode}")
                mode = "stop"

            if mode != self.mode:
                self.modes[self.mode].stop()
                self.modes[mode].start()
                self.mode = mode
                if self.debugging:
                    self.logging.info(f"Selected driving mode: {mode}")
                    
                    

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.drivingModeSubscriber = messageHandlerSubscriber(self.queuesList, DrivingMode, "LastOnly", True)
        pass
