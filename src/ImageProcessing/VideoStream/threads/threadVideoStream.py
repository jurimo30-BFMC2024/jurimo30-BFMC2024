from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (serialCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.ImageProcessing.VideoStream.VideoGridStreamer import VideoGridStreamer
from threading import Thread

class threadVideoStream(ThreadWithStop):
    """This thread handles VideoStream.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, streamer: VideoGridStreamer, logging, debugging=False):
        self.queuesList = queueList
        self.streamer = streamer
        self.logging = logging
        self.debugging = debugging
        self.subscribe()
        super(threadVideoStream, self).__init__()

        self.cameraSubscribers

    def run(self):
        pass

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        pass
