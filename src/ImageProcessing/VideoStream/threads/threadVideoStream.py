from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (serialCamera, mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.ImageProcessing.VideoStream.VideoGridStreamer import VideoGridStreamer
from threading import Thread
from multiprocessing import Process
import numpy as np
import cv2
import base64
import time

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
        self.displayThreads = [
            Thread(target=self.displayRawCamera, daemon=True, kwargs={
                "subscriber": messageHandlerSubscriber(self.queuesList, serialCamera, "lastOnly", True), 
                "row": 0,
                "col": 0,
            }),
            Thread(target=self.displayRawCamera, daemon=True, kwargs={
                "subscriber": messageHandlerSubscriber(self.queuesList, mainCamera, "lastOnly", True), 
                "row": 1,
                "col": 0,
            })
        ]

    @staticmethod
    def decode_frame(encoded_data):
        """Decode base64 encoded frame to an OpenCV image."""
        frame_data = base64.b64decode(encoded_data)
        np_array = np.frombuffer(frame_data, np.uint8)
        frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
        return frame
    
    def displayRawCamera(self, subscriber: messageHandlerSubscriber, row: int, col: int):
        while self._running:
            videoData = subscriber.receiveWithBlock()

            frame = self.decode_frame(videoData)

            self.streamer.display_frame(frame, row=row, col=col)
            # time.sleep(0.05)

    def run(self):
        for th in self.displayThreads:
            th.start()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        pass
