from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (serialCamera, mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.ImageProcessing.VideoStream.VideoGridStreamer import VideoStream
from src.hardware.camera.encoder import decode_frame
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

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.subscribe()
        super(threadVideoStream, self).__init__()
        self.displayThreads = [
            # Thread(target=self.displayRawCamera, daemon=True, kwargs={
            #     "subscriber": messageHandlerSubscriber(self.queuesList, serialCamera, "lastOnly", True), 
            #     "row": 0,
            #     "col": 0,
            # }),
            # Thread(target=self.displayRawCamera, daemon=True, kwargs={
            #     "subscriber": messageHandlerSubscriber(self.queuesList, mainCamera, "lastOnly", True), 
            #     "row": 1,
            #     "col": 0,
            # })
        ]
    
    def displayRawCamera(self, subscriber: messageHandlerSubscriber, row: int, col: int):
        streamer = VideoStream(row, col)
        while self._running:
            videoData = subscriber.receiveWithBlock()

            frame = decode_frame(videoData)

            streamer.display(frame)
            time.sleep(0.1)

    def run(self):
        for th in self.displayThreads:
            th.start()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        pass
