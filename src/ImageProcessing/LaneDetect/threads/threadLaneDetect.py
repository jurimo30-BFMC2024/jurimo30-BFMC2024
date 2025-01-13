import cv2
import base64
import numpy as np

from src.utils.messages.allMessages import (
    mainCamera,
    LaneDetect,
)

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.ImageProcessing.LaneDetect.LaneDetector import LaneDetector

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
        self.detector = LaneDetector(512, 270)

        # Sender za slanje rezultata detekcije
        self.laneDetectionSender = messageHandlerSender(self.queuesList, LaneDetect)
        self.subscribe()
        
    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.videoSubscriber = messageHandlerSubscriber(self.queuesList, mainCamera, "LastOnly", True)

    def run(self):
        while self._running:
            try:
                videoData = self.videoSubscriber.receiveWithBlock()
                # Dekodiraj frejm iz base64
                frame = self.decode_frame(videoData)

                # obradi frejm
                angle = self.detector.process_frame(frame)

                # Slanje rezultate
                self.laneDetectionSender.send(angle)
            except Exception as e:
                print(e)

    @staticmethod
    def decode_frame(encoded_data):
        """Decode base64 encoded frame to an OpenCV image."""
        frame_data = base64.b64decode(encoded_data)
        np_array = np.frombuffer(frame_data, np.uint8)
        frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
        return frame