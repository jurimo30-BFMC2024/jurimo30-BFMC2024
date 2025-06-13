import cv2
import base64
import numpy as np
import time

from src.utils.messages.allMessages import (
    serialCamera,
    LaneDetect,
    StopLineDetect,
    ParkingSpotDetect
)
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.ImageProcessing.LaneDetect.LaneDetector import LaneDetector
from src.ImageProcessing.LaneDetect.imagePreProcessing import ImagePreProcessing as ImgProcessor
from src.ImageProcessing.LaneDetect.StopLineDetector import StopLineDetector as StopDetect
from src.ImageProcessing.LaneDetect.ParkingSpotDetector import ParkingSpotDetector
from src.ImageProcessing.VideoStream.VideoGridStreamer import VideoStream as vs
from src.hardware.camera.encoder import decode_frame

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
        self.laneDetector = LaneDetector(512, 270, logging, False, False)
        self.imgProcessor = ImgProcessor(512, 270, logging, debugging, False)
        self.stopLineDetector = StopDetect(512, 270, logging, False, False)
        self.parkingSpotDetector = ParkingSpotDetector()
        self.strm = vs(1, 0)


        # Sender za slanje rezultata detekcije
        self.laneDetectionSender = messageHandlerSender(self.queuesList, LaneDetect)
        self.stopLineDetectionSender = messageHandlerSender(self.queuesList, StopLineDetect)
        self.parkingSpotDetectionSender = messageHandlerSender(self.queuesList, ParkingSpotDetect)
        self.subscribe()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.videoSubscriber = messageHandlerSubscriber(self.queuesList, serialCamera, "LastOnly", True)

    def run(self):
        frame_count = 0
        start_time_second = time.time()

        while self._running:
            try:
                videoData = self.videoSubscriber.receiveWithBlock()
                # Dekodiraj frejm iz base64
                frame = decode_frame(videoData)

                # Process frame
                edges = self.imgProcessor.process_frame(frame)

                # obradi frejm
                frame, stop_line_data = self.stopLineDetector.process_frame(frame, edges)
                frame, leftX, rightX, leftVisible, rightVisible = self.laneDetector.process_frame(edges, frame)
                frame, parking_line = self.parkingSpotDetector.process_frame(frame, edges)

                # Increment frame count
                frame_count += 1

                # Check if one second has passed
                if time.time() - start_time_second >= 1:
                    frame_count = 0
                    start_time_second = time.time()

                # Slanje rezultate
                self.stopLineDetectionSender.send(stop_line_data)
                self.laneDetectionSender.send((leftX, rightX, leftVisible, rightVisible))
                if parking_line is not None:
                    self.parkingSpotDetectionSender.send(True)  
                self.strm.display(frame)
            except Exception as e:
                print(e)