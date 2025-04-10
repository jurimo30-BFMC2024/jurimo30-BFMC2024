import cv2
import base64
import numpy as np

from src.utils.messages.allMessages import (
    serialCamera,
    LaneDetect,
    IntersectionDetect,
    IntersectionDetect2,
    ParkingSpotDetect,
    RoundAboutAngle
)
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.ImageProcessing.LaneDetect.LaneDetector import LaneDetector
from src.ImageProcessing.LaneDetect.imagePreProcessing import ImagePreProcessing as ImgProcessor
from src.ImageProcessing.LaneDetect.StopLineDetector import StopLineDetector as StopDetect
from src.ImageProcessing.LaneDetect.ParkingSpotDetector import ParkingSpotDetector
from src.ImageProcessing.VideoStream.VideoGridStreamer import VideoStream as vs
from src.ImageProcessing.LaneDetect.RoundaboutNavigator import RoundaboutNavigator  # Import the new module


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
        self.laneDetector = LaneDetector(512, 270, logging, debugging, False)
        self.imgProcessor = ImgProcessor(512, 270, logging, debugging, False)
        self.stopLineDetector = StopDetect(512, 270, logging, debugging, False)
        self.parkingSpotDetector = ParkingSpotDetector()
        self.strm = vs(1, 0)
        self.roundAboutDetector = RoundaboutNavigator(512, 270, logging, debugging)  # Initialize RoundAboutDetector

        # Sender za slanje rezultata detekcije
        self.laneDetectionSender = messageHandlerSender(self.queuesList, LaneDetect)
        self.intersectionDetectionSender = messageHandlerSender(self.queuesList, IntersectionDetect)
        self.intersectionDetectionSender2 = messageHandlerSender(self.queuesList, IntersectionDetect2)
        self.parkingSpotDetectionSender = messageHandlerSender(self.queuesList, ParkingSpotDetect)
        self.roundAboutAngleSender = messageHandlerSender(self.queuesList, RoundAboutAngle)
        self.subscribe()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.videoSubscriber = messageHandlerSubscriber(self.queuesList, serialCamera, "LastOnly", True)

    def run(self):
        while self._running:
            try:
                videoData = self.videoSubscriber.receiveWithBlock()
                # Dekodiraj frejm iz base64
                frame = self.decode_frame(videoData)
                edges = self.imgProcessor.process_frame(frame)

                # !!!!!!!!!!!!! edges ne treba mjenjati koristi se za dalju obradu nad njim
                # !!!!!!!!!!! sva crtanja za debagovanje raditi nad frejmom on se salje na server

                # obradi frejm
                frame, intersection, intersectionA = self.stopLineDetector.process_frame(frame, edges)
                frame, angle = self.laneDetector.process_frame(frame, edges)
                frame, parking_line = self.parkingSpotDetector.process_frame(frame, edges)
                frame, roundaboutAngle = self.roundAboutDetector.process_frame(frame, edges)
                roundaboutExitDetected = False

                # Slanje rezultate
                self.laneDetectionSender.send(angle)
                self.intersectionDetectionSender.send(bool(intersection))
                self.intersectionDetectionSender2.send(bool(intersectionA))
                if parking_line is not None:
                    self.parkingSpotDetectionSender.send(True)
                self.roundAboutAngleSender.send(float(roundaboutAngle))
                self.strm.display(frame)
            except Exception as e:
                print(e)

    @staticmethod
    def decode_frame(encoded_data):
        """Decode base64 encoded frame to an OpenCV image."""
        frame_data = base64.b64decode(encoded_data)
        np_array = np.frombuffer(frame_data, np.uint8)
        frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
        return frame