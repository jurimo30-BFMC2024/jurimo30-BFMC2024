import cv2
import base64
import numpy as np
import os

os.environ['OMP_NUM_THREADS'] = "2"
os.environ['MKL_NUM_THREADS'] = "2"

from ultralytics import YOLO

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    serialCamera,
    ObjectDetection
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.ImageProcessing.VideoStream.VideoGridStreamer import VideoStream

class threadObjectDetection(ThreadWithStop):
    """This thread handles ObjectDetection.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.model = YOLO('src/ImageProcessing/ObjectDetection/threads/runs /content/runs/detect/train/weights/best.pt')
        self.streamer = VideoStream(0, 1)
        super(threadObjectDetection, self).__init__()
        
        # Sender
        self.objectDetectionSender = messageHandlerSender(self.queuesList, ObjectDetection)
        self.subscribe()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.videoSubscriber = messageHandlerSubscriber(self.queuesList, serialCamera, "LastOnly", True)

    def run(self):
        while self._running:
            try:
                videoData = self.videoSubscriber.receiveWithBlock()
                # Decode frame from base64
                frame = self.decode_frame(videoData)

                # Crop frame to top-right quadrant
                frame_cropped = self.crop_top_right(frame)

                # Process cropped frame with YOLO
                objects = self.main(frame_cropped)

                # Display the original frame with annotations
                self.streamer.display(objects)

            except Exception as e:
                print(e)
    
    @staticmethod
    def decode_frame(encoded_data):
        """Decode base64 encoded frame to an OpenCV image."""
        frame_data = base64.b64decode(encoded_data)
        np_array = np.frombuffer(frame_data, np.uint8)
        frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
        return frame

    @staticmethod
    def crop_top_right(frame):
        """Crop the top-right quadrant of the frame."""
        height, width, _ = frame.shape
        return frame[:height // 2, width // 2:]

    def annotate_boxes(self, frame, results, model):
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cls = int(box.cls[0])
            conf = box.conf[0].item()

            # Use model.model.names for class name
            label = f"{model.model.names[cls]}: {conf:.2f}"
            
            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1+5, y1+15), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 0), 2)
            
        return frame

    def main(self, frame):
        results = self.model(frame)[0]
        
        # Annotate boxes on the frame
        frame = self.annotate_boxes(frame=frame, results=results, model=self.model)
        
        # Generate labels for detections (display class name and confidence)
        labels = [
            f"{self.model.model.names[int(cls)]} {conf:0.2f}"
            for cls, conf in zip(results.boxes.cls, results.boxes.conf)
        ]

        # Print class names and confidence to console
        for label in labels:
            print(label)

        return frame
