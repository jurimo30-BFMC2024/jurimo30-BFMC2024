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
        queueList (dict of multiprocessing.queues.Queue): Dictionary of queues.
        logging (logging object): For debugging.
        debugging (bool, optional): Debugging flag. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.model = YOLO('src/ImageProcessing/ObjectDetection/threads/runs /content/runs/detect/train/weights/best.pt')
        self.streamer = VideoStream(0, 1)
        
        # State management variables
        self.current_sign = None          # Currently active sign
        self.confirmation_counter = 0     # Frames with consistent new sign
        self.confirmation_threshold = 3   # Frames needed to confirm new sign
        self.lost_sign_count = 0          # Frames without current sign
        self.lost_sign_threshold = 17     # Frames to consider sign lost
        
        super(threadObjectDetection, self).__init__()
        self.objectDetectionSender = messageHandlerSender(self.queuesList, ObjectDetection)
        self.subscribe()

    def subscribe(self):
        """Subscribes to required messages."""
        self.videoSubscriber = messageHandlerSubscriber(self.queuesList, serialCamera, "LastOnly", True)

    def run(self):
        while self._running:
            try:
                videoData = self.videoSubscriber.receiveWithBlock()
                frame = self.decode_frame(videoData)
                frame_cropped = self.crop_top_right(frame)
                
                # Process frame and get best detection
                processed_frame, best_sign = self.process_frame(frame_cropped)
                
                # Update state and send messages
                self.update_state(best_sign)
                
                self.streamer.display(processed_frame)

            except Exception as e:
                print(e)

    def process_frame(self, frame):
        """Process frame and return annotated frame with best detection."""
        # Get YOLO results
        results = self.model(frame, verbose=self.debugging)[0]
        
        # Find best detection
        best_sign = None
        detections = []
        for box in results.boxes.data:
            x1, y1, x2, y2, conf, cls = box.tolist()
            if conf > 0.75:
                label = self.model.model.names[int(cls)]
                area = (x2 - x1) * (y2 - y1)
                detections.append((conf, area, label))
        
        if detections:
            # Sort by confidence (desc), then area (desc)
            detections.sort(key=lambda x: (-x[0], -x[1]))
            best_sign = detections[0][2]

        # Annotate frame
        return self.annotate_boxes(frame, results), best_sign

    def update_state(self, new_sign):
        """Update detection state and handle messaging."""
        if self.current_sign:
            if new_sign == self.current_sign:
                self.lost_sign_count = 0
            else:
                self.lost_sign_count += 1
                if self.lost_sign_count >= self.lost_sign_threshold:
                    self.objectDetectionSender.send(None)  # Sign lost
                    self.current_sign = None
                    self.lost_sign_count = 0
        else:
            if new_sign:
                self.confirmation_counter += 1
                if self.confirmation_counter >= self.confirmation_threshold:
                    self.objectDetectionSender.send(new_sign)
                    self.current_sign = new_sign
                    self.confirmation_counter = 0
            else:
                self.confirmation_counter = 0

    @staticmethod
    def decode_frame(encoded_data):
        """Decode base64 encoded frame to OpenCV image."""
        frame_data = base64.b64decode(encoded_data)
        np_array = np.frombuffer(frame_data, np.uint8)
        return cv2.imdecode(np_array, cv2.IMREAD_COLOR)

    @staticmethod
    def crop_top_right(frame):
        """Crop top-right quadrant of frame."""
        h, w = frame.shape[:2]
        return frame[:h//2, w//2:]

    def annotate_boxes(self, frame, results):
        """Draw detection boxes on frame."""
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            label = self.model.model.names[int(box.cls[0])]
            conf = box.conf[0].item()
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", 
                       (x1+5, y1+15), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.4, (0, 255, 0), 2)
        return frame
