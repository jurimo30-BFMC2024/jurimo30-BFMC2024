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
        self.model = YOLO('src/ImageProcessing/ObjectDetection/threads/yolo_version_2.7/detect/train/weights/best.pt')
        self.streamer = VideoStream(0, 0)
        
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
                frame_cropped = self.crop_frame(frame)
                frame_cropped = cv2.resize(frame_cropped, (256,256), interpolation=cv2.INTER_AREA)
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
        cv2.rectangle(frame, (200, 200), (255, 255), (150, 255, 150), 3)

        h, w = frame.shape[:2]
        center_box = (w // 2 - 30, 0, w // 2 + 30, h)  # 30x30 center region

        best_sign = None
        detections = []

        for box in results.boxes.data:
            x1, y1, x2, y2, conf, cls = box.tolist()
            label = self.model.model.names[int(cls)]

            # Precompute values
            box_cx = (x1 + x2) / 2
            box_cy = (y1 + y2) / 2
            area = (x2 - x1) * (y2 - y1)
            in_center = (center_box[0] <= box_cx <= center_box[2]) and (center_box[1] <= box_cy <= center_box[3])

# Filter detections based on label and conditions
            if label == "car":
                if conf > 0.75 and in_center:
                    detections.append((conf, area, label, (x1, y1, x2, y2)))
                    color = (0, 0, 255)  # Red for cars
            elif label == "exit": 
                if conf > 0.15 and (y2 > 180 and x2 > 180):
                    detections.append((conf, area, label, (x1, y1, x2, y2)))
                    color = (0, 255, 0)  # Green for exit
            elif conf > 0.75:
                detections.append((conf, area, label, (x1, y1, x2, y2)))
                color = (0, 204, 255)  # Yellow for other objects
            else:
                continue

            # Draw bounding box and label
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(frame, f"{label} {conf:.2f}", 
                       (int(x1) + 5, int(y1) + 15), cv2.FONT_HERSHEY_SIMPLEX,  0.4, color, 2)

        if detections:
            detections.sort(key=lambda x: (-x[0], -x[1]))
            best_sign = detections[0][2]

        return frame, best_sign


    def update_state(self, new_sign):
        """Update detection state and handle messaging."""
        # Reset counters if no sign detected
        if new_sign is None:
            self.lost_sign_count += 1
            self.confirmation_counter = 0
            if self.lost_sign_count >= self.lost_sign_threshold:
                #if self.current_sign is not None:
                    #self.objectDetectionSender.send(None)  # Signal sign lost
                self.current_sign = None
                self.lost_sign_count = 0
            return

        # Case 1: New potential sign while we have current sign
        if self.current_sign is not None:
            if new_sign == self.current_sign:
                # Reset counters for current sign
                self.lost_sign_count = 0
                self.confirmation_counter = 0
            else:
                # Track confirmation for new candidate
                self.confirmation_counter += 1
                
                # If new candidate confirmed before losing current
                if self.confirmation_counter >= self.confirmation_threshold:
                    # Send both lost and new sign
                    #self.objectDetectionSender.send(None)    # Signal previous lost
                    self.objectDetectionSender.send(new_sign)  # Send new sign
                    self.current_sign = new_sign
                    self.confirmation_counter = 0
                    self.lost_sign_count = 0

        # Case 2: No current sign, new detection
        else:
            if new_sign:
                self.confirmation_counter += 1
                # Confirm new sign
                if self.confirmation_counter >= self.confirmation_threshold:
                    self.objectDetectionSender.send(new_sign)
                    self.current_sign = new_sign
                    self.confirmation_counter = 0
                    self.lost_sign_count = 0

    @staticmethod
    def decode_frame(encoded_data):
        """Decode base64 encoded frame to OpenCV image."""
        frame_data = base64.b64decode(encoded_data)
        np_array = np.frombuffer(frame_data, np.uint8)
        return cv2.imdecode(np_array, cv2.IMREAD_COLOR)

    @staticmethod
    def crop_frame(frame):
        """Crop top-right quadrant of frame."""
        h, _ = frame.shape[:2]
        return frame[0:h-63, :]