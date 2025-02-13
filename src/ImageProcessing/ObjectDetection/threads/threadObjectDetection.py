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
from src.ImageProcessing.VideoStream.VideoGridStreamer import VideoStream as vs

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
        self.obj_det_signs = YOLO('src/ImageProcessing/ObjectDetection/threads/runs_road_signs/detect/train/weights/best.pt')
        self.obj_det_steph = YOLO('src/ImageProcessing/ObjectDetection/threads/runs_steph/runs/detect/train/weights/best.pt')
        self.streamer = vs(0, 1)

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
                frame_cp = frame.copy()

                # Obrada znaka (gornja desna polovina slike)
                frame_cropped = self.crop_top_right(frame_cp)
                processed_frame, best_sign = self.process_frame_signs(frame_cropped)

                # Obrada Stephany (cela slika)
                processed_frame, best_steph = self.process_frame_steph(processed_frame)

                # Slanje podataka o znaku i Stephany
                self.update_state(best_sign)
                if best_steph:  
                    self.objectDetectionSender.send(best_steph)  # Slanje ako je Stephany detektovana

                # Prikaz obrađenog frejma
                self.streamer.display(processed_frame)

            except Exception as e:
                print(e)

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

    def process_frame_signs(self, frame):
        """Obrada detekcije znakova."""
        results = self.obj_det_signs(frame, verbose=self.debugging)[0]
        return self.process_detections(frame, results, self.obj_det_signs)

    def process_frame_steph(self, frame):
        """Obrada detekcije Stephany."""
        results = self.obj_det_steph(frame, verbose=self.debugging)[0]
        return self.process_detections(frame, results, self.obj_det_steph)

    def process_detections(self, frame, results, model):
        """Procesuiranje detekcija, anotiranje i odabir najboljeg objekta."""
        best_detection = None
        detections = []

        for box in results.boxes.data:
            x1, y1, x2, y2, conf, cls = box.tolist()
            if conf > 0.75:
                label = model.model.names[int(cls)]
                area = (x2 - x1) * (y2 - y1)
                detections.append((conf, area, label, (x1, y1, x2, y2)))

        if detections:
            detections.sort(key=lambda x: (-x[0], -x[1]))  # Sortiranje po pouzdanosti, pa po veličini
            best_detection = detections[0][2]  # Labela najboljeg objekta
            frame = self.annotate_boxes(frame, detections)

        return frame, best_detection

    def annotate_boxes(self, frame, detections):
        """Crtanje okvira i oznaka na frejmu."""
        for conf, area, label, (x1, y1, x2, y2) in detections:
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (int(x1)+5, int(y1)+15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
        return frame