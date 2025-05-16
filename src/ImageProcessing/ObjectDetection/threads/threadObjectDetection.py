import cv2
import base64
import numpy as np
import os
import time

os.environ['OMP_NUM_THREADS'] = "2"
os.environ['MKL_NUM_THREADS'] = "2"

from ultralytics import YOLO

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    serialCamera,
    ObjectDetection,
    TrafficSignsDetection
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
    """
    - Razdvojiti objekte(auto, exit, stefanija) od znakova
    - Poslati kada se objekat pojavi i kada nestane iz frejma
    - Slati koordinate box-a oko objekta na slici dok je u frejmu
    - Logika stanja znakova je bila dobra (trazio si sa najvecom povrsinom) i
      slao samo taj jedan znak po frejmu, ali ostale objekte moras poslati sve koje vidis
      
    """
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.model = YOLO('src/ImageProcessing/ObjectDetection/threads/yolo_version_2.9/detect/train/weights/best.pt')
        self.streamer = VideoStream(0, 0)
        
        # State management variables
        self.current_sign = None          # Currently active sign
        self.confirmation_counter = 0     # Frames with consistent new sign
        self.confirmation_threshold = 3   # Frames needed to confirm new sign
        self.lost_sign_count = 0          # Frames without current sign
        self.lost_sign_threshold = 17     # Frames to consider sign lost
        
        self.lost_timeout = 1             # Timeout for lost objects
   
        # Initialize relevant_objects structure
        self.relevant_objects = {
            "car": {"position": None, "present": False, "last_seen_time": None, "sent_lost_message": False},
            "exit": {"position": None, "present": False, "last_seen_time": None, "sent_lost_message": False},
            "stefanija": {"position": None, "present": False, "last_seen_time": None, "sent_lost_message": False}
        }
        
        super(threadObjectDetection, self).__init__()
        self.subscribe() # Subscribe on serialCamera topic
        self.send()      #      Sending on topics:
                         #      ------------------
                         ##      ObjectDetection    ##
                         ##   TrafficSignsDetection    ##

    def send(self):
        self.objectDetectionSender = messageHandlerSender(self.queuesList, ObjectDetection)
        self.trafficSignsDetectionSender = messageHandlerSender(self.queuesList, TrafficSignsDetection)

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
                
                # Process frame and get detections
                processed_frame, best_sign, detected_objects = self.process_frame(frame_cropped)
                
                # Update state and send messages
                self.update_state(best_sign, detected_objects)
                
                # Display frame on server
                self.streamer.display(processed_frame)

            except Exception as e:
                print(e)

    def process_frame(self, frame):
        """Process frame and return annotated frame, best detection, and detected objects."""
        # Get YOLO results
        results = self.model(frame, verbose=False)[0]

        # List to store relevant objects
        detected_objects = []

        # List to store traffic signs
        traffic_signs = []

        # Find best detection
        best_sign = None

        # Reset relevant_objects presence
        for obj_key in self.relevant_objects:
            self.relevant_objects[obj_key]["present"] = False

        for box in results.boxes.data:
            x1, y1, x2, y2, conf, cls = box.tolist()
            label = self.model.model.names[int(cls)]
            
            area = (x2 - x1) * (y2 - y1)
            
            # Apply filtering logic
            if label == "car":
                if not (conf > 0.75):
                    continue
                color = (0, 0, 255)
            elif label == "exit":
                if not (conf > 0.15):
                    continue
                color = (0, 255, 0)
            elif label == "stefanija":
                if not (conf > 0.75):
                    continue
                color = (0, 100, 255)
            elif conf > 0.60:
                color = (0, 204, 180)
            else:
                continue
            
            if label not in ("car", "exit", "stefanija"):
                traffic_signs.append((conf, area, label, (x1, y1, x2, y2)))

            # Update relevant_objects if label matches
            if label in self.relevant_objects:
                self.relevant_objects[label]["position"] = (x1, y1, x2, y2)
                self.relevant_objects[label]["present"] = True

            # Draw bounding box and label
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(
                frame, f"{label} {conf:.2f}",
                (int(x1) + 5, int(y1) + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 2
            )

        # Append relevant objects to detected_objects list
        for obj, data in self.relevant_objects.items():
            detected_objects.append({
                "name": obj,
                "position": data["position"],
                "present": data["present"]
            })

        if traffic_signs:
            # Sort by confidence (desc), then area (desc)
            traffic_signs.sort(key=lambda x: (-x[0], -x[1]))
            best_sign = traffic_signs[0][2]

        return frame, best_sign, detected_objects

    def update_state(self, new_sign, detected_objects):
        """Update detection state and handle messaging for traffic signs and detected objects."""
        # Reset counters if no sign detected
        if new_sign is None:
            self.lost_sign_count += 1
            self.confirmation_counter = 0
            if self.lost_sign_count >= self.lost_sign_threshold:
                self.current_sign = None
                self.lost_sign_count = 0
        else:
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
                        if self.debugging:
                            print(f"[INFO]: Novi detektovani znak {new_sign}")
                        self.trafficSignsDetectionSender.send(new_sign)  # Send new sign
                        self.current_sign = new_sign
                        self.confirmation_counter = 0
                        self.lost_sign_count = 0
            # Case 2: No current sign, new detection
            else:
                self.confirmation_counter += 1
                if self.confirmation_counter >= self.confirmation_threshold:
                    if self.debugging:
                        print(f"[INFO]: Novi detektovani znak {new_sign}")
                    self.trafficSignsDetectionSender.send(new_sign)
                    self.current_sign = new_sign
                    self.confirmation_counter = 0
                    self.lost_sign_count = 0

        current_time = time.time()
        for obj in detected_objects:
            name = obj["name"]
            current_position = obj["position"]

            if obj["present"]:
               # Ako je detektovan, ažuriraj vreme i pošalji poruku
               self.relevant_objects[name]["last_seen_time"] = current_time
               if self.relevant_objects[name]["present"]:
                   if self.debugging:
                        print(f"[DETEKCIJA] Objekat '{name}' detektovan na {current_position}")
                   self.relevant_objects[name]["position"] = current_position
                   self.relevant_objects[name]["sent_lost_message"] = False
                   self.objectDetectionSender.send({
                        "name": name,
                        "position": current_position
                    })
            else:
                if self.relevant_objects[name]["last_seen_time"] is not None:
                    time_since_seen = current_time - self.relevant_objects[name]["last_seen_time"]
                    if time_since_seen >= self.lost_timeout and not self.relevant_objects[name]["sent_lost_message"]:
                        if self.debugging:
                            print(f"[GUBITAK] Objekat '{name}' nije detektovan {self.lost_timeout} sekunde – smatra se izgubljenim.")
                        self.relevant_objects[name]["present"] = False
                        self.relevant_objects[name]["position"] = None
                        self.relevant_objects[name]["sent_lost_message"] = True
                        self.objectDetectionSender.send({
                        "name": name,
                        "position": None
                    })

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