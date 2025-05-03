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
from src.hardware.camera.encoder import decode_frame

class threadObjectDetection(ThreadWithStop):
    """This thread handles ObjectDetection."""
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.model = YOLO('src/ImageProcessing/ObjectDetection/threads/yolo_version_2.7/detect/train/weights/best.pt')
        self.streamer = VideoStream(0, 0)
        
        # State management for signs
        self.current_sign = None
        self.previous_sign = None
        self.confirmation_counter = 0
        self.confirmation_threshold = 3
        self.lost_sign_count = 0
        self.lost_sign_threshold = 17
        
        # State management for relevant objects
        self.relevant_objects_state = {
            'car': {'presence_count': 0, 'absence_count': 0, 'active': False, 'reported': False},
            'exit': {'presence_count': 0, 'absence_count': 0, 'active': False, 'reported': False},
            'stefanija': {'presence_count': 0, 'absence_count': 0, 'active': False, 'reported': False}
        }
        self.required_presence_frames = 3
        self.required_absence_frames = 17
        
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
                frame = decode_frame(videoData)
                frame_cropped = self.crop_frame(frame)
                frame_cropped = cv2.resize(frame_cropped, (256,256), interpolation=cv2.INTER_AREA)
                processed_frame, best_sign, relevant_objects = self.process_frame(frame_cropped)
                
                # Update states
                self.update_state(best_sign)
                active_relevant_objects = self.handle_relevant_objects(relevant_objects)
                
                # Check if we need to send a message
                send_message = False
                message_content = {}
                
                # Check sign changes
                if self.current_sign != self.previous_sign:
                    message_content['sign'] = self.current_sign
                    self.previous_sign = self.current_sign
                    send_message = True
                
                # Check relevant objects changes
                new_active_objects = [obj for obj in active_relevant_objects 
                                    if not self.relevant_objects_state[obj]['reported']]
                if new_active_objects:
                    message_content['relevant_objects'] = new_active_objects
                    for obj in new_active_objects:
                        self.relevant_objects_state[obj]['reported'] = True
                    send_message = True
                
                # Send message if needed
                if send_message:
                    if 'sign' in message_content and message_content['sign'] is not None:
                        self.objectDetectionSender.send(message_content['sign'])
                    
                    if 'relevant_objects' in message_content:
                        for obj in message_content['relevant_objects']:
                            self.objectDetectionSender.send(obj)

                    # print(message_content)
                
                self.streamer.display(processed_frame)
            except Exception as e:
                print(e)

    def process_frame(self, frame):
        """Process frame and return annotated frame, best detection, and relevant object list."""
        results = self.model(frame, verbose=self.debugging)[0]
        cv2.rectangle(frame, (200, 200), (255, 255), (150, 255, 150), 3)

        h, w = frame.shape[:2]
        center_box = (w // 2 - 20, int(h * 0.2), w // 2 + 20, h)

        best_sign = None
        detections = []
        relevant_objects = []

        for box in results.boxes.data:
            x1, y1, x2, y2, conf, cls = box.tolist()
            label = self.model.model.names[int(cls)]

            box_cx = (x1 + x2) / 2
            box_cy = (y1 + y2) / 2
            area = (x2 - x1) * (y2 - y1)
            in_center = (
                center_box[0] <= box_cx <= center_box[2] and
                center_box[1] <= box_cy <= center_box[3]
            )

            # Apply filtering logic
            if label == "car":
                if not (conf > 0.75 and in_center):
                    continue
                color = (0, 0, 255)
            elif label == "exit":
                if not (conf > 0.15 and y2 > 180 and x2 > 180):
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

            if label in ("car", "exit", "stefanija"):
                relevant_objects.append(label)
            else:
                detections.append((conf, area, label, (x1, y1, x2, y2)))

            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(
                frame, f"{label} {conf:.2f}",
                (int(x1) + 5, int(y1) + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 2
            )

        if detections:
            detections.sort(key=lambda x: (-x[0], -x[1]))
            best_sign = detections[0][2]

        return frame, best_sign, relevant_objects

    def update_state(self, new_sign):
        """Update detection state for signs."""
        if new_sign is None:
            self.lost_sign_count += 1
            self.confirmation_counter = 0
            if self.lost_sign_count >= self.lost_sign_threshold and self.current_sign is not None:
                self.current_sign = None
                self.lost_sign_count = 0
            return

        if self.current_sign is not None:
            if new_sign == self.current_sign:
                self.lost_sign_count = 0
                self.confirmation_counter = 0
            else:
                self.confirmation_counter += 1
                if self.confirmation_counter >= self.confirmation_threshold:
                    self.current_sign = new_sign
                    self.confirmation_counter = 0
                    self.lost_sign_count = 0
        else:
            self.confirmation_counter += 1
            if self.confirmation_counter >= self.confirmation_threshold:
                self.current_sign = new_sign
                self.confirmation_counter = 0
                self.lost_sign_count = 0

    def handle_relevant_objects(self, current_relevant_objects):
        """
        Track relevant objects and return active ones.
        An object needs to be seen for 3 consecutive frames to be considered active,
        and needs to be absent for 17 frames to be considered inactive.
        """
        active_objects = []
        
        # Update state for each relevant object
        for obj in self.relevant_objects_state:
            state = self.relevant_objects_state[obj]
            
            if obj in current_relevant_objects:
                # Object is present in current frame
                state['presence_count'] += 1
                state['absence_count'] = 0
                
                # Check if we've seen it enough to activate
                if state['presence_count'] >= self.required_presence_frames and not state['active']:
                    state['active'] = True
                    state['reported'] = False
            else:
                # Object is not present in current frame
                if state['active']:
                    state['absence_count'] += 1
                    
                    # Check if we've not seen it enough to deactivate
                    if state['absence_count'] >= self.required_absence_frames:
                        state['active'] = False
                        state['presence_count'] = 0
                        state['reported'] = False
                else:
                    # Reset presence counter if not active
                    state['presence_count'] = 0
            
            # Add to active objects if currently active
            if state['active']:
                active_objects.append(obj)
        
        return active_objects

    @staticmethod
    def crop_frame(frame):
        """Crop top-right quadrant of frame."""
        h, _ = frame.shape[:2]
        return frame[0:h-63, :]