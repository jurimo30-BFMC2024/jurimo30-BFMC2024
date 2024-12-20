import cv2
import threading
import base64
import time
import numpy as np

from src.utils.messages.allMessages import (
    mainCamera,
    LaneDetect,
)

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
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

        # Sender za slanje rezultata detekcije
        self.laneDetectionSender = messageHandlerSender(self.queuesList, LaneDetect)

        self.subscribe()
        

    def run(self):
        while self._running:
            try:
                videoData = self.videoSubscriber.receive()
                if videoData is not None:
                    # Dekodiraj frejm iz base64
                    frame = self.decode_frame(videoData["data"])

                    # Detekcija linija
                    lane_info = self.detect_lines(frame)

                    # Slanje rezultate
                    self.laneDetectionSender.send(lane_info)
            except Exception as e:
                print(e)

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
         # Subscriber za prijem video toka
        self.videoSubscriber = messageHandlerSubscriber(self.queuesList, mainCamera)

    def decode_frame(self, encoded_data):
        """Decode base64 encoded frame to an OpenCV image."""
        frame_data = base64.b64decode(encoded_data)
        np_array = np.frombuffer(frame_data, np.uint8)
        frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
        return frame

    # ================================ Lane detection ===============================================

    def find_contours(image):
        """Pronalaženje kontura u slici."""
        imgray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        _, thresh = cv2.threshold(imgray, 50, 255, cv2.THRESH_BINARY)    
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def draw_contours(frame, contours):
        """Crtanje kontura na slici."""
        return cv2.drawContours(frame, contours, -1, (0, 255, 0), cv2.FILLED)

    #(240, 320, 3) shape
    def region_of_interest(image):
        """Definisanje regiona od interesa."""
        
        height = image.shape[0]
        polygons = np.array([[(-100, height), (400, height), (270, 90), (48, 90)]])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, polygons, 255)
        return cv2.bitwise_and(image, mask)

    def find_lane_center(contours, width):
        """Pronalazi sredinu puta na osnovu kontura."""
        left_x = []
        right_x = []
        center_line_y = 160  # Y-koordinata za merenje centra na slici

        for contour in contours:
            for point in contour:
                x, y = point[0]
                if y > center_line_y - 122 and y < center_line_y + 122:  # Fokusiraj se na određeni red (y)
                    if x < width // 2:
                        left_x.append(x)
                    else:
                        right_x.append(x)

        # Prosečne x koordinate za levu i desnu liniju
        if left_x and right_x:
            left_avg_x = int(np.mean(left_x))
            right_avg_x = int(np.mean(right_x))
            lane_center_x = (left_avg_x + right_avg_x) // 2
            return lane_center_x, left_avg_x, right_avg_x

        return None, None, None

    def calculate_angle(offset_pixels, frame_width, fov_degrees):
        """Izračunava ugao na osnovu pomeraja piksela."""
        # Pretvori FOV iz stepeni u radijane
        fov_radians = np.radians(fov_degrees)
        # Izračunaj ugao za svaki piksel
        pixels_per_radian = frame_width / fov_radians
        angle_radians = offset_pixels / pixels_per_radian
        return np.degrees(angle_radians)  # Konvertuj u stepene
        
    def detect_lines(self,frame):
        height, width, _ = frame.shape
        # Definiši FOV kamere (u stepenima)
        CAMERA_FOV_DEGREES = 66  # Primer: kamera ima FOV od 66 stepeni

        # Primena regiona interesa
        roi = self.region_of_interest(frame)

        # Pronalaženje kontura
        contours = self.find_contours(roi)

        # Pronalazak sredine puta
        lane_center_x, left_avg_x, right_avg_x = self.find_lane_center(contours, width)

        if lane_center_x is not None:
            # Crtanje sredine puta
            # cv2.circle(frame, (lane_center_x, 130), 3, (0, 0, 255), -1)
            
            # Crtanje leve i desne linije
            # cv2.line(frame, (left_avg_x, 120), (left_avg_x, 140), (255, 0, 0), 3)
            # cv2.line(frame, (right_avg_x, 120), (right_avg_x, 140), (255, 0, 0), 3)

            # Izračunaj odstupanje i ugao
            car_center_x = width // 2
            offset_pixels = lane_center_x - car_center_x
            angle_degrees = self.calculate_angle(offset_pixels, width, CAMERA_FOV_DEGREES)
            
            distance = right_avg_x - left_avg_x
            # Prikaz odstupanja na ekranu
            # print(distance)
            if distance < 230 and angle_degrees < 0:
                #text = f"Skreni desno za: {np.abs(angle_degrees):.2f} stepeni"
                return np.abs(angle_degrees)
            elif distance < 230 and angle_degrees > 0:
                #text = f"Skreni levo za: {(-angle_degrees):.2f} stepeni"
                return -angle_degrees
            elif distance > 230 and angle_degrees < 0:
                #text = f"Skreni levo za: {angle_degrees:.2f} stepeni"
                return angle_degrees
            elif distance > 230 and angle_degrees > 0:
                #text = f"Skreni desno za: {angle_degrees:.2f} stepeni"
                return angle_degrees
            #else:
                #text = f"Drzis pravac"
            
            #cv2.putText(frame, text, (20, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (52, 85, 235), 2)

        
        # Crtanje kontura na frejmu
        # frame_with_contours = self.draw_contours(frame, contours)

        # Prikaz rezultata
        #cv2.imshow("Lane Detection", frame)
        #cv2.imshow("Lane Detection", frame_with_contours)
        