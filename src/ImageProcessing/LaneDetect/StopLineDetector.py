import cv2
import numpy as np
import math
from src.ImageProcessing.VideoStream.VideoGridStreamer import VideoStream as vs
from src.core.Auto.PID import PIDController as pid

class StopLineDetector:
    def __init__(self, width: int, height: int,logging, debugging=False, pc = False, camera_fov_degrees: float = 79.3):
        self.debugging = debugging
        self.logging = logging
        self.camera_fov_degrees = camera_fov_degrees    
        self.width = width
        self.height = height
        self.pc = pc
        
        self.stopReg = np.array([[
                (self.width*0.75, self.height*0.68),
                (self.width*0.25, self.height*0.68),
                (self.width*0.25, self.height*0.83),
                (self.width*0.75, self.height*0.83)
            ]], np.int32)
        
        self.interStopReg = np.array([[
                (self.width*0.68, self.height*0.40),
                (self.width*0.32, self.height*0.40),
                (self.width*0.25, self.height*0.68),
                (self.width*0.75, self.height*0.68)
            ]], np.int32)

    def detectIntersection(self, lines) -> bool:
        if lines is None:
            return False, []
        
        lines2 = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0
                distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

                if slope < 0.2 and slope > -0.2:
                    if distance > 50:
                        lines2.append([(x1, y1), (x2, y2)])
        if(len(lines2) >= 2):
            return True, lines2
        
        return False, []

    def region_of_interest(self, img, Reg):
        mask = np.zeros_like(img, dtype=np.uint8)
        cv2.fillPoly(mask, Reg, (255, 255, 255)) # Mask now matches BGR format
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image
    
    def detect_lines(self, img, tres = 30):
        return cv2.HoughLinesP(img, rho=1, theta=np.pi / 180, threshold=tres, minLineLength=10, maxLineGap=30)
    
    def process_frame(self, frame: np.ndarray, edges):
        roi2 = self.region_of_interest(edges, self.stopReg)
        roi3 = self.region_of_interest(edges, self.interStopReg)

        lines2 = self.detect_lines(roi2, 10)
        lines3 = self.detect_lines(roi3, 15)

        intersection, linesX = self.detectIntersection(lines2)
        intersectionA, linesY = self.detectIntersection(lines3)

        if self.debugging:
            points = self.stopReg.reshape((-1, 1, 2))
            cv2.polylines(frame, [points], isClosed=True, color=(255, 255, 0), thickness=2)
            points = self.interStopReg.reshape((-1, 1, 2))
            cv2.polylines(frame, [points], isClosed=True, color=(150, 255, 50), thickness=2)

            if len(linesX) > 0:
                for line in linesX:
                    (x1, y1), (x2, y2) = line
                    cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 3)
            
            if len(linesY) > 0:
                for line in linesY:
                    (x1, y1), (x2, y2) = line
                    cv2.line(frame, (x1, y1), (x2, y2), (200, 50, 0), 3)

        if intersectionA:
            intersection = False

        return frame, intersection, intersectionA