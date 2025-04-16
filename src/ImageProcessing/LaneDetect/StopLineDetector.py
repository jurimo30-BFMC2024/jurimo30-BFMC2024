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
                (self.width*0.75, self.height*0.64),
                (self.width*0.25, self.height*0.64),
                (self.width*0.25, self.height*0.83),
                (self.width*0.75, self.height*0.83)
            ]], np.int32)
        
        self.interStopReg = np.array([[
                (self.width*0.68, self.height*0.40),
                (self.width*0.32, self.height*0.40),
                (self.width*0.25, self.height*0.64),
                (self.width*0.75, self.height*0.64)
            ]], np.int32)

    def detectIntersection(self, lines):
        if lines is None:
            return False, [], None
        
        lines2 = []
        slope_degrees = None

        for line in lines:
            for x1, y1, x2, y2 in line:
                dx = x2 - x1
                dy = y2 - y1
                distance = np.sqrt(dx**2 + dy**2)

                if dx != 0:
                    slope = dy / dx
                else:
                    slope = 0  # Used only for filtering, real angle from atan2

                if -0.3 < slope < 0.3 and distance > 50:
                    lines2.append([(x1, y1), (x2, y2)])
                    slope_degrees = math.degrees(math.atan2(dy, dx))

        if len(lines2) >= 2:
            return True, lines2, slope_degrees

        return False, [], None

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

        lines2 = self.detect_lines(roi2, 5)
        lines3 = self.detect_lines(roi3, 10)

        intersection, linesX, slope_degrees = self.detectIntersection(lines2)
        intersectionA, linesY, _ = self.detectIntersection(lines3)

        if self.debugging:
            points = self.stopReg.reshape((-1, 1, 2))
            cv2.polylines(frame, [points], isClosed=True, color=(255, 255, 0), thickness=2)
            points = self.interStopReg.reshape((-1, 1, 2))
            cv2.polylines(frame, [points], isClosed=True, color=(150, 255, 50), thickness=2)


            # Pretpostavka da se linesX odnosi na linije detektovane na zaustavnoj liniji
            if len(linesX) > 0:
                points = []

                for (x1, y1), (x2, y2) in linesX:
                    points.append((x1, y1))
                    points.append((x2, y2))

                points = np.array(points)
                x = points[:, 0]
                y = points[:, 1]

                # Fit a line: y = m*x + b
                m, b = np.polyfit(x, y, 1)

                # Define two x points to draw the fitted line
                x_start = int(np.min(x))
                x_end = int(np.max(x))
                y_start = int(m * x_start + b)
                y_end = int(m * x_end + b)

                # Kombinuje sve detektovane linije u jednu veliku prosecnu
                cv2.line(frame, (x_start, y_start), (x_end, y_end), (255, 0, 0), 3)

            # Ovo me ne interesuje jer pretpostavljam da su linesY za ono dalje detektovanje raskrsnice
            # pa cu ostaviti zasad samo ovako
            if len(linesY) > 0:
                for line in linesY:
                    (x1, y1), (x2, y2) = line
                    cv2.line(frame, (x1, y1), (x2, y2), (200, 50, 0), 3)

        if intersectionA:
            intersection = False

        return frame, (intersection, slope_degrees), intersectionA