import cv2
import numpy as np
import math
from src.ImageProcessing.VideoStream.VideoGridStreamer import VideoStream as vs
from src.core.Auto.PID import PIDController as pid

class LaneDetector:
    def __init__(self, width: int, height: int,logging, debugging=False, pc = False, camera_fov_degrees: float = 79.3):
        self.debugging = debugging
        self.l245ogging = logging
        self.camera_fov_degrees = camera_fov_degrees    
        self.width = width
        self.height = height
        self.pc = pc
        self.currentAngle = 0
        if not pc:
            self.strm = vs(1, 0)
        
        self.roadReg = np.array([[
                (int(self.width * 0.02), self.height - int(self.height * 0.2)),
                (int(self.width * 0.98), self.height - int(self.height * 0.2)),
                (int(self.width * 0.8), self.height // 2 - int(self.height * 0.05)),
                (int(self.width * 0.2), self.height // 2 - int(self.height * 0.05))
            ]], np.int32)
        
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
        
        self.squareLeft = (100, 180, 190, 220)
        self.squareRight = (320, 180, 410, 220)

    def calculate_steering_angle(self, lines, img_width, img_height):
        if lines is None:
            return 0  # No lines detected, keep going straight
        left_slopes = []
        right_slopes = []
        left_intercepts = []
        right_intercepts = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0
                intercept = y1 - slope * x1


                if slope < -0.4:  # Left lanes have negative slope
                    left_slopes.append(slope)
                    left_intercepts.append(intercept)
                elif slope > 0.4:  # Right lanes have positive slope
                    right_slopes.append(slope)
                    right_intercepts.append(intercept)
        # (Ostatak funkcije ostaje isti)
        left_avg_slope = np.mean(left_slopes) if left_slopes else 0
        left_avg_intercept = np.mean(left_intercepts) if left_intercepts else 0

        right_avg_slope = np.mean(right_slopes) if right_slopes else 0
        right_avg_intercept = np.mean(right_intercepts) if right_intercepts else 0

        y = img_height // 2

        left_x = self.extrapolate_missing_lane(left_avg_slope, left_avg_intercept, y, img_width) if left_avg_slope != 0 else 0
        right_x = self.extrapolate_missing_lane(right_avg_slope, right_avg_intercept, y, img_width) if right_avg_slope != 0 else img_width

    # 360,180), (400,220),

        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0

                if slope < -0.4:
                    if self.isPointInSquare(x1, y1, self.squareRight) or self.isPointInSquare(x2, y2, self.squareRight):
                        return 0
                if slope > 0.4:
                    if self.isPointInSquare(x1, y1, self.squareLeft) or self.isPointInSquare(x2, y2, self.squareLeft):
                        return 0


        if left_x == 0 and right_x != img_width:
            return -21
        elif right_x == img_width and left_x != 0:
            return 23
        elif left_x == 0 and right_x == img_width:
            return 0

        center_x = (left_x + right_x) // 2
        angle = np.arctan2(center_x - img_width // 2, img_width // 2)
        angle_degrees = np.degrees(angle)
        max_angle = 25
        scaled_angle = max(-max_angle, min(max_angle, angle_degrees)) -1
        return scaled_angle

    def extrapolate_missing_lane(self, slope, intercept, y, width):
        if slope == 0:
            return width // 2
        return int((y - intercept) / slope)

    def isPointInSquare(self, x, y, square):
        x1, y1, x2, y2 = square
        x_min = min(x1, x2)
        x_max = max(x1, x2)
        y_min = min(y1, y2)
        y_max = max(y1, y2)
        
        return x_min <= x <= x_max and y_min <= y <= y_max

    def region_of_interest(self, img):
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, self.roadReg, 255)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image
    
    def distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

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
    def gamma_correction(self, image: np.ndarray, gamma: float) -> np.ndarray:
        image_normalized = image  / 255.0
        corrected = np.power(image_normalized, gamma)
        corrected = np.uint8(corrected * 255)
        return corrected

    def region_of_interest2(self, img):
        mask = np.zeros_like(img, dtype=np.uint8)
        cv2.fillPoly(mask, self.stopReg, (555, 255, 255)) # Mask now matches BGR format
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image
    
    def region_of_interest3(self, img):
        mask = np.zeros_like(img, dtype=np.uint8)
        cv2.fillPoly(mask, self.interStopReg, (255, 255, 255)) # Mask now matches BGR format
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image
    
    def drawRectangle(self, frame, square):
        x1, y1, x2, y2 = square
        cv2.rectangle(frame, (x1,y1), (x2,y2), (25, 200, 50), 2)
    
    def detect_lines(self, img, tres = 30):
        return cv2.HoughLinesP(img, rho=1, theta=np.pi / 180, threshold=tres, minLineLength=15, maxLineGap=5)
    
    def process_frame(self, frame: np.ndarray):
        angle_degrees: float = 0.0

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        gama = self.gamma_correction(gray, 12)
        blurred = cv2.GaussianBlur(gama, [5, 5], 0)
        edges = cv2.Canny(blurred, 50, 150)

        roi2 = self.region_of_interest2(edges)
        roi = self.region_of_interest(edges)
        roi3 = self.region_of_interest3(edges)

        lines = self.detect_lines(roi, 25)
        lines2 = self.detect_lines(roi2, 10)
        lines3 = self.detect_lines(roi3, 15)

        intersection, linesX = self.detectIntersection(lines2)
        intersectionA, linesY = self.detectIntersection(lines3)

        angle_degrees = float(self.calculate_steering_angle(lines, frame.shape[1], frame.shape[0]))

        if self.debugging:
            # Draw lines on the frame
            if lines is not None:
                for line in lines:
                    for x1, y1, x2, y2 in line:
                        slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0
                        if slope < -0.4 or slope > 0.4:
                            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
            

            self.drawRectangle(frame, self.squareLeft)
            self.drawRectangle(frame, self.squareRight)

            points = self.roadReg.reshape((-1, 1, 2))
            cv2.polylines(frame, [points], isClosed=True, color=(0, 255, 0), thickness=2)
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
        
        if not self.pc:
            self.strm.display(frame)

        if intersectionA:
            intersection = False

        return frame, angle_degrees, intersection, intersectionA