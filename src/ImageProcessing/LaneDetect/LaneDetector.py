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
        
        self.roadReg = np.array([[
                (int(self.width * 0.02), self.height - int(self.height * 0.25)),
                (int(self.width * 0.98), self.height - int(self.height * 0.25)),
                (int(self.width * 0.85), self.height // 2 + int(self.height * 0.05)),
                (int(self.width * 0.15), self.height // 2 + int(self.height * 0.05))
            ]], np.int32)
        
        self.squareLeft = (120, 180, 220, 220)
        self.squareRight = (290, 180, 390, 220)

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

        y = img_height // 2 +40

        left_x = self.extrapolate_missing_lane(left_avg_slope, left_avg_intercept, y, img_width) if left_avg_slope != 0 else 0
        right_x = self.extrapolate_missing_lane(right_avg_slope, right_avg_intercept, y, img_width) if right_avg_slope != 0 else img_width

    # 360,180), (400,220),

        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0

                if slope < -0.4:
                    if self.isPointInSquare(x1, y1, self.squareRight) or self.isPointInSquare(x2, y2, self.squareRight):
                        return 8
                if slope > 0.4:
                    if self.isPointInSquare(x1, y1, self.squareLeft) or self.isPointInSquare(x2, y2, self.squareLeft):
                        return -8


        if left_x == 0 and right_x != img_width:
            return -21
        elif right_x == img_width and left_x != 0:
            return 23
        elif left_x == 0 and right_x == img_width:
            return 0

        center_x = (left_x + right_x) // 2
        angle = np.arctan2(center_x - (img_width // 2 - 20), img_width // 2 - 20)
        angle_degrees = np.degrees(angle) * 1.2
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
    
    def drawRectangle(self, frame, square):
        x1, y1, x2, y2 = square
        cv2.rectangle(frame, (x1,y1), (x2,y2), (25, 200, 50), 2)
    
    def detect_lines(self, img, tres = 30):
        return cv2.HoughLinesP(img, rho=1, theta=np.pi / 180, threshold=tres, minLineLength=8, maxLineGap=25)

    def process_frame(self, frame: np.ndarray, edges):
        angle_degrees: float = 0.0

        roi = self.region_of_interest(edges)
        lines = self.detect_lines(roi, 5)

        angle_degrees = float(self.calculate_steering_angle(lines, frame.shape[1], frame.shape[0]))

        if self.debugging:
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


        return frame, angle_degrees