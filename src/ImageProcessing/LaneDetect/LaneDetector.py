import cv2
import numpy as np

class LaneDetector:
    def __init__(self, width: int, height: int, logging, debugging=False, camera_fov_degrees: float = 79.3):
        self.logging = logging
        self.debugging = debugging

        self.camera_fov_degrees = camera_fov_degrees    
        self.width = width
        self.height = height

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


                if slope < -0.5:  # Left lanes have negative slope
                    left_slopes.append(slope)
                    left_intercepts.append(intercept)
                elif slope > 0.5:  # Right lanes have positive slope
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

                if slope < -0.5:
                    if self.isPointInSquare(x1, y1, 340, 180, 470, 220) or self.isPointInSquare(x1, y1, 340, 180, 470, 220):
                        return 0
                if slope > 0.5:
                    if self.isPointInSquare(x1, y1, 80, 180, 190, 220) or self.isPointInSquare(x1, y1, 80, 180, 190, 220):
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

    def isPointInSquare(self, x, y, x1 = 360, y1 = 180, x2 = 400, y2 = 220):
        x_min = min(x1, x2)
        x_max = max(x1, x2)
        y_min = min(y1, y2)
        y_max = max(y1, y2)
        
        return x_min <= x <= x_max and y_min <= y <= y_max

    def region_of_interest(self, img):
        height, width = img.shape[:2]
        mask = np.zeros_like(img)
        
        # Define a triangular region of interest (lower part of the image)
        polygon = np.array([[
            (width * 0.07, height - int(height * 0.2)),
            (width * 0.9, height - int(height * 0.2)),
            (int(width * 0.7), height // 2 - int(height * 0.2)),
            (int(width * 0.2), height // 2 - int(height * 0.2))
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def detect_lines(self, img):
        # Use Hough transformation to detect lines
        return cv2.HoughLinesP(
            img, rho=1, theta=np.pi / 180, threshold=60, minLineLength=15, maxLineGap=120
        )

    def process_frame(self, frame: np.ndarray):
        """Process a single frame for lane detection."""
        angle_degrees: float = 0.0
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        roi = self.region_of_interest(edges)
        lines = self.detect_lines(roi)

        angle_degrees = float(self.calculate_steering_angle(lines, frame.shape[1], frame.shape[0]))

        # Draw lines on the frame
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

        return frame, angle_degrees