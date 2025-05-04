import cv2
import numpy as np
import math
import time
from collections import deque
from pid_controller import PIDController  # use external PID implementation

class LaneDetector:
    def __init__(self, width: int, height: int, logging, debugging=False, pc=False, camera_fov_degrees: float = 79.3):
        self.debugging = debugging
        self.logging = logging
        self.camera_fov_degrees = camera_fov_degrees    
        self.width = width
        self.height = height
        self.pc = pc
        self.currentAngle = 0
        self.last_frame_time = time.time()  # Dodato za praćenje vremena između frejmova
        
        # Define road region for processing
        self.roadReg = np.array([[
            (int(self.width * 0.02), self.height - int(self.height * 0.02)),   # donji lijevi ugao
            (int(self.width * 0.22), self.height - int(self.height * 0.02)),  # donji lijevi prije ulegnuća
            (int(self.width * 0.30), self.height - int(self.height * 0.35)), # donji lijevi ulegnuće
            (int(self.width * 0.7), self.height - int(self.height * 0.35)), # donji desni ulegnuće
            (int(self.width * 0.78), self.height - int(self.height * 0.02)),  # donji desni prije ulegnuća
            (int(self.width * 0.98), self.height - int(self.height * 0.02)),  # donji desni ugao
            (int(self.width * 0.8), self.height // 2 - int(self.height * 0.05)), # gornji desni ugao
            (int(self.width * 0.2), self.height // 2 - int(self.height * 0.05))  # gornji lijevi ugao
        ]], np.int32)
        
        # Line history for smoothing
        self.left_lines_history = deque(maxlen=5)
        self.right_lines_history = deque(maxlen=5)
        
        # Line filtering parameters
        self.min_line_length = 8
        self.max_line_gap = 25
        self.min_line_pixels = 5
        
        # Parameters for filtering out stop lines and puddles
        self.min_slope_threshold = 0.2  # Increased from 0.15 - stronger filter for horizontal lines
        self.max_slope_threshold = 5.0  # Maximum slope for valid lane lines
        self.stop_line_angle_threshold = 20  # Maximum angle (in degrees) to consider a line as a stop line
        
        # Filter out puddles and other irregular shapes
        self.min_lane_line_length = 15  # Minimum length for lane lines (pixels)
        self.max_deviation_from_straight = 0.2  # Max deviation from straight line (lower = straighter lines)
        
        # Measurement point - height in the image where we calculate lane center
        self.measure_height = int(self.height * 0.8)  # 80% down the image
        
        # PID controller for steering
        self.pid = PIDController(kp=0.8, ki=0.1, kd=0, output_limits=(-25, 25))
        
        # Image center reference point
        self.center_x = self.width // 2
        
        # Colors for visualization
        self.color_left_lane = (255, 100, 0)    # Blue-ish
        self.color_right_lane = (0, 0, 255)     # Red
        self.color_road_region = (100, 255, 0)    # Green
        self.color_lane_center = (255, 0, 255)  # Magenta
        self.color_measure_line = (0, 255, 255) # Yellow
        self.color_image_center = (0, 255, 0)   # Green
        self.color_steering = (255, 255, 0)     # Yellow

    def region_of_interest(self, img):
        """Apply a region of interest mask to focus only on the road area"""
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, self.roadReg, 255)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image
    
    def distance(self, point1, point2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    
    def detect_lines(self, img, tres=30):
        """Detect lines using Hough transform"""
        return cv2.HoughLinesP(
            img, 
            rho=1, 
            theta=np.pi/180, 
            threshold=tres, 
            minLineLength=self.min_line_length, 
            maxLineGap=self.max_line_gap
        )
    
    def filter_lines(self, lines):
        """Filter lines based on size, angle, and position to separate left and right lanes"""
        left_lines = []
        right_lines = []
        
        if lines is None:
            return [], []
        
        image_center_x = self.width // 2
        
        for line in lines:
            for x1, y1, x2, y2 in line:
                # Skip very short lines - could be noise
                if self.distance((x1, y1), (x2, y2)) < self.min_line_pixels:
                    continue
                
                # Calculate line parameters
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                midpoint_x = (x1 + x2) / 2
                
                # Filter out horizontal lines and very short lines
                if abs(slope) < self.min_slope_threshold or length < 10:
                    continue
                
                # Classification based on position and slope
                # In curves, both lines might have similar slopes but different positions
                if midpoint_x < image_center_x and slope < 0:  # Left side negative slope -> left line
                    left_lines.append(line)
                elif midpoint_x > image_center_x and slope > 0:  # Right side positive slope -> right line
                    right_lines.append(line)
                # Special case for curves - both lines might have similar slopes
                elif midpoint_x < image_center_x - 50:  # Clearly on left side regardless of slope
                    left_lines.append(line)
                elif midpoint_x > image_center_x + 50:  # Clearly on right side regardless of slope
                    right_lines.append(line)
        
        return left_lines, right_lines

    def extrapolate_lane_line(self, lines, height):
        """Extrapolate a lane line to reach the given height in the image"""
        x_coords = []
        y_coords = []
        
        for line in lines:
            for x1, y1, x2, y2 in line:
                x_coords.extend([x1, x2])
                y_coords.extend([y1, y2])
        
        if not x_coords or not y_coords:
            return None
            
        # Fit a polynomial to the points
        fit = np.polyfit(y_coords, x_coords, 1)
        
        # Calculate x-coordinate at the given height
        x = int(fit[0] * height + fit[1])
        
        return x

    def process_frame(self, frame: np.ndarray, edges: np.ndarray):
        """Process a frame to detect lane lines and calculate steering angle"""
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_frame_time
        self.last_frame_time = current_time
        
        # Apply region of interest mask
        roi = self.region_of_interest(edges)
        
        # Detect lines
        lines = self.detect_lines(roi, 5)
        
        # Filter and separate left and right lane lines
        left_lines, right_lines = self.filter_lines(lines)
        
        # Smooth the detected lines over time
        if left_lines:
            self.left_lines_history.append(left_lines)
        if right_lines:
            self.right_lines_history.append(right_lines)
            
        # Use recent history for stability
        left_lines_stable = [line for lines_group in self.left_lines_history for line in lines_group]
        right_lines_stable = [line for lines_group in self.right_lines_history for line in lines_group]
        
        # Find lane lines at the measurement height
        left_x = self.extrapolate_lane_line(left_lines_stable, self.measure_height)
        right_x = self.extrapolate_lane_line(right_lines_stable, self.measure_height)
        
        # Calculate lane center
        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) // 2
        elif left_x is not None:
            lane_center = left_x + 100  # Estimate, assuming a lane width of ~200px
        elif right_x is not None:
            lane_center = right_x - 100  # Estimate, assuming a lane width of ~200px
        else:
            lane_center = self.center_x  # Default to center if no lanes detected
            
        # Calculate error (offset from center)
        error = self.center_x - lane_center
        
        # Use PID to calculate steering angle
        angle_degrees = self.pid.compute(error, dt=dt)
        
        # Create a visualization frame for dashboard
        viz_frame = frame.copy()
        
        # Draw region of interest with transparent overlay
        road_mask = np.zeros_like(frame)
        cv2.fillPoly(road_mask, self.roadReg, (0, 50, 0))
        viz_frame = cv2.addWeighted(viz_frame, 1.0, road_mask, 0.3, 0)
        
        # Draw region outline
        points = self.roadReg.reshape((-1, 1, 2))
        cv2.polylines(viz_frame, [points], isClosed=True, color=self.color_road_region, thickness=3)
        
        # Draw left lane lines
        if left_lines_stable:
            for line in left_lines_stable:
                for x1, y1, x2, y2 in line:
                    cv2.line(viz_frame, (x1, y1), (x2, y2), self.color_left_lane, 2)
        
        # Draw right lane lines
        if right_lines_stable:
            for line in right_lines_stable:
                for x1, y1, x2, y2 in line:
                    cv2.line(viz_frame, (x1, y1), (x2, y2), self.color_right_lane, 2)
        
        # Draw measurement line
        cv2.line(viz_frame, (0, self.measure_height), (self.width, self.measure_height), 
                self.color_measure_line, 1)
        
        # Draw lane positions at measurement height
        if left_x is not None:
            cv2.circle(viz_frame, (left_x, self.measure_height), 6, self.color_left_lane, -1)
        if right_x is not None:
            cv2.circle(viz_frame, (right_x, self.measure_height), 6, self.color_right_lane, -1)
        
        # Draw lane center
        if left_x is not None or right_x is not None:
            cv2.circle(viz_frame, (lane_center, self.measure_height), 8, self.color_lane_center, -1)
            # Draw line from lane center to top of image to show projected path
            cv2.line(viz_frame, (lane_center, self.measure_height), 
                    (lane_center, self.height//4), self.color_lane_center, 1)
        
        # Draw image center reference
        cv2.line(viz_frame, (self.center_x, self.height - 20), 
                (self.center_x, self.height//4), self.color_image_center, 1)
        
        # Draw current steering angle indicator
        angle_indicator_length = 70
        angle_rad = math.radians(angle_degrees)
        end_x = int(self.center_x - angle_indicator_length * math.sin(angle_rad))
        end_y = int(self.height - angle_indicator_length * math.cos(angle_rad))
        cv2.line(viz_frame, (self.center_x, self.height - 20), (end_x, end_y), 
                self.color_steering, 3)
        
        # Debug log for terminal
        if self.logging:
            print(f"Lane Detection: Left={left_x is not None}, Right={right_x is not None}, "
                f"Center={lane_center}, Error={error:.1f}px, Angle={angle_degrees:.1f}°")
        

        
        return viz_frame, angle_degrees