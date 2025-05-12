import cv2
import numpy as np
from math import atan2, degrees


class StopLineDetector:
    def __init__(self, width: int, height: int, logging, debugging=False, camera_fov_degrees: float = 79.3):
        self.width = width
        self.height = height
        self.logging = logging
        self.debugging = debugging
        self.camera_fov_degrees = camera_fov_degrees
        
        # Parameters for stop line detection
        self.min_line_length = int(width * 0.3)  # Minimum length of line
        self.max_line_gap = int(width * 0.05)    # Maximum allowed gap between line segments
        self.horizontal_angle_threshold = 30     # Maximum angle from horizontal (in degrees)
        self.previous_detections = []            # Store previous detections for smoothing
        self.detection_history_size = 5          # Number of previous frames to consider
        self.detection_threshold = 3             # Minimum detections to consider valid
        self.last_detected = False               # Was a stop line detected in the previous frame
        
        # Define polygon ROI for stop line detection
        self.roi_polygon = np.array([
            [int(width * 0.1), int(height * 0.7)],    # Bottom left
            [int(width * 0.1), int(height * 0.3)],    # Top left
            [int(width * 0.9), int(height * 0.3)],    # Top right
            [int(width * 0.9), int(height * 0.7)]     # Bottom right
        ], np.int32)
        
        # ROI polygon color and thickness for visualization
        self.roi_color = (150, 50, 80)
        self.roi_thickness = 2


    def process_frame(self, frame: np.ndarray, edges: np.ndarray):
        detected = False
        distance = 0
        angle = 0
        best_line = None  # Store the best line data
        
        # Apply polygon ROI mask
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, [self.roi_polygon], 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # Use Hough Line Transform to detect lines
        lines = cv2.HoughLinesP(
            masked_edges, 
            rho=1, 
            theta=np.pi/180, 
            threshold=50, 
            minLineLength=self.min_line_length, 
            maxLineGap=self.max_line_gap
        )
        
        potential_stop_lines = []
        
        # Process detected lines
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # Calculate line angle
                if x2 - x1 == 0:  # Vertical line
                    line_angle = 90
                else:
                    line_angle = degrees(atan2((y2 - y1) / (x2 - x1)))
                
                # Check if the line is approximately horizontal (within threshold)
                if line_angle <= self.horizontal_angle_threshold:
                    # Calculate line width (horizontal span)
                    line_width = abs(x2 - x1)
                    
                    # Check if line spans a significant portion of the image width
                    if line_width > self.min_line_length:
                        # Calculate line position (y-coordinate) - average of endpoints
                        line_position = (y1 + y2) / 2
                        
                        # Calculate distance from bottom of the frame
                        line_distance = self.height - line_position
                        
                        potential_stop_lines.append({
                            'points': (x1, y1, x2, y2),
                            'angle': line_angle,
                            'distance': line_distance,
                            'width': line_width
                        })
        
        # Select the best stop line candidate
        if potential_stop_lines:
            # Sort by width (descending) and then by distance (ascending)
            potential_stop_lines.sort(key=lambda x: (-x['width'], x['distance']))
            best_line = potential_stop_lines[0]
            
            x1, y1, x2, y2 = best_line['points']
            distance = best_line['distance']
            angle = best_line['angle']
            
            # Update detection history
            self.previous_detections.append(1)
            if len(self.previous_detections) > self.detection_history_size:
                self.previous_detections.pop(0)
            
            # Draw the detected stop line (only visual, no text)
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
            
            detected = True
        else:
            # No potential stop lines found
            self.previous_detections.append(0)
            if len(self.previous_detections) > self.detection_history_size:
                self.previous_detections.pop(0)
        
        # Check if we have consistent detections
        if sum(self.previous_detections) >= self.detection_threshold:
            detected = True
        else:
            detected = False
        
        # Always draw the ROI polygon on the frame
        cv2.polylines(frame, [self.roi_polygon], True, self.roi_color, self.roi_thickness)
        
        # Debug logging to console
        if self.logging and detected:
            print(f"Stop line detected: Distance={distance:.1f}px, Angle={angle:.1f}°")
        
        return frame, (detected, distance, angle)