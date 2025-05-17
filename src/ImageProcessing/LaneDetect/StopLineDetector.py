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
        self.min_line_length = int(width * 0.2)  # Minimum length of line
        self.max_line_gap = int(width * 0.05)    # Maximum allowed gap between line segments
        self.horizontal_angle_threshold = 15     # Maximum angle from horizontal (in degrees)
        
        # Define polygon ROI for stop line detection
        self.roi_polygon = np.array([
            [int(width * 0.2), int(height * 0.7)],    # Bottom left
            [int(width * 0.2), int(height * 0.4)],    # Top left
            [int(width * 0.8), int(height * 0.4)],    # Top right
            [int(width * 0.8), int(height * 0.7)]     # Bottom right
        ], np.int32)
        
        # ROI polygon color and thickness for visualization
        self.roi_color = (150, 50, 80)
        self.roi_thickness = 2


        # Frame retention parameters
        self.last_detected_line = None  # Store the last detected line data
        self.frames_to_retain = 5       # Number of frames to retain the last detected line
        self.frames_since_last_detection = 0


    def process_frame(self, frame: np.ndarray, edges: np.ndarray):
        detected = False
        distance = 0.0
        angle = 0.0
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
            threshold=10,
            minLineLength=self.min_line_length/2,
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
                    line_angle = degrees(atan2((y2 - y1) , (x2 - x1)))
                
                # Check if the line is approximately horizontal (within threshold)
                if abs(line_angle) <= self.horizontal_angle_threshold:
                    # Calculate line width (horizontal span)
                    line_width = abs(x2 - x1)
                    
                    # Check if line spans a significant portion of the image width
                    if line_width > self.min_line_length:
                        # Calculate line position (y-coordinate) closest to the bottom of the frame
                        line_position = max(y1, y2)
                        
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
            # Find the most left and down point and the most right and down point
            left_down = min(potential_stop_lines, key=lambda x: (x['points'][1], x['points'][0]))
            right_down = min(potential_stop_lines, key=lambda x: (x['points'][3], x['points'][2]))
            
            x1, y1 = left_down['points'][:2]
            x2, y2 = right_down['points'][2:]
            best_line = {'points': (x1, y1, x2, y2)}
            
            distance = self.height - max(y1, y2)  # Distance from the bottom of the image
            angle = degrees(atan2((y2 - y1), (x2 - x1)))
            
            # Draw the detected stop line (only visual, no text)
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
            
            detected = True

            # Update last detected line and reset frame counter
            self.last_detected_line = best_line
            self.frames_since_last_detection = 0
        else:
            # If no line is detected, check if we can use the last detected line
            if self.last_detected_line and self.frames_since_last_detection < self.frames_to_retain:
                best_line = self.last_detected_line
                x1, y1, x2, y2 = best_line['points']
                distance = self.height - max(y1, y2)
                angle = degrees(atan2((y2 - y1), (x2 - x1)))
                
                # Draw the last detected stop line
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                
                detected = True
                self.frames_since_last_detection += 1
            else:
                # Clear the last detected line if retention period is over
                self.last_detected_line = None
                detected = False

        # Always draw the ROI polygon on the frame
        cv2.polylines(frame, [self.roi_polygon], True, self.roi_color, self.roi_thickness)
        
        # Debug logging to console
        #if self.logging and detected:
        #    print(f"Stop line detected: Distance={distance:.1f}px, Angle={angle:.1f}°")
        
        return frame, (detected, float(distance), float(angle))