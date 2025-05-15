import cv2
import numpy as np
import math
import time
from collections import deque

class LaneDetector:
    def __init__(self, width: int, height: int, logging, debugging=False, pc=False, camera_fov_degrees: float = 79.3):
        self.debugging = debugging
        self.logging = logging
        self.camera_fov_degrees = camera_fov_degrees
        self.width = width
        self.height = height
        self.pc = pc

        # Define road region for processing
        self.roadReg = np.array([[
            (int(self.width * 0.02), self.height - int(self.height * 0.02)),   # donji lijevi ugao
            (int(self.width * 0.20), self.height - int(self.height * 0.02)),  # donji lijevi prije ulegnuća
            (int(self.width * 0.28), self.height - int(self.height * 0.39)), # donji lijevi ulegnuće
            (int(self.width * 0.68), self.height - int(self.height * 0.39)), # donji desni ulegnuće
            (int(self.width * 0.76), self.height - int(self.height * 0.02)),  # donji desni prije ulegnuća
            (int(self.width * 0.98), self.height - int(self.height * 0.02)),  # donji desni ugao
            (int(self.width * 0.9), self.height // 2 - int(self.height * 0.15)), # gornji desni ugao
            (int(self.width * 0.1), self.height // 2 - int(self.height * 0.15))  # gornji lijevi ugao
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

        # Filter out puddles and other irregular shapes
        self.min_lane_line_length = 15  # Minimum length for lane lines (pixels)

        # Measurement point - height in the image where we calculate lane center
        self.measure_height = int(self.height * 0.50)  # 80% down the image

        # Colors for visualization
        self.color_left_lane = (255, 100, 0)    # Blue-ish
        self.color_right_lane = (0, 0, 255)     # Red
        self.color_road_region = (100, 255, 0)    # Green

    def region_of_interest(self, img):
        """Apply a region of interest mask to focus only on the road area"""
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, [self.roadReg], 255)  # Use list for single polygon
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
                # Avoid division by zero for vertical lines
                if x2 == x1:
                    slope = np.inf  # Represent vertical lines with infinity
                else:
                    slope = (y2 - y1) / (x2 - x1)

                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                midpoint_x = (x1 + x2) / 2

                # Filter out horizontal lines and very short lines
                # Also filter based on slope thresholds
                if abs(slope) < self.min_slope_threshold or length < self.min_lane_line_length:
                    # Keep near-vertical lines if slope is inf
                    if slope != np.inf:
                        continue

                # Classification based on position and slope
                if slope < 0:  # Potential left line (negative slope)
                    # Allow lines slightly to the right of center in curves
                    if midpoint_x < image_center_x + int(self.width * 0.15):
                        left_lines.append(line)
                elif slope > 0:  # Potential right line (positive slope)
                    # Allow lines slightly to the left of center in curves
                    if midpoint_x > image_center_x - int(self.width * 0.15):
                        right_lines.append(line)
                elif slope == np.inf:  # Vertical lines - classify based on position
                    if midpoint_x < image_center_x:
                        left_lines.append(line)
                    else:
                        right_lines.append(line)

        return left_lines, right_lines

    def extrapolate_lane_line(self, lines_history, height):
        """Extrapolate a lane line to reach the given height in the image using line history."""
        x_coords = []
        y_coords = []

        # Use all lines from the history for robustness
        all_lines = [line for lines_group in lines_history for line in lines_group]

        if not all_lines:  # Check if the flattened list of lines is empty
            return None

        for line in all_lines:
            for x1, y1, x2, y2 in line:
                # Ensure points are reasonably separated vertically to avoid division by zero/instability
                if abs(y1 - y2) > 5:
                    x_coords.extend([x1, x2])
                    y_coords.extend([y1, y2])

        if len(x_coords) < 2:  # Need at least two points (one line) to fit
            return None

        try:
            # Fit a polynomial (degree 1 - a line) to the points
            fit = np.polyfit(y_coords, x_coords, 1)

            # Calculate x-coordinate at the given height
            x = int(fit[0] * height + fit[1])

            # Add basic sanity check for extrapolated x-coordinate
            if 0 <= x <= self.width:
                return x
            else:
                if self.logging:
                    print(f"Warning: Extrapolated X ({x}) out of bounds.")
                return None  # Out of bounds

        except (np.linalg.LinAlgError, ValueError) as e:
            if self.logging:
                print(f"Warning: Polyfit failed - {e}")
            return None  # Fitting failed

    def process_frame(self, edges: np.ndarray, frame_to_draw_on: np.ndarray):
        """Process edges, detect lanes, draw on frame, and return drawn frame + positions."""
        # Apply region of interest mask to edges
        roi_edges = self.region_of_interest(edges)

        # Detect lines on the masked edges
        lines = self.detect_lines(roi_edges, 5)

        # Filter and separate left and right lane lines
        left_lines, right_lines = self.filter_lines(lines)

        # Update line history
        if left_lines:
            self.left_lines_history.append(left_lines)
        if right_lines:
            self.right_lines_history.append(right_lines)

        # Get stable lines from history (flattened list)
        left_lines_stable = [line for lines_group in self.left_lines_history for line in lines_group]
        right_lines_stable = [line for lines_group in self.right_lines_history for line in lines_group]

        # Find lane line x-coordinates at the measurement height using history
        left_x = self.extrapolate_lane_line(self.left_lines_history, self.measure_height)
        right_x = self.extrapolate_lane_line(self.right_lines_history, self.measure_height)

        # Safety checks for lane line positions
        # Check if lines are valid
        if left_x is not None and right_x is not None:
            # Check if right lane is to the left of left lane (invalid)
            if right_x < left_x:
                if self.logging and self.debugging:
                    print(f"Warning: Right lane ({right_x}) is to the left of left lane ({left_x}). Ignoring.")
                right_x = None
                
            # Check if lines are too close to each other (less than 50 pixels apart)
            elif right_x - left_x < 50:
                if self.logging and self.debugging:
                    print(f"Warning: Lane lines are too close: {right_x - left_x} px. Minimum is 50px. Ignoring right line.")
                right_x = None

        # --- Drawing Logic ---
        # Draw region of interest with transparent overlay
        road_mask = np.zeros_like(frame_to_draw_on)
        cv2.fillPoly(road_mask, [self.roadReg], (0, 50, 0)) # Use list for single polygon
        frame_to_draw_on = cv2.addWeighted(frame_to_draw_on, 1.0, road_mask, 0.3, 0)

        # Draw region outline
        points = self.roadReg.reshape((-1, 1, 2))
        cv2.polylines(frame_to_draw_on, [points], isClosed=True, color=self.color_road_region, thickness=3)

        # Draw left lane lines if available
        if left_lines_stable:
            for line in left_lines_stable:
                for x1, y1, x2, y2 in line:
                    cv2.line(frame_to_draw_on, (x1, y1), (x2, y2), self.color_left_lane, 2)

        # Draw right lane lines if available
        if right_lines_stable:
            for line in right_lines_stable:
                for x1, y1, x2, y2 in line:
                    cv2.line(frame_to_draw_on, (x1, y1), (x2, y2), self.color_right_lane, 2)

        # Draw measurement points
        if left_x is not None:
            cv2.circle(frame_to_draw_on, (left_x, self.measure_height), radius=7, color=(255, 255, 0), thickness=-1)
        if right_x is not None:
            cv2.circle(frame_to_draw_on, (right_x, self.measure_height), radius=7, color=(255, 0, 255), thickness=-1)

        # --- End of Drawing Logic ---

        # Debug log for terminal (simplified)
        if self.logging and self.debugging:
            print(f"Lane Detection: LeftX={left_x}, RightX={right_x}, "
                  f"LeftLines={len(left_lines_stable)}, RightLines={len(right_lines_stable)}")

        # Return the frame with drawings and the detection results
        return frame_to_draw_on, left_x, right_x