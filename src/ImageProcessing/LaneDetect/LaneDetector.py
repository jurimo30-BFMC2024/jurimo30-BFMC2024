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
            (int(self.width * 0.22), self.height - int(self.height * 0.02)),  # donji lijevi prije ulegnuća
            (int(self.width * 0.30), self.height - int(self.height * 0.35)), # donji lijevi ulegnuće
            (int(self.width * 0.65), self.height - int(self.height * 0.35)), # donji desni ulegnuće
            (int(self.width * 0.73), self.height - int(self.height * 0.02)),  # donji desni prije ulegnuća
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

        # Filter out puddles and other irregular shapes
        self.min_lane_line_length = 15  # Minimum length for lane lines (pixels)

        # Measurement point - height in the image where we calculate lane center
        # Promena sa 80% na 65% visine slike - tačka će biti više udaljena od dna
        self.measure_height = int(self.height * 0.55)  # 65% down the image (povećana udaljenost od dna)

        # Colors for visualization
        self.color_left_lane = (255, 100, 0)    # Blue-ish
        self.color_right_lane = (0, 0, 255)     # Red
        self.color_road_region = (100, 255, 0)    # Green

        # Zadnje vidljive pozicije linija
        self.last_left_x = None
        self.last_right_x = None

        # Flegovi vidljivosti
        self.left_visible = False
        self.right_visible = False

        # Boja za tačke pozicije linija
        self.color_lane_point = (0, 255, 255)  # Žuta
        self.lane_point_radius = 6
        
        # Debug visualization parameters
        self.offset_from_left = 30  # Fixed distance right of left lane (in pixels)
        self.offset_from_right = 30  # Fixed distance left of right lane (in pixels)
        self.debug_point_radius = 5
        self.debug_center_line_length = 75
        self.debug_points_spacing = 20

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
        if lines is None:
            return [], []

        image_center_x = self.width // 2

        # Prvo pronađimo sve potencijalne linije i njihove srednje tačke
        potential_left_lines = []
        potential_right_lines = []

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

                # Strogo filtriranje - desna linija mora biti desno, leva levo od centra
                # Ignorišemo linije koje su na pogrešnoj strani puta bez obzira na nagib
                if slope < 0:  # Negativni nagib - potencijalna leva linija
                    # Leva linija mora biti levo od centra
                    if midpoint_x < image_center_x:
                        potential_left_lines.append((line, midpoint_x))
                    # Ignorišemo leve linije koje su desno od centra
                elif slope > 0:  # Pozitivni nagib - potencijalna desna linija
                    # Desna linija mora biti desno od centra
                    if midpoint_x > image_center_x:
                        potential_right_lines.append((line, midpoint_x))
                elif slope == np.inf:  # Vertikalne linije - klasifikuj prema poziciji
                    if midpoint_x < image_center_x:
                        potential_left_lines.append((line, midpoint_x))
                    else:
                        potential_right_lines.append((line, midpoint_x))

        # Minimum razmak između leve i desne linije
        min_lane_width = int(self.width * 0.2)  # Minimum 20% širine slike
        
        # Za slučaj pešačkog prelaza sa više paralelnih linija:
        # Ako imamo više potencijalnih levih linija, uzimamo samo najlevlju
        # Ako imamo više potencijalnih desnih linija, uzimamo samo najdešnju
        final_left_lines = []
        final_right_lines = []
        
        # Sortiramo linije po x poziciji
        potential_left_lines.sort(key=lambda item: item[1])  # Sortiramo po midpoint_x
        potential_right_lines.sort(key=lambda item: item[1])  # Sortiramo po midpoint_x
        
        # U slučaju više paralelnih linija, samo uzimamo najlevlju levu i najdešnju desnu liniju
        if len(potential_left_lines) > 1:
            # Provera za paralelne linije - uzimamo samo najlevlju
            left_most_line = potential_left_lines[0]
            final_left_lines.append(left_most_line[0])
        else:
            # Ako nema više paralelnih linija, koristimo sve
            for line_data, _ in potential_left_lines:
                final_left_lines.append(line_data)
                
        if len(potential_right_lines) > 1:
            # Provera za paralelne linije - uzimamo samo najdešnju
            right_most_line = potential_right_lines[-1]
            final_right_lines.append(right_most_line[0])
        else:
            # Ako nema više paralelnih linija, koristimo sve
            for line_data, _ in potential_right_lines:
                final_right_lines.append(line_data)
        
        # Provera da li su krajnje leva i desna linija preblizu jedna drugoj
        if final_left_lines and final_right_lines:
            # Nađemo srednju tačku krajnje leve i krajnje desne linije
            left_x_sum = 0
            left_count = 0
            for line in final_left_lines:
                for x1, y1, x2, y2 in line:
                    left_x_sum += (x1 + x2) / 2
                    left_count += 1
            
            right_x_sum = 0
            right_count = 0
            for line in final_right_lines:
                for x1, y1, x2, y2 in line:
                    right_x_sum += (x1 + x2) / 2
                    right_count += 1
            
            if left_count > 0 and right_count > 0:
                left_avg_x = left_x_sum / left_count
                right_avg_x = right_x_sum / right_count
                
                # Ako su linije preblizu jedna drugoj, zadrži samo onu koja je dalje od centra
                if right_avg_x - left_avg_x < min_lane_width:
                    if image_center_x - left_avg_x > right_avg_x - image_center_x:
                        final_right_lines = []  # Ignoriši desne linije
                    else:
                        final_left_lines = []  # Ignoriši leve linije

        return final_left_lines, final_right_lines

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
        current_left_x = self.extrapolate_lane_line(self.left_lines_history, self.measure_height)
        current_right_x = self.extrapolate_lane_line(self.right_lines_history, self.measure_height)

        # Ažuriranje flegova vidljivosti - true SAMO ako su linije stvarno vidljive u trenutnom frejmu
        # Ključna promena: provjera postojanja linija u trenutnom frejmu, ne iz historije
        self.left_visible = len(left_lines) > 0 and current_left_x is not None
        self.right_visible = len(right_lines) > 0 and current_right_x is not None

        # Ažuriranje zadnjih pozicija ako su vidljive
        if current_left_x is not None:
            self.last_left_x = current_left_x
        if current_right_x is not None:
            self.last_right_x = current_right_x

        # Vrednosti koje ćemo vratiti - ili trenutne ako su vidljive, ili zadnje poznate
        left_x = current_left_x if current_left_x is not None else self.last_left_x
        right_x = current_right_x if current_right_x is not None else self.last_right_x

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

        # Draw horizontal line at measurement height
        cv2.line(frame_to_draw_on, (0, self.measure_height), (self.width, self.measure_height), 
                 (255, 255, 255), 1, cv2.LINE_AA)

        # Draw left lane point
        if left_x is not None:
            # Prilagođena boja: žuta ako je vidljiva, tamnija ako koristimo zadnju poznatu
            point_color = self.color_lane_point if self.left_visible else (0, 128, 128)
            cv2.circle(frame_to_draw_on, (left_x, self.measure_height), 
                       self.lane_point_radius, point_color, -1)

        # Draw right lane point
        if right_x is not None:
            # Prilagođena boja: žuta ako je vidljiva, tamnija ako koristimo zadnju poznatu
            point_color = self.color_lane_point if self.right_visible else (0, 128, 128)
            cv2.circle(frame_to_draw_on, (right_x, self.measure_height), 
                       self.lane_point_radius, point_color, -1)

        # --- Debug Visualization ---
        if True:
            # Image center calculation
            image_center_x = int(self.width * 0.47)
            
            # Original debug elements
            # Reference points for debug
            if left_x is not None:
                cv2.circle(frame_to_draw_on, (left_x, self.measure_height), self.debug_point_radius, (255, 255, 255), -1)
                cv2.circle(frame_to_draw_on, (left_x + self.offset_from_left, self.measure_height), self.debug_point_radius, (255, 255, 255), -1)
                cv2.circle(frame_to_draw_on, (left_x - self.offset_from_left, self.measure_height), self.debug_point_radius, (255, 255, 255), -1)
            
            if right_x is not None:
                cv2.circle(frame_to_draw_on, (right_x, self.measure_height), self.debug_point_radius, (255, 255, 255), -1)
                cv2.circle(frame_to_draw_on, (right_x + self.offset_from_right, self.measure_height), self.debug_point_radius, (255, 255, 255), -1)
                cv2.circle(frame_to_draw_on, (right_x - self.offset_from_right, self.measure_height), self.debug_point_radius, (255, 255, 255), -1)

            # Debug center lines
            if left_x is not None:
                cv2.line(frame_to_draw_on, (left_x, self.measure_height), (left_x, 0), (255, 255, 255), 1, cv2.LINE_AA)
            if right_x is not None:
                cv2.line(frame_to_draw_on, (right_x, self.measure_height), (right_x, 0), (255, 255, 255), 1, cv2.LINE_AA)
                
            # New debug visualization in top 100px:
            
            # Center vertical line (75px long)
            start_y = 5
            # Convert floating point coordinates to integers

            cv2.line(frame_to_draw_on, (image_center_x, start_y), 
                    (image_center_x, start_y + self.debug_center_line_length), 
                    (255, 255, 255), 2, cv2.LINE_AA)
            
            # Calculate and draw the three vertically stacked points - ensuring they stay in top 100px:
            # Spacing adjusted to fit within top 100px
            point1_y = start_y + self.debug_center_line_length + 5  # First point just below the line
            point2_y = point1_y + 15  # Reduced spacing between points
            point3_y = point2_y + 15  # Reduced spacing between points
            
            # Point 1: Orange point showing center between lanes
            if left_x is not None and right_x is not None:
                lane_center_x = (left_x + right_x) // 2
                cv2.circle(frame_to_draw_on, (lane_center_x, point1_y), 
                          self.debug_point_radius, (0, 165, 255), -1)  # Orange
            
            # Point 2: Fixed distance left of right lane
            if right_x is not None:
                fixed_left_pt_x = right_x - self.offset_from_right
                cv2.circle(frame_to_draw_on, (fixed_left_pt_x, point2_y), 
                          self.debug_point_radius, (0, 255, 255), -1)  # Yellow
            
            # Point 3: Blue point fixed distance right of left lane
            if left_x is not None:
                fixed_right_pt_x = left_x + self.offset_from_left
                cv2.circle(frame_to_draw_on, (fixed_right_pt_x, point3_y), 
                          self.debug_point_radius, (255, 0, 0), -1)  # Blue

        # Debug log for terminal (simplified)
        if self.debugging:
            print(f"Lane Detection: LeftX={left_x} (visible={self.left_visible}), "
                  f"RightX={right_x} (visible={self.right_visible})")

        # Return the frame with drawings, detection results and visibility flags
        return frame_to_draw_on, left_x, right_x, self.left_visible, self.right_visible