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
            (int(self.width * 0.98), self.height - int(self.height * 0.2)),  
            (int(self.width * 0.8), self.height // 2 - int(self.height * 0.15)), # gornji desni ugao
            (int(self.width * 0.2), self.height // 2 - int(self.height * 0.15)),  # gornji lijevi ugao
            (int(self.width * 0.02), int(self.height*0.7))  # gornji lijevi ugao
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
        
        # Parametri za težinski sistem
        self.distance_threshold = 40.0  # Smanjena maksimalna udaljenost za punu težinu
        self.weight_decay = 0.05  # Pojačan faktor opadanja težine sa udaljenošću
        self.noise_penalty_distance = 120.0  # Udaljenost posle koje se primenjuje jaka penalizacija za šum

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

    def calculate_line_weight(self, midpoint_x, midpoint_y, length, is_left_lane):
        """Računa težinu linije na osnovu udaljenosti od starih pozicija"""
        base_weight = 1.0
        
        # Koristi zadnje poznate pozicije za računanje udaljenosti
        reference_x = None
        if is_left_lane and self.last_left_x is not None:
            reference_x = self.last_left_x
        elif not is_left_lane and self.last_right_x is not None:
            reference_x = self.last_right_x
        
        if reference_x is not None:
            # Računaj horizontalnu udaljenost od referentne pozicije
            distance = abs(midpoint_x - reference_x)
            
            # Progresivno smanjenje težine na osnovu udaljenosti
            if distance > self.distance_threshold:
                excess_distance = distance - self.distance_threshold
                weight_reduction = np.exp(-self.weight_decay * excess_distance)
                base_weight *= weight_reduction
                
                # Jaka penalizacija za linije vrlo daleko (šum)
                if distance > self.noise_penalty_distance:
                    base_weight *= 0.1  # Drastično smanjenje
                
                # Dodatno penalizuj kratke linije koje su daleko
                if length < 30:  # Kratke linije
                    base_weight *= 0.3  # Pojačana penalizacija
                elif length < 50:  # Srednje kratke linije
                    base_weight *= 0.6
        else:
            # Ako nema referentnu poziciju, proveri da li je linija na očekivanoj strani
            image_center_x = self.width // 2
            
            # Za leve linije - penalizuj ako su previše desno
            if is_left_lane and midpoint_x > image_center_x + (self.width * 0.1):
                base_weight *= 0.2  # Jaka penalizacija za leve linije desno od centra
            
            # Za desne linije - penalizuj ako su previše levo
            elif not is_left_lane and midpoint_x < image_center_x - (self.width * 0.1):
                base_weight *= 0.2  # Jaka penalizacija za desne linije levo od centra
        
        # Dodatni faktor za poziciju - penalizuj ekstremne pozicije
        if is_left_lane:
            # Leva linija ne sme biti u desnoj polovini
            if midpoint_x > self.width * 0.6:
                base_weight *= 0.05  # Skoro potpuna eliminacija
        else:
            # Desna linija ne sme biti u levoj polovini
            if midpoint_x < self.width * 0.4:
                base_weight *= 0.05  # Skoro potpuna eliminacija
        
        return max(base_weight, 0.01)  # Minimalna težina

    def filter_lines(self, lines):
        """Filter lines based on size, angle, and position to separate left and right lanes"""
        if lines is None:
            return [], []

        image_center_x = self.width // 2

        # Potential line containers with weights
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
                midpoint_y = (y1 + y2) / 2

                # For steep curves, relax the slope filtering to capture more angled lines
                # Dynamic slope threshold based on position in the image (tighter at bottom, looser at top)
                # This helps with curved roads where lines have varying slopes
                relative_height = midpoint_y / self.height
                min_slope_threshold = self.min_slope_threshold * (1.0 - relative_height * 0.5)  # Relax threshold for higher points

                # Filter out horizontal lines and very short lines
                if abs(slope) < min_slope_threshold or length < self.min_lane_line_length:
                    # Keep near-vertical lines if slope is inf
                    if slope != np.inf:
                        continue

                # More adaptive classification for curved roads
                # Instead of strictly classifying by slope sign, use a combination of position and slope
                # This allows detecting lines that might bend or have unusual angles in curves
                
                # Strože granice za klasifikaciju - smanjuje šum
                is_left_side = midpoint_x < image_center_x - (image_center_x * 0.05)  # Leva 45% slike
                is_far_left = midpoint_x < image_center_x * 0.25  # Leva 25% slike
                
                is_right_side = midpoint_x > image_center_x + (image_center_x * 0.05)  # Desna 45% slike
                is_far_right = midpoint_x > image_center_x * 1.75  # Desna 25% slike
                
                # Lane line classification using both position and slope
                if (slope < 0 and is_left_side) or (is_far_left and abs(slope) > min_slope_threshold):
                    # Računaj težinu za levu liniju
                    weight = self.calculate_line_weight(midpoint_x, midpoint_y, length, True)
                    # Dodatna provera - odbaci linije sa vrlo malom težinom
                    if weight > 0.1:
                        potential_left_lines.append((line, midpoint_x, length, abs(slope), weight))
                elif (slope > 0 and is_right_side) or (is_far_right and abs(slope) > min_slope_threshold):
                    # Računaj težinu za desnu liniju
                    weight = self.calculate_line_weight(midpoint_x, midpoint_y, length, False)
                    # Dodatna provera - odbaci linije sa vrlo malom težinom
                    if weight > 0.1:
                        potential_right_lines.append((line, midpoint_x, length, abs(slope), weight))
                elif slope == np.inf:  # Vertical lines - classify by position only
                    if midpoint_x < image_center_x:
                        weight = self.calculate_line_weight(midpoint_x, midpoint_y, length, True)
                        if weight > 0.1:
                            potential_left_lines.append((line, midpoint_x, length, float('inf'), weight))
                    else:
                        weight = self.calculate_line_weight(midpoint_x, midpoint_y, length, False)
                        if weight > 0.1:
                            potential_right_lines.append((line, midpoint_x, length, float('inf'), weight))

        # Minimum lane width for validation
        min_lane_width = int(self.width * 0.2)  # Minimum 20% of image width
        
        # Sort by weighted score (length * slope * weight^2) - kvadriramo težinu za pojačavanje efekata
        if potential_left_lines:
            potential_left_lines.sort(key=lambda item: item[2] * item[3] * (item[4] ** 2), reverse=True)
        
        if potential_right_lines:
            potential_right_lines.sort(key=lambda item: item[2] * item[3] * (item[4] ** 2), reverse=True)
        
        # Take the top candidates (most significant lines) - smanjeno za strožu selekciju
        max_lines_per_side = 2  # Smanjen broj linija po strani
        final_left_lines = [item[0] for item in potential_left_lines[:max_lines_per_side]] if potential_left_lines else []
        final_right_lines = [item[0] for item in potential_right_lines[:max_lines_per_side]] if potential_right_lines else []
        
        # Validation: check if the detected lanes are too close to each other
        # This helps filter out cases where the same line is detected as both left and right
        if final_left_lines and final_right_lines:
            left_x_points = []
            right_x_points = []
            
            for line in final_left_lines:
                for x1, y1, x2, y2 in line:
                    left_x_points.extend([x1, x2])
            
            for line in final_right_lines:
                for x1, y1, x2, y2 in line:
                    right_x_points.extend([x1, x2])
            
            if left_x_points and right_x_points:
                left_avg_x = sum(left_x_points) / len(left_x_points)
                right_avg_x = sum(right_x_points) / len(right_x_points)
                
                # If lanes are too close, keep only the one with more supporting evidence
                if right_avg_x - left_avg_x < min_lane_width:
                    if len(left_x_points) > len(right_x_points):
                        final_right_lines = []  # Left lane has more evidence
                    else:
                        final_left_lines = []  # Right lane has more evidence

        return final_left_lines, final_right_lines

    def extrapolate_lane_line(self, lines_history, height):
        """Extrapolate a lane line to reach the given height in the image using line history."""
        x_coords = []
        y_coords = []

        # Use all lines from the history for robustness
        all_lines = [line for lines_history in lines_history for line in lines_history]

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
                if self.debugging:
                    print(f"Warning: Extrapolated X ({x}) out of bounds.")
                return None  # Out of bounds

        except (np.linalg.LinAlgError, ValueError) as e:
            if self.debugging:
                print(f"Warning: Polyfit failed - {e}")
            return None  # Fitting failed

    def process_frame(self, edges: np.ndarray, frame_to_draw_on: np.ndarray, left_offset_distance=50, right_offset_distance=50):
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

        # --- End of Drawing Logic ---

        # Debug log for terminal (simplified)
        if self.debugging:
            print(f"Lane Detection: LeftX={left_x} (visible={self.left_visible}), "
                  f"RightX={right_x} (visible={self.right_visible})")
                  
        # Draw debug lines at the top of the screen if requested
        if True:
            frame_to_draw_on = self.draw_debug_lines(
                frame_to_draw_on, 
                left_x, 
                right_x, 
                210, 
                130
            )

        # Return the frame with drawings, detection results and visibility flags
        return frame_to_draw_on, left_x, right_x, self.left_visible, self.right_visible

    def draw_debug_lines(self, frame, left_x, right_x, left_offset_distance, right_offset_distance):
        """
        Draw debug lines with staggered ending points all the way to the top of the screen:
        - Reference line at 47% of width
        - Yellow line showing the center between left and right lanes
        - Blue line at fixed distance right from left lane
        - Red line at fixed distance left from right lane
        """
        # End positions at different heights
        ref_end_y = int(self.height * 0.2)       # 20% from top
        center_end_y = int(self.height * 0.25)   # 25% from top
        left_end_y = int(self.height * 0.3)      # 30% from top
        right_end_y = int(self.height * 0.35)    # 35% from top
        
        # Reference line at 47% width - from top to ref_end_y
        reference_x = int(self.width * 0.47)
        cv2.line(frame, (reference_x, 0), (reference_x, ref_end_y), 
                (255, 255, 255), 2)  # White line
                
        # If both lane positions are available
        if left_x is not None and right_x is not None:
            # Center line between lanes (yellow) - from top to center_end_y
            center_x = (left_x + right_x) // 2
            cv2.line(frame, (center_x, 0), (center_x, center_end_y), 
                    (0, 255, 255), 2)  # Yellow line
            
            # Blue line at fixed distance right from left lane - from top to left_end_y
            left_offset_x = left_x + left_offset_distance
            cv2.line(frame, (left_offset_x, 0), (left_offset_x, left_end_y), 
                    (255, 0, 0), 2)  # Blue line
            
            # Red line at fixed distance left from right lane - from top to right_end_y
            right_offset_x = right_x - right_offset_distance
            cv2.line(frame, (right_offset_x, 0), (right_offset_x, right_end_y), 
                    (0, 0, 255), 2)  # Red line

        return frame