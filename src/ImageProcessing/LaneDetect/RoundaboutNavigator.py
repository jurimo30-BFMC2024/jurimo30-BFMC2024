import cv2
import numpy as np

class RoundaboutNavigator:
    def __init__(self, width: int, height: int, logging, debugging=False):
        self.width = width
        self.height = height
        self.logging = logging
        self.debugging = debugging
        self.target_distance_from_left = width * 0.45  # Podešavanje razmaka od leve linije
        self.exit_region = (int(width * 0.70), int(height * 0.2), int(width*0.95), int(height * 0.8))  # Gornji desni deo slike
        self.left_search_region = (0, int(height * 0.7), int(width * 0.4), height)  # Donji levi deo slike
        self.oldAngle = 0
    
    def detect_left_line(self, edges):
        x1, y1, x2, y2 = self.left_search_region
        roi = edges[y1:y2, x1:x2]
        lines = cv2.HoughLinesP(roi, rho=1, theta=np.pi / 180, threshold=5, minLineLength=8, maxLineGap=25)
        left_lines = []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                x1 += self.left_search_region[0]
                x2 += self.left_search_region[0]
                y1 += self.left_search_region[1]
                y2 += self.left_search_region[1]

                left_lines.append((x1, y1, x2, y2))
        
        return left_lines
    
    def calculate_steering_angle(self, left_lines):
        if not left_lines:
            return 0  # Ako nema linije, vozilo ide pravo

        # Calculate the x-coordinate of the point at the fixed distance from the left line
        avg_x = np.mean([line[0] for line in left_lines])  # Prosečna x koordinata leve linije
        target_x = avg_x + self.target_distance_from_left

        # Calculate the error between the center of the image and the target point
        center_x = self.width // 2
        error = center_x - target_x

        # Calculate the steering angle based on the error
        max_steering_angle = 25  # Ograničenje ugla
        angle = np.clip(error * 0.2, -max_steering_angle, max_steering_angle)

        return angle
    
    def draw_lines(self, frame, lines, color=(0, 0, 255)):
        if lines is not None:
            for x1, y1, x2, y2 in lines:
                cv2.line(frame, (x1, y1), (x2, y2), color, 3)

    def draw_rectangle(self, frame, region, color=(255, 0, 0)):
        # Draw a rectangle on the frame based on the given region
        x1, y1, x2, y2 = region
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    def draw_center_and_target(self, frame, left_lines):
        # Draw the center of the image
        center_x = self.width // 2
        center_y = self.height // 2
        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)  # Green dot for the center

        # Draw the target point at a fixed distance from the left line
        if left_lines:
            avg_x = np.mean([line[0] for line in left_lines])  # Prosečna x koordinata leve linije
            target_x = int(avg_x + self.target_distance_from_left)
            cv2.circle(frame, (target_x, center_y), 5, (255, 0, 0), -1)  # Blue dot for the target point

    def overlay_info(self, frame, steering_angle, error):
        # Overlay steering angle and alignment error on the top of the frame
        text = f"Steering Angle: {steering_angle:.2f}° | Alignment Error: {error:.2f}px"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    def process_frame(self, frame, edges):
        left_lines = self.detect_left_line(edges)
        steering_angle = self.calculate_steering_angle(left_lines) * 3
        if steering_angle == 0:
            steering_angle = 25
        
        if steering_angle < 0:
            steering_angle = 0
        self.oldAngle = steering_angle

        # Calculate alignment error for overlay
        avg_x = np.mean([line[0] for line in left_lines]) if left_lines else self.width // 2
        target_x = avg_x + self.target_distance_from_left if left_lines else self.target_distance_from_left
        center_x = self.width // 2
        alignment_error = center_x - target_x

        if self.debugging:
            self.draw_lines(frame, left_lines, (0, 0, 255))  # Red left line
            self.draw_rectangle(frame, self.left_search_region, (255, 0, 0))  # Blue search region
            self.draw_center_and_target(frame, left_lines)  # Draw center and target point
            # self.overlay_info(frame, steering_angle, alignment_error)  # Overlay info
        
        return frame, -steering_angle



