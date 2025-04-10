import cv2
import numpy as np

class RoundaboutNavigator:
    def __init__(self, width: int, height: int, logging, debugging=False):
        self.width = width
        self.height = height
        self.logging = logging
        self.debugging = debugging
        self.target_distance_from_left = width * 0.30  # Podešavanje razmaka od leve linije
        self.exit_region = (int(width * 0.70), int(height * 0.2), int(width*0.95), int(height * 0.8))  # Gornji desni deo slike
        self.left_search_region = (0, int(height * 0.6), int(width * 0.4), height)  # Donji levi deo slike
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
        
        avg_x = np.mean([line[0] for line in left_lines])  # Prosečna x koordinata linije
        error = avg_x - self.target_distance_from_left
        
        max_steering_angle = 25  # Ograničenje ugla
        angle = np.clip(error * -0.2, -max_steering_angle, max_steering_angle)
        
        return -angle

    
    def draw_rectangle(self, frame, region, color=(0, 255, 0)):
        x1, y1, x2, y2 = region
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    
    def draw_lines(self, frame, lines, color=(0, 0, 255)):
        if lines is not None:
            for x1, y1, x2, y2 in lines:
                cv2.line(frame, (x1, y1), (x2, y2), color, 3)
    
    def process_frame(self, frame, edges):
        left_lines = self.detect_left_line(edges)
        steering_angle = self.calculate_steering_angle(left_lines)
        if steering_angle == 0:
            steering_angle = self.oldAngle
        self.oldAngle = steering_angle
        exit_detected = self.detect_exit(edges)
        

        if self.debugging:
            self.draw_lines(frame, left_lines, (0, 0, 255))  # Crvena leva linija
            self.draw_rectangle(frame, self.exit_region, (0, 255, 0))  # Zelena zona izlaza
            self.draw_rectangle(frame, self.left_search_region, (255, 0, 0))  # Plava zona pretrage leve linije
            
            if exit_detected:
                cv2.putText(frame, "Exit Detected", (self.width // 2 - 100, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        return frame, float(steering_angle), float(exit_detected)



