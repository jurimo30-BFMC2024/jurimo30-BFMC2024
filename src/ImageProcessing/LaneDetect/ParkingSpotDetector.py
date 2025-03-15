import cv2
import numpy as np
import math

class ParkingSpotDetector:
    def __init__(self):
        pass

    def region_of_interest(self, image):
        height, width = image.shape[:2]
        polygons = np.array([
            [(0, height), (width * 0.2, height), (width * 0.2, height - height * 0.33), (0, height - height * 0.33)],
            [(width * 0.8, height), (width, height), (width, height - height * 0.33), (width * 0.8, height - height * 0.33)],
        ], dtype=np.int32)
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, polygons, (255, 255, 255))
        return cv2.bitwise_and(image, mask)

    def calculate_line_angle(self, x1, y1, x2, y2):
        return math.degrees(math.atan2(y2 - y1, x2 - x1))

    def calculate_line_length(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def make_parking_line(self, lines):
        center_line = 270 // 2
        left_x, right_x, left_y, right_y = [], [], [], []

        for line in lines:
            x1, y1, x2, y2 = line
            if x1 < center_line:
                left_y.extend([y1, y2])
                left_x.append(max(x1, x2))
            else:
                right_y.extend([y1, y2])
                right_x.append(min(x1, x2))

        if left_x and right_x and left_y and right_y:
            x1 = sum(left_x) // len(left_x)
            x2 = sum(right_x) // len(right_x)
            y1 = sum(left_y) // len(left_y)
            y2 = sum(right_y) // len(right_y)
            return x1, y1, x2, y2
        return None

    def detect_parking_spots(self, edges):
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 20, minLineLength=20, maxLineGap=10)

        horizontal_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = self.calculate_line_angle(x1, y1, x2, y2)
                if abs(angle) < 10:
                    horizontal_lines.append((x1, y1, x2, y2))
            return self.make_parking_line(horizontal_lines)
        return None

    def process_frame(self, frame, edges):
        roi = self.region_of_interest(edges)
        line = self.detect_parking_spots(roi)
        if line:
            x1, y1, x2, y2 = line
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
        return frame, line

if __name__ == "__main__":
    video_path = "./2.mp4"
    video_capture = cv2.VideoCapture(video_path)
    if not video_capture.isOpened():
        raise ValueError("Error: Could not open video file.")

    detector = ParkingSpotDetector()
    while video_capture.isOpened():
        ret, frame = video_capture.read()
        if not ret:
            break
        processed_frame, line = detector.process_frame(frame)
        cv2.imshow("Parking Spot Detection", processed_frame)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            print("Exiting video playback.")
            break
    
    video_capture.release()
    cv2.destroyAllWindows()
