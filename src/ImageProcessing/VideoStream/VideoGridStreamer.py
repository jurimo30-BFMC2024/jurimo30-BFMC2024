import cv2
from flask import Flask, Response
import time
import numpy as np
from threading import Lock

class VideoGridStreamer:
    def __init__(self, grid_rows, grid_cols, width, height):
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        self.frames = [[None for _ in range(grid_cols)] for _ in range(grid_rows)]
        self.default_frame = self.create_blank_frame([width, height])  # Create a blank square
        self.app = Flask(__name__)
        self.app.add_url_rule('/', 'video_feed', self.video_feed)
        self.mutex = Lock()

    def create_blank_frame(self, size):
        """Create a blank frame of given size (black image)."""
        return np.zeros((size[1], size[0], 3), dtype=np.uint8)  # (height, width, channels)

    def display_frame(self, frame, row, col):
        """Update the frame for a specific grid square."""
        with self.mutex:
            if 0 <= row < self.grid_rows and 0 <= col < self.grid_cols:
                resized_frame = cv2.resize(frame, (self.default_frame.shape[1], self.default_frame.shape[0]))
                self.frames[row][col] = resized_frame

    def generate_frames(self):
        while True:
            # Fill in the grid
            grid_frame = self.compose_grid()

            # Encode the grid frame
            ret, buffer = cv2.imencode('.jpg', grid_frame)
            grid_frame = buffer.tobytes()

            time.sleep(0.05)  # Frame rate control

            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + grid_frame + b'\r\n')
                

    def compose_grid(self):
        """Compose the entire grid from current frames."""
        with self.mutex:
            rows = []
            for row in self.frames:
                # Replace None with the default frame
                row_frames = [frame if frame is not None else self.default_frame for frame in row]
                rows.append(cv2.hconcat(row_frames))
            return cv2.vconcat(rows)

    def video_feed(self):
        """Video feed endpoint for Flask."""
        return Response(self.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def run(self, host='0.0.0.0', port=5000):
        """Run the Flask app."""
        self.app.run(host=host, port=port, debug=False)

# Example Usage
if __name__ == "__main__":
    streamer = VideoGridStreamer(grid_rows=1, grid_cols=2, width=1024, height=540)

    def readVideo(path, pos):
        cap = cv2.VideoCapture(path)
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart the video when it ends
                continue

            time.sleep(0.1)  # Frame rate control

            streamer.display_frame(frame, pos[0], pos[1])

    from threading import Thread

    worker1 = Thread(target=readVideo, args=("./leva.avi", [0,0]), daemon=True).start()
    worker2 = Thread(target=readVideo, args=("./output_video1734731379.7589087.avi", [0,1]), daemon=True).start()

    streamer.run()
