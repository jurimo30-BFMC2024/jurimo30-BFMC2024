from multiprocessing import Process, Manager, Condition
import cv2
from flask import Flask, Response
import time
import numpy as np


class VideoGridStreamer:
    def __init__(self, grid_rows, grid_cols, width, height):
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        self.default_frame = self.create_blank_frame([width, height])
        self.manager = Manager()
        self.shared_frames = self.manager.list(
            [self.manager.list([None for _ in range(grid_cols)]) for _ in range(grid_rows)]
        )
        self.condition = Condition()  # Synchronization primitive
        self.flask_process = None

        # Flask app setup
        self.app = Flask(__name__)
        self.app.add_url_rule('/', 'video_feed', self.video_feed)

    def create_blank_frame(self, size):
        """Create a blank frame of given size (black image)."""
        return np.zeros((size[1], size[0], 3), dtype=np.uint8)  # (height, width, channels)

    def generate_frames(self):
        while True:
            with self.condition:
                self.condition.wait()  # Wait for the grid to be updated

                # Compose the grid using shared memory frames
                grid_frame = self.compose_grid()

                # Encode the grid frame
                ret, buffer = cv2.imencode('.jpg', grid_frame)
                grid_frame = buffer.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + grid_frame + b'\r\n')

    def compose_grid(self):
        """Compose the entire grid from shared frames."""
        rows = []
        for row in self.shared_frames:
            row_frames = [
                np.frombuffer(frame, dtype=np.uint8).reshape(self.default_frame.shape)
                if frame is not None else self.default_frame
                for frame in row
            ]
            rows.append(cv2.hconcat(row_frames))
        return cv2.vconcat(rows)

    def video_feed(self):
        """Video feed endpoint for Flask."""
        return Response(self.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def display_frame(self, frame, row, col):
        """Update the frame for a specific grid square."""
        if 0 <= row < self.grid_rows and 0 <= col < self.grid_cols:
            resized_frame = cv2.resize(frame, (self.default_frame.shape[1], self.default_frame.shape[0]))
            with self.condition:
                self.shared_frames[row][col] = resized_frame.tobytes()
                self.condition.notify()  # Notify generate_frames that the grid has been updated

    def start_flask_server(self, host='0.0.0.0', port=5000):
        """Start the Flask server."""
        self.app.run(host=host, port=port, debug=False)

    def start(self, host='0.0.0.0', port=5000):
        """Run the Flask server in a separate process."""
        self.flask_process = Process(
            target=self.start_flask_server,
            args=(host, port),
            daemon=True
        )
        self.flask_process.start()

    def stop(self):
        """Stop the Flask server."""
        if self.flask_process and self.flask_process.is_alive():
            self.flask_process.terminate()
            self.flask_process.join()


# Example Usage
if __name__ == "__main__":
    streamer = VideoGridStreamer(grid_rows=1, grid_cols=2, width=1024, height=540)
    streamer.start()

    def read_video(path, pos):
        cap = cv2.VideoCapture(path)
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart the video when it ends
                continue

            time.sleep(0.1)  # Frame rate control
            streamer.display_frame(frame, pos[0], pos[1])

    from threading import Thread

    worker1 = Thread(target=read_video, args=("./leva.avi", [0, 0]), daemon=True)
    worker2 = Thread(target=read_video, args=("./output_video1734731379.7589087.avi", [0, 1]), daemon=True)

    worker1.start()
    worker2.start()

    try:
        worker1.join()
        worker2.join()
    except KeyboardInterrupt:
        streamer.stop()
