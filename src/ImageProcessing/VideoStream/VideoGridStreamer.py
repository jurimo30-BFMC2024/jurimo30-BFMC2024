from multiprocessing import Process, Manager, Condition
import cv2
from flask import Flask, Response
import time
import numpy as np

class Frames:
    width=768
    height=405
    manager = Manager()
    default_frame = None
    shared_frames = manager.list()

class VideoGridStreamer:
    def __init__(self, grid_rows, grid_cols, fps: int = 10):
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        self.fps = fps

        for _ in range(grid_rows):
            Frames.shared_frames.append(Frames.manager.list([None for _ in range(grid_cols)]))

        Frames.default_frame = self.create_blank_frame([Frames.width, Frames.height])
        self.flask_process = None

        # Flask app setup
        self.app = Flask(__name__)
        self.app.add_url_rule('/', 'video_feed', self.video_feed)

    def __del__(self):
        """Stop the Flask server."""
        if self.flask_process and self.flask_process.is_alive():
            self.flask_process.terminate()
            self.flask_process.join()

    def create_blank_frame(self, size):
        """Create a blank frame of given size (black image)."""
        return np.zeros((size[1], size[0], 3), dtype=np.uint8)  # (height, width, channels)

    def generate_frames(self):
        t_per_frame = 1 / self.fps
        while True:
            exec_time = time.time()
            # Compose the grid using shared memory frames
            grid_frame = self.compose_grid()

            # Encode the grid frame
            ret, buffer = cv2.imencode('.jpg', grid_frame, [cv2.IMWRITE_JPEG_QUALITY, 40])
            grid_frame = buffer.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + grid_frame + b'\r\n')
            time_to_sleep = t_per_frame - (time.time() - exec_time)
            time.sleep(max(0, time_to_sleep))

    def compose_grid(self):
        """Compose the entire grid from shared frames."""
        rows = []
        for row in Frames.shared_frames:
            row_frames = [
                np.frombuffer(frame, dtype=np.uint8).reshape(Frames.default_frame.shape)
                if frame is not None else Frames.default_frame
                for frame in row
            ]
            rows.append(cv2.hconcat(row_frames))
        return cv2.vconcat(rows)

    def video_feed(self):
        """Video feed endpoint for Flask."""
        return Response(self.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

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

class VideoStream:
    def __init__(self, row, col):
        self.row = row
        self.col = col

    def display(self, frame):
        resized_frame = cv2.resize(frame, (Frames.width, Frames.height))
        try:
            Frames.shared_frames[self.row][self.col] = resized_frame.tobytes()
        except IndexError as e:
            pass

# Example Usage
if __name__ == "__main__":
    def read_video(path, pos):
        streamer = VideoStream(pos[0], pos[1])
        cap = cv2.VideoCapture(path)
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart the video when it ends
                continue

            time.sleep(0.1)  # Frame rate control
            streamer.display(frame)

    streamer = VideoGridStreamer(grid_rows=1, grid_cols=2)
    streamer.start()

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
