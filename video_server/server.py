import cv2
import socket
import struct
import numpy as np
from threading import Lock, Thread, Semaphore
import tkinter as tk
from tkinter import messagebox, simpledialog
import time
import os

class MulticastServer:
    def __init__(self, multicast_group: str, port: int):
        self.multicast_group = multicast_group
        self.port = port
        self.sock = None
        self.lock = Lock()
        # self.semaphore = Semaphore(0)
        self.frames = {}  # Dictionary to store frames for each feed ID
        self._running = True

        self.root = tk.Tk()
        self.root.title("Video Server Control")
        self.root.geometry("400x60")
        # Add buttons
        self.button_frame = tk.Frame(self.root)
        self.button_frame.pack(pady=10)

        self.exit_button = tk.Button(self.button_frame, text="Exit", command=self.on_exit, width=12)
        self.exit_button.grid(row=0, column=0, padx=5)

        self.clear_button = tk.Button(self.button_frame, text="Clear", command=self.on_clear, width=12)
        self.clear_button.grid(row=0, column=1, padx=5)

        self.recording = False  # Initial state of the record switch
        self.record_button = tk.Button(self.button_frame, text="Recording: OFF", command=self.on_record_toggle, width=12)
        self.record_button.grid(row=0, column=2, padx=5)

    def on_exit(self):
        """Handle exit button press."""
        if messagebox.askyesno("Exit", "Are you sure you want to exit?"):
            self._running = False
            self.root.destroy()

    def on_clear(self):
        """Handle clear button press."""
        with self.lock:
            self.frames = {}

    def on_record_toggle(self):
        """Handle record toggle switch."""
        self.recording = not self.recording
        self.record_button.config(text="Recording: ON" if self.recording else "Recording: OFF")

    def setup_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.port))
        group = socket.inet_aton(self.multicast_group)
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    def display_in_grid(self):
        """
        Display all received frames in a grid dynamically with index in the corner.
        The window updates even when no new frames are received.
        """
        cv2.namedWindow("Feed Grid", cv2.WINDOW_NORMAL)  # Make the window resizable

        while self._running:
            # Get current frames or use a blank canvas if no frames exist
            with self.lock:
                images = list(self.frames.values())
                indexes = list(self.frames.keys())
                num_images = len(images)

            if num_images == 0:
                # Show a blank canvas when no frames are available
                blank_canvas = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(blank_canvas, "Waiting for frames...", (50, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow("Feed Grid", blank_canvas)
                cv2.waitKey(50)  # Avoid blocking, check for updates frequently
                continue

            # Calculate grid dimensions (square or slightly rectangular)
            grid_rows = int(np.ceil(np.sqrt(num_images)))
            grid_cols = int(np.ceil(num_images / grid_rows))

            # Find the maximum height and width of all images
            max_height = max(img.shape[0] for img in images)
            max_width = max(img.shape[1] for img in images)
            n_channels = images[0].shape[2] if len(images[0].shape) > 2 else 1

            # Create a blank canvas
            canvas_height = grid_rows * max_height
            canvas_width = grid_cols * max_width
            canvas = np.zeros((canvas_height, canvas_width, n_channels), dtype=np.uint8)

            # Place each image in the grid
            for idx, img in enumerate(images):
                r, c = divmod(idx, grid_cols)
                y_start, y_end = r * max_height, (r + 1) * max_height
                x_start, x_end = c * max_width, (c + 1) * max_width

                # Resize image to fit grid cell if necessary
                resized_img = cv2.resize(img, (max_width, max_height), interpolation=cv2.INTER_AREA)

                # Add index in the top-left corner of the image
                cv2.putText(resized_img, f"{indexes[idx]}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Place image in the canvas
                if n_channels == 1:  # Grayscale
                    canvas[y_start:y_end, x_start:x_end] = resized_img
                else:  # Color
                    canvas[y_start:y_end, x_start:x_end, :] = resized_img

            # Show the canvas
            cv2.imshow("Feed Grid", canvas)
            cv2.waitKey(50)

        cv2.destroyAllWindows()
    

    def record(self):
        fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Use 'XVID' or another codec if needed
        writers = {}
        recording = False
        recorded_videos_folder = './recorded_videos'
        tmp_folder = f'{recorded_videos_folder}/tmp'

        def ask_for_folder():
            # Ask for folder input when recording is stopped
            while True:
                folder_path = simpledialog.askstring("Input", "Enter something:")
                if not os.path.exists(f"{recorded_videos_folder}/{folder_path}"):
                    break
                else:
                    print(f"folder exists: {recorded_videos_folder}/{folder_path}")

            if folder_path:
                if os.path.exists(tmp_folder):
                    print(f"Recording saved in: {folder_path}")
                    os.rename(tmp_folder, f"{recorded_videos_folder}/{folder_path}")
                else:
                    print(f"Directory {tmp_folder} does not exist.")
            else:
                print("No folder selected.")

            

        while self._running:
            if not recording and self.recording:
                if not os.path.exists(tmp_folder):
                    os.makedirs(tmp_folder)
                recording = self.recording

            if recording:
                with self.lock:
                    for idx in self.frames:
                        if idx not in writers:
                            writers[idx] = cv2.VideoWriter(f'{tmp_folder}/output{idx}.avi', fourcc, 20.0, 
                                                        (self.frames[idx].shape[1], self.frames[idx].shape[0]))

                        if self.frames[idx] is not None:
                            writers[idx].write(self.frames[idx])

            if recording and not self.recording:
                # Release VideoWriter for each feed
                for idx in writers:
                    if writers[idx]:
                        writers[idx].release()
                        # print(f"VideoWriter for output{idx}.avi released.")
                writers.clear()

                # Call dialog box in a non-blocking way using after
                self.root.after(1, ask_for_folder)
                recording = False

            time.sleep(0.05)


    def receive_frames(self):
        self.setup_socket()
        print(f"Listening for frames on {self.multicast_group}:{self.port}...")

        while self._running:
            data, addr = self.sock.recvfrom(65536)
            if data:
                # Extract the feed ID (first byte)
                feed_id = data[0]
                np_data = np.frombuffer(data[1:], dtype=np.uint8)  # Remaining data is the frame

                # Deserialize frame
                frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
                if frame is not None:
                    # Update the frame for the feed ID
                    with self.lock:
                        self.frames[feed_id] = frame
            

    def start(self):
        # Start the receiving and displaying threads
        receive_thread = Thread(target=self.receive_frames, daemon=True)
        display_thread = Thread(target=self.display_in_grid)
        record_thread = Thread(target=self.record)

        receive_thread.start()
        display_thread.start()
        record_thread.start()

        self.root.mainloop()
        self._running = False
        self.sock.close()
        display_thread.join()
        record_thread.join()

if __name__ == "__main__":
    server = MulticastServer("224.0.0.1", 4201)
    server.start()
