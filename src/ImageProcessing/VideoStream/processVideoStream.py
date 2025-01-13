if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.ImageProcessing.VideoStream.threads.threadVideoStream import threadVideoStream
from src.ImageProcessing.VideoStream.VideoGridStreamer import VideoGridStreamer

class processVideoStream(WorkerProcess):
    """This process handles VideoStream.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.streamer = VideoGridStreamer(grid_rows=1, grid_cols=2, width=1024, height=540)
        super(processVideoStream, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processVideoStream, self).run()
        self.streamer.run(host='0.0.0.0', port=5000)

    def _init_threads(self):
        """Create the VideoStream Publisher thread and add to the list of threads."""
        VideoStreamTh = threadVideoStream(
            self.queuesList, self.streamer, self.logging, self.debugging
        )
        self.threads.append(VideoStreamTh)
