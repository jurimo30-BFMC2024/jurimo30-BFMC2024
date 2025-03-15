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
        self.streamer = VideoGridStreamer(grid_rows=2, grid_cols=1)
        self.streamer.start(host='0.0.0.0', port=4201)
        super(processVideoStream, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processVideoStream, self).run()

    def _init_threads(self):
        """Create the VideoStream Publisher thread and add to the list of threads."""
        VideoStreamTh = threadVideoStream(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(VideoStreamTh)
