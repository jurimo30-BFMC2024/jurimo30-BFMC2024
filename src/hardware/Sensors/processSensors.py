if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.hardware.Sensors.threads.threadSensors import threadSensors

class processSensors(WorkerProcess):
    """This process handles Sensors.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processSensors, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processSensors, self).run()

    def _init_threads(self):
        """Create the Sensors Publisher thread and add to the list of threads."""
        SensorsTh = threadSensors(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(SensorsTh)
