if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.simulator.SimCom.threads.threadSimCom import threadSimCom

class processSimCom(WorkerProcess):
    """This process handles SimCom.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processSimCom, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processSimCom, self).run()

    def _init_threads(self):
        """Create the SimCom Publisher thread and add to the list of threads."""
        SimComTh = threadSimCom(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(SimComTh)
