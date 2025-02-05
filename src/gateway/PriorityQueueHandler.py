from threading import Semaphore, Thread, Lock
import queue
import time
from collections import deque, defaultdict

class PriorityQueueHandler:
    """
    A priority-based message queue handler that processes messages from multiple queues in priority order.
    """

    def __init__(self, queue_list, logger, debugging):
        """
        Initialize the PriorityQueueHandler with the given queues and start worker threads.

        Args:
            queue_list (dict): Dictionary of multiprocessing.Queue instances for each priority.
            logger: Logger instance for logging errors and debug information.
            debugging (bool): Whether to enable debugging statistics.
        """
        self.queue_list = queue_list
        self.logger = logger
        self.debugging = debugging
        self.message_semaphore = Semaphore(0)
        self.priority_queue = queue.PriorityQueue()
        
        # Define priority levels based on the original check order
        self.priority_order = {
            "Config": 0,
            "Critical": 1,
            "Warning": 2,
            "General": 3,
        }

        # Counter to ensure unique tuples for the priority queue
        self.counter = 0

        if self.debugging:
            # Statistics for processing times
            self.process_times = deque(maxlen=1000)
            self.total_process_time = 0

            # Debugging statistics
            self.message_counts = defaultdict(int)  # Track frequency of each message
            self.most_processing_time_message = None  # Track message with the highest processing time
            self.max_processing_time = 0  # Track the highest processing time
            self.messages_around_max_time = []  # Track messages around the local max processing time
            self.last_print_time = 0

        self.threads = []
        for priority in self.queue_list:
            thread = Thread(target=self._queue_worker, args=(priority,), daemon=True)
            thread.start()
            self.threads.append(thread)

    def _queue_worker(self, priority):
        """
        Worker thread that monitors a specific priority queue and forwards messages to the priority queue.

        Args:
            priority (str): The priority level this worker is responsible for.
        """
        try:
            while True:
                message = self.queue_list[priority].get()
                level = self.priority_order[priority]
                self.counter += 1  
                try:
                    self.priority_queue.put((level, self.counter, priority, message, time.time()), False)
                    self.message_semaphore.release()
                except Exception as e:
                    self.logger.error(f"Exception in _queue_worker: {e}")
        except:
            pass

    def get(self):
        """
        Retrieve the highest priority message available.

        Blocks until a message is available. Messages are returned in priority order.

        Returns:
            tuple: (priority_name, message) where priority_name is the string name of the priority level.
        """
        self.message_semaphore.acquire()  # Block until a message is available
        try:
            _, _, priority, message, rcv_t = self.priority_queue.get()

            if self.debugging:
                process_time = (time.time() - rcv_t) * 1000
                self.process_times.append(process_time)

                # Update message frequency
                message_key = (message["Owner"], message["msgID"])
                self.message_counts[message_key] += 1

                # Update message with the most processing time
                if process_time > self.max_processing_time:
                    self.max_processing_time = process_time
                    self.most_processing_time_message = message

                # Track messages around the local max processing time
                if len(self.process_times) >= 3:
                    local_max_index = self.process_times.index(max(self.process_times))
                    if local_max_index > 0 and local_max_index < len(self.process_times) - 1:
                        self.messages_around_max_time = [
                            self.process_times[local_max_index - 1],
                            self.process_times[local_max_index],
                            self.process_times[local_max_index + 1]
                        ]

                if time.time() - self.last_print_time > 1:
                    print(self.get_debugging_statistics())
                    self.last_print_time = time.time()

            return priority, message
        except Exception as e:
            self.logger.error(f"Exception[PQH_get]: {e}")

    def get_debugging_statistics(self):
        """
        Retrieve debugging statistics.

        Returns:
            dict: A dictionary containing debugging statistics.
        """
        most_sent_message = max(self.message_counts, key=self.message_counts.get)
        return {
            "most_sent_message": {
                "message": most_sent_message,
                "count": self.message_counts[most_sent_message]
            },
            "most_processing_time_message": {
                "message": self.most_processing_time_message,
                "processing_time": self.max_processing_time
            },
            "messages_around_max_time": self.messages_around_max_time
        }