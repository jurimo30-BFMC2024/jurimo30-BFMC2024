"""
PriorityQueueHandler Module

This module implements a priority-based message queue handler using Python's `threading` 
and `multiprocessing` libraries. The `PriorityQueueHandler` class manages multiple 
queues assigned to different priorities (e.g., "Critical", "Warning", "General", "Config") 
and processes messages from these queues in priority order.

Features:
- Separate worker threads for each priority queue.
- Internal priority-based message queueing for thread-safe access.
- Blocking `get` method to fetch messages based on their priority.

Usage:
- Initialize the `PriorityQueueHandler` with a dictionary of `Queue` objects for each priority.
- Add messages to the respective priority queues.
- Retrieve messages using the `get` method in priority order.

Example:
    queue_list = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }
    handler = PriorityQueueHandler(queue_list)

    # Add messages to queues
    queue_list["Critical"].put("Critical Message 1")

    # Get and process messages
    priority, message = handler.get()
    print(f"Priority: {priority}, Message: {message}")
"""

from threading import Semaphore, Thread
from multiprocessing import Queue
import queue
import time

class PriorityQueueHandler:
    """
    A priority-based message queue handler that processes messages from multiple queues in priority order.
    """

    def __init__(self, queue_list):
        """
        Initialize the PriorityQueueHandler with the given queues and start worker threads.

        Args:
            queue_list (dict): Dictionary of multiprocessing.Queue instances for each priority.
        """
        self.queue_list = queue_list
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

        # Start worker threads for each priority queue
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
                # Block until a message is available in the external queue
                message = self.queue_list[priority].get()
                # Calculate the priority level and enqueue the message
                level = self.priority_order[priority]
                self.counter += 1  # Increment counter for unique tuples
                try:
                    self.priority_queue.put((level, self.counter, priority, message), False)
                except Exception as e:
                    print(f"Exception in _queue_worker: {e}")
                self.message_semaphore.release()  # Signal new message
        except :
            pass

    def get(self):
        """
        Retrieve the highest priority message available.

        Blocks until a message is available. Messages are returned in priority order.

        Returns:
            tuple: (priority_name, message) where priority_name is the string name of the priority level.
        """
        self.message_semaphore.acquire()  # Block until a message is available
        while True:
            try:
                _, _, priority, message = self.priority_queue.get(True, 0.5)
                return priority, message
            except Exception as e:
                print(f"Exception[PQH_get]: {e}")
                # pass