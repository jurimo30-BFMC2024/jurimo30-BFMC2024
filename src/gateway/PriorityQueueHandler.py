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
from collections import deque

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

        # Statistika vremena obrade
        self.process_times = deque(maxlen=1000)
        self.max_process_time = 0
        self.total_process_time = 0

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
                except Exception as e:
                    print(f"Exception in _queue_worker: {e}")
                self.message_semaphore.release()
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
        while True:
            try:
                _, _, priority, message, rcv_t = self.priority_queue.get(True, 0.5)
                process_time = (time.time() - rcv_t) * 1000
                self.process_times.append(process_time)

                # Ažuriranje statistike
                self.max_process_time = max(self.max_process_time, process_time)

                # Lokalni prosjek samo zadnjih 1000 vrijednosti
                local_avg = sum(self.process_times) / len(self.process_times)
                local_min = min(self.process_times)
                local_max = max(self.process_times)
                
                # print(f"Process time: {process_time:.2f}ms | Max: {self.max_process_time:.2f}ms | Local Avg: {local_avg:.2f}ms | Local Min: {local_min:.2f}ms | Local Max: {local_max:.2f}ms")

                return priority, message
            except Exception as e:
                print(f"Exception[PQH_get]: {e}")
