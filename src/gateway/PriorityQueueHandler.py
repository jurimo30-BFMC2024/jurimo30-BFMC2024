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

class PriorityQueueHandler:
    def __init__(self, queue_list):
        self.queue_list = queue_list
        self.message_semaphore = Semaphore(0)
        
        self.internal_queue = {
            "Critical": Queue(),
            "Warning": Queue(),
            "General": Queue(),
            "Config": Queue(),
        }

        # Create and start threads for each queue
        self.threads = []
        for priority in self.queue_list:
            thread = Thread(target=self._queue_worker, args=(priority,), daemon=True)
            thread.start()
            self.threads.append(thread)

    def _queue_worker(self, priority):
        try:
            while True:
                message = self.queue_list[priority].get()  # Block until a message is available
                self.internal_queue[priority].put(message)  # Add the message to the internal queue
                self.message_semaphore.release()  # Notify get method that a message is available
        except:
            pass
            
    def get(self):
        self.message_semaphore.acquire()  # Block until a message is available
        while True:
            # Check each queue in priority order
            for priority in ["Config", "Critical", "Warning", "General"]:
                if not self.internal_queue[priority].empty():
                    return priority, self.internal_queue[priority].get()
