import threading
import collections

class DequePipeSync:
    """
    A class that wraps a collections.deque and a Pipe connection.
    It provides an append method to add items, and runs a background
    thread which listens on the pipe for a boolean signal. The boolean
    indicates if a blocking pop should occur (wait for data if necessary)
    or a non-blocking pop (return None if no data). The popped message is
    sent back on the same pipe.
    """
    def __init__(self, max_elements, pipe_conn):
        """
        Constructor.

        Args:
            max_elements (int): Maximum number of elements for the deque.
            pipe_conn: A connection object (e.g., one end of a multiprocessing.Pipe)
                       used for receiving pop signals and sending back messages.
        """
        self.deque = collections.deque(maxlen=max_elements)
        self.pipe_conn = pipe_conn

        # Condition variable to synchronize access to the deque.
        self.cond = threading.Condition()

        # Start the listener thread as a daemon.
        self.thread = threading.Thread(target=self._pipe_listener, daemon=True)
        self.thread.start()

    def send(self, item):
        """
        Append an item to the deque.
        Notifies the waiting thread in case a blocking pop is pending.
        """
        with self.cond:
            self.deque.append(item)
            self.cond.notify()  # Notify one waiting thread

    def _pipe_listener(self):
        """
        Thread target function that listens for a boolean signal from the pipe.
        Depending on the signal, it pops an item from the deque:
          - True: wait until an item is available.
          - False: perform a non-blocking pop; if none, result is None.
        The result is then sent back through the pipe.
        """
        while True:
            try:
                request = self.pipe_conn.recv()
            except:
                # The pipe was closed. Exit the thread.
                break

            if request["mode"] == "recv":
                block_flag = request["block"]
            elif request["mode"] == "len":
                self.pipe_conn.send(len(self.deque))
                continue

            # Perform the pop operation with appropriate blocking behavior.
            with self.cond:
                if block_flag:
                    # Blocking pop: wait until at least one element is available.
                    while not self.deque:
                        self.cond.wait()
                    result = self.deque.popleft()
                else:
                    # Non-blocking pop: return an element if available, else None.
                    result = self.deque.popleft() if self.deque else None

            try:
                # Send the result back on the pipe.
                self.pipe_conn.send(result)
            except Exception as e:
                print(f"DequePipeSync Exception: {e}")
        
    def __del__(self):
        """
        Destructor for the DequePipeWrapper class.
        Ensures that the pipe connection is closed which causes the listener
        thread to exit its loop gracefully.
        """
        try:
            if self.pipe_conn:
                self.pipe_conn.close()
        except Exception:
            pass