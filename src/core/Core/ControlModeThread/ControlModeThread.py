import threading

class ControlModeThread:
    def __init__(self):
        self._thread = None
        self._running = threading.Event()

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        
        self._running.set()
        self._thread = threading.Thread(target=self.run, daemon=True)
        self._thread.start()

    def stop(self):
        if self._thread is None or not self._thread.is_alive():
            print("Thread is not running.")
            return
        
        self._running.clear()
        self._thread.join()
        self._thread = None  # Reset the thread object

    def run(self):
        """This method should be overridden by subclasses."""
        raise NotImplementedError