import threading

class ControlModeThread:
    def __init__(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._running = threading.Event()  # Controls whether the thread continues running.
        self._suspend_cond = threading.Condition()  # For handling suspension and resumption.
        self._is_suspended = True  # Indicates if the thread is currently suspended.
        self._terminate = False  # Indicates if the thread should terminate.
        self._thread.start()

    def __del__(self):
        """Signal the thread to terminate and block until it ends."""
        self._terminate = True
        self.start()  # Ensure the thread isn't suspended
        self._thread.join()

    def start(self):
        """Resume the thread if it's suspended."""
        with self._suspend_cond:
            if not self._is_suspended:
                return  # Already running
            self._is_suspended = False
            self._running.set()  # Allow the thread to proceed
            self._suspend_cond.notify()

    def stop(self):
        """Suspend the thread and block until it is suspended."""
        with self._suspend_cond:
            if self._is_suspended:
                return  # Already suspended
            self._running.clear()  # Prevent further processing in the thread
            self._suspend_cond.wait_for(lambda: self._is_suspended)  # Wait until thread suspends

    def _run(self):
        """Main loop of the thread."""
        while not self._terminate:
            with self._suspend_cond:
                while self._is_suspended and not self._terminate:
                    self._suspend_cond.wait()  # Wait for a start signal
                if self._terminate:
                    break
                self._is_suspended = False  # Mark thread as running

            try:
                self.loop()  # Call the user-defined method
            except Exception as e:
                print(f"Error in thread: {e}")
            
            # Ensure the thread suspends when `_running` is cleared
            with self._suspend_cond:
                if not self._running.is_set():
                    self._is_suspended = True
                    self._suspend_cond.notify_all()  # Notify stop() that the thread is suspended

    def loop(self):
        """This method should be overridden by subclasses."""
        raise NotImplementedError
