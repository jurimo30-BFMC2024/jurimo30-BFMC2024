import numpy as np

class MovingAverage:
    def __init__(self, history_size):
        """
        Initialize the moving average filter.
        
        Parameters:
            history_size (int): The size of the history array for the moving average.
        """
        self.history_size = history_size
        self.history = np.zeros(history_size)
        self.index = 0
        self.count = 0

    def add(self, value):
        """
        Add a new value to the filter's history.

        Parameters:
            value (float): The new value to add.
        """
        self.history[self.index] = value
        self.index = (self.index + 1) % self.history_size
        self.count = min(self.count + 1, self.history_size)

    def get_average(self):
        """
        Compute the moving average of the values in the history.

        Returns:
            float: The moving average.
        """
        if self.count == 0:
            return 0  # Avoid division by zero if no values are added yet
        return np.sum(self.history) / self.count

    def filter(self, value):
        """
        Add a new value and return the updated moving average.

        Parameters:
            value (float): The new value to add.

        Returns:
            float: The updated moving average.
        """
        self.add(value)
        return self.get_average()