import time

class MotionScheduler:
    def __init__(self):
        self.schedule = []  # List of (angle, speed, duration) tuples
        self.start_time = None
    
    def set_schedule(self, schedule):
        if not all(len(item) == 3 for item in schedule):
            raise ValueError("Each tuple must contain (angle, speed, duration)")
        
        self.schedule = schedule
        self.start_time = time.time()
    
    def run(self):
        if not self.schedule or self.start_time is None:
            return None, None, True  # No schedule set
        
        elapsed = time.time() - self.start_time
        total_time = 0
        
        for angle, speed, duration in self.schedule:
            total_time += duration
            if elapsed < total_time:
                return angle, speed, False
        
        return 0, 0, True  # Return last state and finished flag

# Example Usage:
if __name__ == "__main__":
    scheduler = MotionScheduler()
    scheduler.set_schedule([(10, 5, 2), (20, 10, 3), (30, 15, 5)])
    while True:
        angle, speed, finished = scheduler.run()
        print(f"Angle: {angle}, Speed: {speed}, Finished: {finished}")
        if finished:
            break
        time.sleep(0.5)
