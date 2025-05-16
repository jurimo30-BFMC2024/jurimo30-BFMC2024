import time
from typing import List, Tuple
import random
import math

class Localization:
    def __init__(self, segments_data):
        self.total_target_speed = 0.0
        self.num_speed_samples = 0
        self.start_time = time.time()
        self.last_update_time = self.start_time
        self.average_target_speed = 0.0
        self.speed_error = 0.0
        self.total_distance = 0.0  # Accumulated distance on current segment

        self.segments_data = segments_data
        self.current_segment = None

        scale_factor = 13  # e.g., each unit in graph

        for segment in segments_data:
            for node in segment['nodes']:
                x, y = node['pos']
                node['pos'] = (x * scale_factor, y * scale_factor)

            # Modify list in place
            segment['distances'] = [d * scale_factor for d in segment['distances']]

            segment['length'] *= scale_factor

            print(segment)


        # fields for position tracking
        self.location: Tuple[float, float] = (0.0, 0.0)
        self._cum_dists: List[float] = []  # cumulative distances along nodes

    def start_new_segment(self):
        """
        Begin a new segment:
         - Pop the next segment from the queue
         - Build cumulative distances list
         - Reset timers & distance
         - Set location to the first node's position
        """
        # get next segment
        self.current_segment = self.segments_data.pop(0)

        # build cumulative distances [0, d0, d0+d1, ...]
        dists = self.current_segment["distances"]
        self._cum_dists = [0.0]
        for d in dists:
            self._cum_dists.append(self._cum_dists[-1] + d)

        print(f"_cum_dists: {self._cum_dists}")

        # reset timers & distance
        self.start_time = time.time()
        self.last_update_time = self.start_time
        self.total_distance = 0.0

        # initial position = first node
        first_node = self.current_segment["nodes"][0]["pos"]
        self.location = first_node

        # reset speed error & averages if desired
        self.total_target_speed = 0.0
        self.num_speed_samples = 0
        self.average_target_speed = 0.0

    def update_position(self, target_speed: float):
        """
        Call this periodically with the current target speed.
         - Updates average target speed
         - Integrates distance traveled (corrected by speed_error)
         - Updates self.location accordingly
        """
        current_time = time.time()
        dt = current_time - self.last_update_time

        # update running average of target speed
        self.total_target_speed += target_speed
        self.num_speed_samples += 1
        self.average_target_speed = self.total_target_speed / self.num_speed_samples

        # estimate actual speed and integrate
        estimated_speed = max(0.0, target_speed - self.speed_error)
        self.total_distance += estimated_speed * dt

        # update position on current segment
        self._update_location()

        self.last_update_time = current_time

    def _update_location(self):
        """
        Internal: based on self.total_distance and self._cum_dists,
        find which two nodes we're between, and interpolate.
        """
        seg = self.current_segment
        nodes = seg["nodes"]
        total_len = seg["length"]

        # clamp to [0, total_len]
        d = min(max(self.total_distance, 0.0), total_len)
        print(f"d: {d}")

        # find index i so that cum[i] <= d < cum[i+1]
        # if at very end, snap to last node
        if d >= self._cum_dists[-1]:
            self.location = nodes[-1]["pos"]
            return

        # binary or linear search
        for i in range(len(self._cum_dists)-1):
            if self._cum_dists[i] <= d < self._cum_dists[i+1]:
                # compute interpolation factor along this segment
                seg_dist = self._cum_dists[i+1] - self._cum_dists[i]
                if seg_dist <= 0.0:
                    t = 0.0
                else:
                    t = (d - self._cum_dists[i]) / seg_dist

                x0, y0 = nodes[i]["pos"]
                x1, y1 = nodes[i+1]["pos"]
                # linear interp
                x = x0 + t * (x1 - x0)
                y = y0 + t * (y1 - y0)
                self.location = (x, y)
                return

    def update_speed_error(self):
        """
        Call at end of segment to adjust speed_error:
         - actual_speed = segment_length / time_taken
         - speed_error = average_target_speed - actual_speed
        """
        current_time = time.time()
        passed_time = current_time - self.start_time
        if passed_time > 0 and self.current_segment:
            actual_speed = self.current_segment["length"] / passed_time
            self.speed_error = self.average_target_speed - actual_speed
            print(f"average_target_speed: {self.average_target_speed}, actual_speed: {actual_speed}, segment_length: {self.current_segment['length']}, passed_t: {passed_time}, speed_error: {self.speed_error}")

    def get_location(self, noise_stddev=0.05):
        """
        Returns current estimated location with optional Gaussian noise.
        `noise_stddev` is the standard deviation of the noise in units.
        """
        return self.location
        # if self.location is None:
        #     return None
        # x, y = self.location
        # noisy_x = x + random.gauss(0, noise_stddev)
        # noisy_y = y + random.gauss(0, noise_stddev)
        # return (noisy_x, noisy_y)

    def start_relative_localization(self):
        """
        Save the last known position of the car for relative localization.
        """
        self.saved_position = self.location
        print(f"Relative localization started. Saved position: {self.saved_position}")

    def update_position_with_steering(self, speed: float, steering_angle_deg: float):
        """
        Update the car's position based on speed and steering angle.
        :param speed: Speed of the car (units per second).
        :param steering_angle_deg: Steering angle in degrees.
        """
        if self.location is None:
            print("Error: Current location is not set.")
            return

        # Measure time since the last update
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Convert steering angle to radians
        steering_angle_rad = math.radians(steering_angle_deg)

        # Calculate the change in position
        dx = speed * math.cos(steering_angle_rad) * dt
        dy = speed * math.sin(steering_angle_rad) * dt

        # Update the current location
        x, y = self.location
        self.location = (x + dx, y + dy)
        print(f"Updated position with speed {speed} and steering angle {steering_angle_deg}: {self.location}")

    def stop_relative_localization(self):
        """
        Reset the car's position to the last saved position.
        """
        if hasattr(self, 'saved_position'):
            self.location = self.saved_position
            print(f"Relative localization stopped. Reset position to: {self.location}")
        else:
            print("Error: No saved position to reset to.")

if __name__ == "__main__":
    segments_data = [
        {
            'nodes': [{'idx': '1', 'pos': (0, 0)}, {'idx': '2', 'pos': (10, 0)}],
            'distances': [10.0],
            'length': 10.0
        },
        {
            'nodes': [{'idx': '2', 'pos': (10, 0)}, {'idx': '3', 'pos': (10, 10)}, {'idx': '4', 'pos': (10, 20)}],
            'distances': [10.0, 10.0],
            'length': 20.0
        }
    ]

    loc = Localization(segments_data.copy())
    loc.start_new_segment()

    print("Initial position:", loc.get_location())

    # Simulate moving along the first segment at 2 units/sec
    for i in range(6):
        loc.update_position(25.0)
        print(f"Distance[est]: {loc.total_distance:.2f}, Distance[ctrl]: {loc.average_target_speed * (0.5 * i)}, Position: {loc.get_location()}")
        time.sleep(0.5)

    print("\nReached end of first segment.")
    loc.update_speed_error()
    print("Speed error:", loc.speed_error)

    loc.start_new_segment()
    print("\nStarted new segment")
    print("Initial position:", loc.get_location())

    # Simulate moving along second segment with same speed
    for i in range(10):
        loc.update_position(40.0)
        print(f"Distance[est]: {loc.total_distance:.2f}, Distance[ctrl]: {loc.average_target_speed * (0.5 * i)}, Position: {loc.get_location()}")
        # print(f"Distance: {loc.total_distance:.2f}, Position: {loc.get_location()}")
        time.sleep(0.5)

    print("\nReached end of second segment.")
    loc.update_speed_error()
    print("Speed error:", loc.speed_error)

    # Test relative localization
    loc.start_relative_localization()
    loc.update_position_with_steering(speed=50.0, steering_angle_deg=20)
    time.sleep(0.5)
    loc.update_position_with_steering(speed=50.0, steering_angle_deg=-20)
    time.sleep(0.5)
    loc.stop_relative_localization()
    print(f"Final position after stopping relative localization: {loc.get_location()}")

    print("\nDone.")
