import time
from typing import List, Tuple
import random
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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

        scale_factor = 100

        for segment in segments_data:
            for node in segment['nodes']:
                x, y = node['pos']
                node['pos'] = (x * scale_factor, y * scale_factor)

            # Modify list in place
            segment['distances'] = [d * scale_factor for d in segment['distances']]

            segment['length'] *= scale_factor

            # print(segment)

        # fields for position tracking
        self.location: Tuple[float, float] = (0.0, 0.0)
        self._cum_dists: List[float] = []  # cumulative distances along nodes

        self.heading_error = 0.0  # Difference between IMU and calculated heading
        self.orientation = 0.0  # Current calculated orientation in degrees

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

        # print(f"_cum_dists: {self._cum_dists}")

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
        current_node = self._update_location()

        self.last_update_time = current_time
        return current_node

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
        # print(f"d: {d}")

        # find index i so that cum[i] <= d < cum[i+1]
        # if at very end, snap to last node
        if d >= self._cum_dists[-1]:
            self.location = nodes[-1]["pos"]
            return nodes[-1]["idx"]

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
                return nodes[i]["idx"]
        return None

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
            # print(f"average_target_speed: {self.average_target_speed}, actual_speed: {actual_speed}, segment_length: {self.current_segment['length']}, passed_t: {passed_time}, speed_error: {self.speed_error}")

    def get_location(self):
        """
        Returns current estimated location with optional Gaussian noise.
        `noise_stddev` is the standard deviation of the noise in units.
        """
        return self.location

    def _calculate_path_heading(self):
        """
        Calculate expected heading based on current path segment
        Returns heading in degrees (0-360)
        """
        if not self.current_segment:
            return 0.0

        nodes = self.current_segment["nodes"]
        # Find current segment we're on
        for i in range(len(nodes) - 1):
            x0, y0 = nodes[i]["pos"]
            x1, y1 = nodes[i + 1]["pos"]

            # Check if we're between these nodes
            if self._cum_dists[i] <= self.total_distance < self._cum_dists[i + 1]:
                dx = x1 - x0
                dy = y1 - y0
                heading = math.degrees(math.atan2(dy, dx))
                return heading % 360

        # Handle edge case: if at or beyond the last node, use the last segment's heading
        if len(nodes) >= 2 and self.total_distance >= self._cum_dists[-1]:
            x0, y0 = nodes[-2]["pos"]
            x1, y1 = nodes[-1]["pos"]
            dx = x1 - x0
            dy = y1 - y0
            heading = math.degrees(math.atan2(dy, dx))
            return heading % 360

        return 0.0

    def calibrate_heading(self, imu_heading):
        """
        Update heading error based on difference between IMU and calculated heading
        :param imu_heading: Heading from IMU in degrees (0-360)
        """
        calculated_heading = self._calculate_path_heading()
        error = imu_heading - calculated_heading
        # Normalize error to [-180, 180]
        error = (error + 180) % 360 - 180
        # Update error with some smoothing
        self.heading_error = 0.95 * self.heading_error + 0.05 * error
        return self.heading_error

    def update_position_with_steering(self, speed: float, steering_angle_deg: float, orientation_deg: float):
        """
        Update the car's position based on speed, steering angle, and absolute orientation.
        Now accounts for heading calibration.
        """
        if self.location is None:
            raise ValueError("Current location is not set")

        # Measure time since the last update
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Apply heading correction
        corrected_orientation = orientation_deg - self.heading_error
        
        # Convert angles to radians
        steering_angle_rad = math.radians(steering_angle_deg)
        orientation_rad = math.radians(corrected_orientation)

        # Car parameters (adjust these based on your car's dimensions)
        wheelbase = 20.0  # distance between front and rear axles in units
        max_steering_angle = math.radians(25.0)  # maximum steering angle in radians
        
        # Limit steering angle
        steering_angle_rad = max(-max_steering_angle, min(max_steering_angle, steering_angle_rad))

        if abs(steering_angle_rad) < 0.001:
            # For straight motion, use simple linear update
            dx = speed * math.cos(orientation_rad) * dt
            dy = speed * math.sin(orientation_rad) * dt
        else:
            # Calculate turning radius using bicycle model
            turning_radius = wheelbase / math.tan(steering_angle_rad)
            
            # Calculate angular velocity (omega)
            angular_velocity = speed / turning_radius
            
            # Calculate change in orientation
            delta_orientation = angular_velocity * dt
            
            # Calculate position change
            dx = turning_radius * (math.sin(orientation_rad + delta_orientation) - math.sin(orientation_rad))
            dy = turning_radius * (math.cos(orientation_rad) - math.cos(orientation_rad + delta_orientation))

        # Update the current location
        x, y = self.location
        self.location = (x + dx, y + dy)

    def clamp_location_to_graph(self):
        """
        Clamp the current location to the nearest point on the graph vectors.
        :raises ValueError: If no current segment is set or if location is not set
        """
        if self.current_segment is None:
            raise ValueError("No current segment to clamp location")
            
        if self.location is None:
            raise ValueError("Current location is not set")

        nodes = self.current_segment["nodes"]
        closest_point = None
        min_distance = float('inf')

        for i in range(len(nodes) - 1):
            x0, y0 = nodes[i]["pos"]
            x1, y1 = nodes[i + 1]["pos"]

            # Project the current location onto the segment
            px, py = self.location
            dx, dy = x1 - x0, y1 - y0
            segment_length_squared = dx**2 + dy**2
            if segment_length_squared == 0:
                # Degenerate segment, use the start point
                projection = (x0, y0)
            else:
                t = max(0, min(1, ((px - x0) * dx + (py - y0) * dy) / segment_length_squared))
                projection = (x0 + t * dx, y0 + t * dy)

            # Calculate the distance to the projection
            distance = math.sqrt((px - projection[0])**2 + (py - projection[1])**2)
            if distance < min_distance:
                min_distance = distance
                closest_point = projection

        # Update the location to the closest point
        self.location = closest_point
        # print(f"Clamped location to: {self.location}")

if __name__ == "__main__":
    def plot_track_and_position(segments, positions=None):
        """Plot the track segments and optionally the car's positions"""
        plt.figure(figsize=(10, 10))
        
        # Plot track segments
        for segment in segments:
            nodes = segment['nodes']
            x_coords = [node['pos'][0] for node in nodes]
            y_coords = [node['pos'][1] for node in nodes]
            plt.plot(x_coords, y_coords, 'b-', linewidth=2, label='Track')
            
            # Plot nodes
            plt.plot(x_coords, y_coords, 'ko', markersize=8)
            
            # Add node labels
            for node in nodes:
                x, y = node['pos']
                plt.annotate(f"Node {node['idx']}", (x, y), 
                           xytext=(5, 5), textcoords='offset points')
        
        if positions:
            # Plot car positions
            x_pos = [p[0] for p in positions]
            y_pos = [p[1] for p in positions]
            plt.plot(x_pos, y_pos, 'r.--', linewidth=1, markersize=5, label='Car Path')
        
        plt.grid(True)
        plt.axis('equal')
        plt.title('Localization Test Track')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        if positions:
            plt.legend()
        plt.show()

    def test_initialization():
        print("\n=== Testing Initialization ===")
        # Simple test track: two segments forming an L shape
        test_segments = [
            {
                'nodes': [
                    {'idx': '1', 'pos': (0, 0)},
                    {'idx': '2', 'pos': (10, 0)}
                ],
                'distances': [10],
                'length': 10
            },
            {
                'nodes': [
                    {'idx': '2', 'pos': (10, 0)},
                    {'idx': '3', 'pos': (15, 5)}
                ],
                'distances': [10],
                'length': 10
            }
        ]
        
        # Plot initial track
        plot_track_and_position(test_segments)
        
        loc = Localization(test_segments.copy())
        print("Initialized with test segments")
        assert len(loc.segments_data) == 2, "Should have 2 segments"
        print("✓ Initialization test passed")
        return loc, test_segments

    def test_segment_navigation(loc):
        print("\n=== Testing Segment Navigation ===")
        positions = []  # Store positions for plotting
        
        loc.start_new_segment()
        initial_pos = loc.get_location()
        positions.append(initial_pos)
        print(f"Initial position: {initial_pos}")
        assert initial_pos == (0, 0), "Should start at origin"

        # Move along first segment
        target_speed = 5.0  # 5 units/sec
        while loc.total_distance < 10:
            loc.update_position(target_speed)
            positions.append(loc.get_location())
            time.sleep(0.1)
            
        print(f"End of segment position: {loc.get_location()}")
        print("✓ Segment navigation test passed")
        return positions

    def test_speed_error(loc):
        print("\n=== Testing Speed Error Calculation ===")
        loc.update_speed_error()
        print(f"Speed error after first segment: {loc.speed_error}")
        assert loc.speed_error != 0, "Speed error should be calculated"
        print("✓ Speed error calculation test passed")

    def test_steering(loc, positions):
        loc.start_new_segment()
        print("\n=== Testing Steering ===")
        initial_pos = loc.get_location()
        positions.append(initial_pos)
        print(f"Position before steering: {initial_pos}")
        
        # Test small turn
        for _ in range(5):  # Simulate several steering updates
            loc.update_position_with_steering(speed=5.0, steering_angle_deg=10, orientation_deg=10*_)
            positions.append(loc.get_location())
            time.sleep(0.1)
        
        pos_after_turn = loc.get_location()
        print(f"Position after steering: {pos_after_turn}")
        assert pos_after_turn != initial_pos, "Position should change after steering"
        
        # Test clamping to graph
        loc.clamp_location_to_graph()
        clamped_pos = loc.get_location()
        positions.append(clamped_pos)
        print(f"Position after clamping: {clamped_pos}")
        assert clamped_pos != pos_after_turn, "Position should be clamped to nearest point on graph"
        print("✓ Steering and clamping test passed")
        return positions

    def run_all_tests():
        try:
            loc, test_segments = test_initialization()
            positions = test_segment_navigation(loc)
            test_speed_error(loc)
            positions = test_steering(loc, positions)
            
            # Plot final results
            plot_track_and_position(test_segments, positions)
            
            print("\n=== All tests passed successfully! ===")
        except AssertionError as e:
            print(f"\n❌ Test failed: {e}")
        except Exception as e:
            print(f"\n❌ Unexpected error: {e}")

    run_all_tests()
