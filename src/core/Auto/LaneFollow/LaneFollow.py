import cv2
import numpy as np
import math
import time
from src.ImageProcessing.LaneDetect.pid_controller import PIDController

class LaneFollower:
    def __init__(self, width: int, height: int, logging, debugging=False, pc=False):
        self.debugging = debugging
        self.logging = logging
        self.width = width
        self.height = height
        self.pc = pc
        self.last_frame_time = time.time()
        self.lastStatus = 0

        # Measurement point - height in the image where we calculate lane center
        self.measure_height = int(self.height * 0.8)  # 80% down the image

        # PID controller for steering
        self.pid = PIDController(kp=0.5, ki=0.01, kd=0, output_limits=(-25, 25))

        # Image center reference point
        self.center_x = self.width // 2

    def restartPid(self):
        """Restart PID controller"""
        self.pid.reset()
        self.last_frame_time = time.time()
    def process_following(self, left_x: int | None, right_x: int | None):
        """Calculates lane center, error, steering angle. Does NOT draw on the frame."""
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_frame_time
        self.last_frame_time = current_time

        print(f" LAst frame time: {dt}")

        # Calculate lane center
        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) // 2
            lane_width_estimate = right_x - left_x
        elif left_x is not None:
            # Estimate based on typical lane width (adjust 200px if needed)
            lane_width_estimate = 200
            lane_center = left_x + lane_width_estimate // 2
        elif right_x is not None:
            # Estimate based on typical lane width (adjust 200px if needed)
            lane_width_estimate = 200
            lane_center = right_x - lane_width_estimate // 2
        else:
            lane_center = self.center_x  # Default to center if no lanes detected
            lane_width_estimate = 200 # Default estimate

        # Calculate error (offset from center)
        error = self.center_x - lane_center

        if abs(error) > 50:
            if(self.lastStatus != 0):
                self.pid.set_tunings(kp=0.5, ki=0.01, kd=0)
                self.lastStatus = 0
        else:   
            if(self.lastStatus != 1):
                self.pid.set_tunings(kp=0.05, ki=0.01, kd=0)
                self.lastStatus = 1
        # Use PID to calculate steering angle
        angle_degrees = self.pid.compute(error, dt=dt)

        # Debug log for terminal
        if self.logging:
            print(f"Lane Following: LeftX={left_x}, RightX={right_x}, "
                  f"Center={lane_center}, Error={error:.1f}px, Angle={angle_degrees:.1f}°")

        # Return calculation data only
        lane_data = {
            "left_detected": left_x is not None,
            "right_detected": right_x is not None,
            "lane_center": lane_center,
            "error": error,
            "left_x": left_x,
            "right_x": right_x,
            "lane_width_estimate": lane_width_estimate
        }

        self.finalAngle = -int(angle_degrees*10)
        print (f"Ugao je {self.finalAngle}")
        return self.finalAngle

