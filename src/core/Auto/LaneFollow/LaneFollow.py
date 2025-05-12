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
        self.pid = PIDController(kp=0.35, ki=0.01, kd=0.01, output_limits=(-25, 25))

        # Image center reference point
        self.center_x = self.width * 0.47
        
        # Otvori fajl za upisivanje podataka o obradi
        self.log_file = open("lane_follow_log.txt", "w")
        self.log_file.write("dt greska izlaz_pid\n")

    def __del__(self):
        """Zatvori log fajl prilikom uništenja objekta"""
        if hasattr(self, 'log_file'):
            self.log_file.close()

    def restartPid(self):
        """Restart PID controller"""
        self.pid.reset()
        self.last_frame_time = time.time()

    def set_pid_highway(self, highway: bool):
       if highway:
           kp = 0.25
           ki = 0.0
           kd = 0.1
       else:
           kp = 0.35
           ki = 0.01
           kd = 0.01
       self.pid.set_tunings(kp=kp, ki=ki, kd=kd)
       self.pid.reset()

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
        print(f"Error: {error}")

        angle_degrees = self.pid.compute(error, dt=dt)
        
        # Upisivanje podataka u fajl
        self.log_file.write(f"{dt:.6f} {error} {-angle_degrees}\n")
        self.log_file.flush()  # Osigurava da se podaci odmah upišu

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

