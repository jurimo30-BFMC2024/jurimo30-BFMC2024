import time
from src.core.Auto.PIDController import PIDController

class TunnelController:
    def __init__(self, target_distance=18.0, logging=None, debugging=False, log_to_file=False):
        self.debugging = debugging
        self.logging = logging
        self.log_to_file = log_to_file
        self.last_frame_time = time.time()
        self.target_distance = target_distance  # Desired distance from right wall in cm
        
        # PID controller for maintaining wall distance
        self.pid = PIDController(kp=0.4, ki=0.01, kd=0.05, kaw=3, output_limits=(-25, 25))
        
        # Restart mechanism
        self.restartNeeded = False
        
        # Open log file for recording control data only if logging is enabled
        if self.log_to_file:
            self.log_file = open("tunnel_control_log.txt", "w")
            self.log_file.write("timestamp module message\n")

    def __del__(self):
        """Close log file when object is destroyed"""
        if hasattr(self, 'log_file') and self.log_to_file:
            self.log_file.close()

    def restartPid(self):
        """Restart PID controller"""
        self.pid.reset()
        self.last_frame_time = time.time()

    def set_target_distance(self, distance: float):
        """Set new target distance from wall"""
        self.target_distance = distance

    def process_tunnel_control(self, distance_from_right_wall: float):
        """Calculate steering angle to maintain fixed distance from right wall"""
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_frame_time
        self.last_frame_time = current_time

        # Calculate error (positive = too far from wall, negative = too close)
        error = distance_from_right_wall - self.target_distance

        # Compute PID output
        angle_degrees = self.pid.compute(error, dt=dt)

        # Restart mechanism for large corrections
        if abs(angle_degrees) > 24:
            self.restartNeeded = True

        if self.restartNeeded and abs(angle_degrees) < 3:
            self.restartNeeded = False
            self.restartPid()

        # Log data to file
        if self.log_to_file:
            timestamp = time.time()
            message = f"dt={dt:.6f} error={error:.2f} pid_output={-angle_degrees:.2f} distance={distance_from_right_wall:.2f}"
            self.log_file.write(f"{timestamp:.6f} TUNNEL {message}\n")
            self.log_file.flush()

        # Debug output
        if self.debugging:
            print(f"Tunnel Control: Distance={distance_from_right_wall:.1f}cm, "
                  f"Target={self.target_distance:.1f}cm, Error={error:.1f}cm, "
                  f"Angle={angle_degrees:.1f}°")

        # Return steering angle (converted to integer scaled by 10)
        self.finalAngle = -int(angle_degrees * 10)
        return self.finalAngle

    # Legacy method for backward compatibility
    def getControlData(self, distance_from_right_wall):
        """Legacy method - use process_tunnel_control instead"""
        return self.process_tunnel_control(distance_from_right_wall)