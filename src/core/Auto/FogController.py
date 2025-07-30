import cv2
import numpy as np
import math
import time
from src.core.Auto.PIDController import PIDController

class FogController:
    """
    Controller for handling vehicle behavior in foggy conditions.
    Implements conservative driving strategies with enhanced safety measures.
    """
    
    def __init__(self, width: int, height: int, logging, debugging=False):
        self.debugging = debugging
        self.logging = logging
        self.width = width
        self.height = height
        
        # Fog-specific parameters
        self.fog_speed_reduction_factor = 0.6  # Reduce speed to 60% of normal
        self.min_fog_speed = 80  # Minimum speed in fog conditions
        self.max_fog_speed = 200  # Maximum speed in fog conditions
        self.fog_safety_distance_multiplier = 1.5  # Increase safety distance by 50%
        
        # Enhanced PID controller for fog conditions (more conservative)
        self.pid = PIDController(kp=0.25, ki=0.005, kd=0.02, kaw=2, output_limits=(-20, 20))
        
        # Reference point in image (slightly more centered for safety)
        self.center_x = self.width * 0.5  # Use true center in fog
        self.measure_height = int(self.height * 0.75)  # Look closer to vehicle
        
        # Fog nodes - should match the ones defined in autoFSM
        self.fog_nodes = {
            "114", "115", "116", "117", "118", "119", 
            "122", "123", "124", "125", "126", "127", "128"
        }
        
        # State tracking
        self.fog_active = False
        self.last_frame_time = time.time()
        self.consecutive_poor_visibility_frames = 0
        self.poor_visibility_threshold = 5  # frames
        
        # Enhanced lane detection for fog
        self.lane_confidence_threshold = 0.3  # Lower threshold for fog conditions
        self.emergency_stop_distance_fog = 50  # cm - increased safety distance
        
        if self.debugging:
            self.logging.info("FogController initialized with conservative parameters")
    
    def is_in_fog_zone(self, current_node: str):
        """Check if vehicle is in a fog zone"""
        return current_node in self.fog_nodes
    
    def assess_visibility(self, image):
        """
        Assess visibility conditions from camera image.
        Returns visibility score (0.0 = poor, 1.0 = good)
        """
        if image is None:
            return 0.0
        
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # Calculate contrast and variance as visibility indicators
        contrast = gray.std()
        mean_brightness = gray.mean()
        
        # Normalize contrast (typical range 0-100, but adjust threshold)
        contrast_score = min(contrast / 60.0, 1.0)  # More sensitive threshold
        
        # Penalize very low or very high brightness
        if mean_brightness < 30 or mean_brightness > 200:
            brightness_score = 0.5
        else:
            brightness_score = 1.0
        
        visibility_score = (contrast_score * 0.7 + brightness_score * 0.3)
        
        if self.debugging:
            print(f"Fog visibility assessment: contrast={contrast:.1f}, brightness={mean_brightness:.1f}, score={visibility_score:.2f}")
        
        return visibility_score
    
    def calculate_fog_speed(self, normal_speed: int, front_distance: float, visibility_score: float):
        """
        Calculate appropriate speed for fog conditions.
        """
        # Base fog speed reduction
        fog_speed = normal_speed * self.fog_speed_reduction_factor
        
        # Further reduce based on poor visibility
        if visibility_score < 0.4:
            fog_speed *= 0.7  # Very poor visibility
        elif visibility_score < 0.6:
            fog_speed *= 0.85  # Moderate visibility
        
        # Adjust for front distance (more conservative in fog)
        safe_distance = self.emergency_stop_distance_fog * self.fog_safety_distance_multiplier
        if front_distance < safe_distance:
            distance_factor = max(front_distance / safe_distance, 0.1)
            fog_speed *= distance_factor
        
        # Apply fog speed limits
        fog_speed = max(self.min_fog_speed, min(fog_speed, self.max_fog_speed))
        
        if self.debugging:
            print(f"Fog speed calculation: normal={normal_speed}, fog={fog_speed:.0f}, visibility={visibility_score:.2f}, distance={front_distance:.1f}")
        
        return int(fog_speed)
    
    def calculate_fog_steering(self, left_x: int | None, right_x: int | None, 
                              left_visible: bool, right_visible: bool, visibility_score: float):
        """
        Calculate steering angle for fog conditions with enhanced safety.
        """
        # Use more conservative lane following in fog
        if left_visible and right_visible and left_x is not None and right_x is not None:
            # Both lines visible - calculate center
            lane_center = (left_x + right_x) / 2
            error = self.center_x - lane_center
            
            # Reduce sensitivity in fog conditions
            if visibility_score < 0.5:
                error *= 0.8  # Reduce steering sensitivity
                
        elif right_visible and right_x is not None:
            # Only right line visible - stay at safe distance from it
            safe_distance_from_right = 120  # pixels - increased safety margin
            target_x = right_x - safe_distance_from_right
            error = self.center_x - target_x
            
        elif left_visible and left_x is not None:
            # Only left line visible - stay at safe distance from it
            safe_distance_from_left = 120  # pixels - increased safety margin
            target_x = left_x + safe_distance_from_left
            error = self.center_x - target_x
            
        else:
            # No lines visible - go straight with minimal adjustment
            error = 0
            if self.debugging:
                print("Fog control: No lanes visible, maintaining straight course")
            
            # Don't apply PID when no lanes are visible, just return straight
            return 0
        
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_frame_time
        self.last_frame_time = current_time
        
        # Apply PID control
        if dt > 0:
            angle = self.pid.compute(error, dt)
        else:
            angle = 0
        
        if self.debugging:
            print(f"Fog steering: error={error:.1f}, angle={angle:.1f}, visibility={visibility_score:.2f}")
        
        return -int(angle * 10)  # Convert to motor units
    
    def process_fog_control(self, left_x: int | None, right_x: int | None, 
                           left_visible: bool, right_visible: bool, current_node: str,
                           normal_speed: int, front_distance: float, image=None):
        """
        Main processing function for fog control.
        Returns (steering_angle, speed) tuple.
        """
        if not self.is_in_fog_zone(current_node):
            self.fog_active = False
            return None, None
        
        if not self.fog_active:
            self.fog_active = True
            self.pid.reset()
            if self.debugging:
                print("Fog control activated")
        
        # Assess visibility conditions
        visibility_score = 1.0  # Default to good visibility if no image
        if image is not None:
            visibility_score = self.assess_visibility(image)
        
        # Track poor visibility conditions
        if visibility_score < 0.4:
            self.consecutive_poor_visibility_frames += 1
        else:
            self.consecutive_poor_visibility_frames = 0
        
        # Calculate fog-appropriate steering
        steering_angle = self.calculate_fog_steering(
            left_x, right_x, left_visible, right_visible, visibility_score
        )
        
        # Calculate fog-appropriate speed
        fog_speed = self.calculate_fog_speed(normal_speed, front_distance, visibility_score)
        
        # Emergency procedures for prolonged poor visibility
        if self.consecutive_poor_visibility_frames > self.poor_visibility_threshold:
            fog_speed = min(fog_speed, self.min_fog_speed)
            if self.debugging:
                print("Extended poor visibility detected - further reducing speed")
        
        if self.debugging:
            print(f"Fog control output: angle={steering_angle}, speed={fog_speed}")
        
        return steering_angle, fog_speed
    
    def is_active(self):
        """Check if fog control is currently active"""
        return self.fog_active
    
    def reset(self):
        """Reset fog controller to initial state"""
        self.fog_active = False
        self.pid.reset()
        self.consecutive_poor_visibility_frames = 0
        self.last_frame_time = time.time()
        if self.debugging:
            print("Fog controller reset")
    
    def get_safety_distance_multiplier(self):
        """Return the safety distance multiplier for fog conditions"""
        return self.fog_safety_distance_multiplier
    
    def update_fog_parameters(self, speed_reduction=None, safety_multiplier=None):
        """Allow dynamic adjustment of fog parameters"""
        if speed_reduction is not None:
            self.fog_speed_reduction_factor = speed_reduction
        if safety_multiplier is not None:
            self.fog_safety_distance_multiplier = safety_multiplier
        
        if self.debugging:
            print(f"Fog parameters updated: speed_factor={self.fog_speed_reduction_factor}, safety_mult={self.fog_safety_distance_multiplier}")