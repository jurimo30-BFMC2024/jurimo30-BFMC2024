import cv2
import numpy as np
import math
import time
from src.core.Auto.PIDController import PIDController

class Highway:
    def __init__(self, width: int, height: int, logging, debugging=False):
        self.debugging = debugging
        self.logging = logging
        self.width = width
        self.height = height
        
        # Rastojanje od desne linije koje treba održavati na autoputu
        self.target_distance_right = 100   # pikseli za desnu liniju
        
        # Nodovi na kojima se prati samo desna linija
        self.right_line_only_nodes = {
            "398", "399", "400", "401", "444", "445", "446", "447",
            "421", "422", "423", "463", "464", "465", "466",
            "426", "425", "424", "462", "461", "460",
            "479", "480", "481", "482", "402", "403", "404"
            # Dodaj specifične nodove gde treba pratiti samo desnu liniju
        }
        
        # PID kontroler za praćenje desne linije (parametri iz SpecialSituationControl)
        self.pid = PIDController(kp=0.35, ki=0.01, kd=0, kaw=0, output_limits=(-25, 25))
        
        # Referentna tačka u slici
        self.center_x = self.width * 0.47
        self.measure_height = int(self.height * 0.8)
        
        # Stanje modula
        self.last_frame_time = time.time()
    
    def calculate_right_line_error(self, right_x: int):
        """Izračunava grešku za praćenje desne linije na autoputu"""
        # Za desnu liniju, želimo da budemo na target_distance_right levo od nje
        target_x = right_x - self.target_distance_right
        error = self.center_x - target_x
        return error
        
    def process_highway_control(self, left_x: int | None, right_x: int | None, 
                              left_visible: bool, right_visible: bool, current_node: str):

        if current_node in self.right_line_only_nodes:    
            current_time = time.time()
            dt = current_time - self.last_frame_time
            self.last_frame_time = current_time
            
            # Na autoputu pratimo samo desnu liniju
            if right_visible and right_x is not None:
                error = self.calculate_right_line_error(right_x)
                angle = self.pid.compute(error, dt=dt)
                
                if self.debugging:
                    print(f"Highway control: RightX={right_x}, Error={error:.1f}px, Angle={angle:.1f}°")
                    
            
            return -int(angle * 10)
        else:
            return None
    
    def reset(self):
        """Reset modula u početno stanje"""
        # self.highway_active = False
        self.pid.reset()
        self.last_frame_time = time.time()
        if self.debugging:
            print("Highway module reset")