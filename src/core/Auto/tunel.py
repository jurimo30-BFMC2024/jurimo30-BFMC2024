import cv2
import numpy as np
import math
import time
from src.core.Auto.PIDController import PIDController

class TunnelControl:
    def __init__(self, width: int, height: int, logging, debugging=False):
        self.debugging = debugging
        self.logging = logging
        self.width = width
        self.height = height
        
        # Rastojanje od desne linije koje treba održavati u tunelu
        self.target_distance_right = 100   # pikseli za desnu liniju
        
        # PID kontroler za praćenje desne linije (parametri iz SpecialSituationControl)
        self.pid = PIDController(kp=0.35, ki=0.01, kd=0, kaw=0, output_limits=(-25, 25))
        
        # Referentna tačka u slici
        self.center_x = self.width * 0.47
        self.measure_height = int(self.height * 0.8)
        
        # Čvorovi tunela - definiši na osnovu mape
        self.tunnel_nodes = {
            # Dodaj specifične nodove gde vozilo prolazi kroz tunel
            # Primer:
            "166", "167", "168", "169", "170", "171",
            # Dodaj ostale čvorove tunela...
        }
        
        # Stanje modula
        self.tunnel_active = False
        self.last_frame_time = time.time()
        
    def is_in_tunnel_zone(self, current_node: str):
        """Proverava da li je vozilo u zoni tunela"""
        return current_node in self.tunnel_nodes
        
    def calculate_right_line_error(self, right_x: int):
        """Izračunava grešku za praćenje desne linije u tunelu"""
        # Za desnu liniju, želimo da budemo na target_distance_right levo od nje
        target_x = right_x - self.target_distance_right
        error = self.center_x - target_x
        return error
        
    def process_tunnel_control(self, left_x: int | None, right_x: int | None, 
                              left_visible: bool, right_visible: bool, current_node: str):
        """
        Glavna metoda za obradu kontrole u tunelu
        
        Args:
            left_x: X koordinata leve linije
            right_x: X koordinata desne linije  
            left_visible: Da li je leva linija vidljiva
            right_visible: Da li je desna linija vidljiva
            current_node: Trenutni čvor na mapi
            
        Returns:
            tuple: (angle, speed) ili (None, None) ako modul nije aktivan
        """
        current_time = time.time()
        dt = current_time - self.last_frame_time
        self.last_frame_time = current_time
        
        # Proverava da li je vozilo u zoni tunela
        in_tunnel = self.is_in_tunnel_zone(current_node)
        
        # Ažurira stanje tunela
        if in_tunnel and not self.tunnel_active:
            # Vozilo prvi put ulazi u tunel
            self.tunnel_active = True
            self.pid.reset()
            if self.debugging:
                print(f"Entering tunnel at node {current_node}")
        elif not in_tunnel and self.tunnel_active:
            # Vozilo izlazi iz tunela
            if self.debugging:
                print(f"Exiting tunnel at node {current_node}")
            self.tunnel_active = False
            self.pid.reset()
        
        if self.tunnel_active:
            return self._handle_tunnel_control(right_x, right_visible, dt)
        else:
            return None, None
    
    def _handle_tunnel_control(self, right_x, right_visible, dt):
        """Upravljanje u tunelu - prati samo desnu liniju"""
        speed = 200  # Fiksna brzina u tunelu
        
        if right_visible and right_x is not None:
            error = self.calculate_right_line_error(right_x)
            angle = self.pid.compute(error, dt=dt)
            
            if self.debugging:
                print(f"Tunnel control: RightX={right_x}, Error={error:.1f}px, Angle={angle:.1f}°")
        else:
            # Ako desna linija nije vidljiva, ide pravo
            angle = 0
            if self.debugging:
                print("Tunnel control: Right line not visible, going straight")
        
        return -int(angle * 10), speed
    
    def is_active(self):
        """Proverava da li je modul trenutno aktivan"""
        return self.tunnel_active
        
    def reset(self):
        """Reset modula u početno stanje"""
        self.tunnel_active = False
        self.pid.reset()
        self.last_frame_time = time.time()
        if self.debugging:
            print("Tunnel module reset")
