import cv2
import numpy as np
import math
import time
from src.core.Auto.PIDController import PIDController

class SpecialSituationControl:
    def __init__(self, width: int, height: int, logging, debugging=False):

        self.debugging = debugging
        self.logging = logging
        self.width = width
        self.height = height
        
        # Parametri za detekciju stanja jedne linije
        self.single_line_threshold_ms = 300  # Vreme u ms kada se smatra da je samo jedna linija vidljiva
        self.last_both_lines_time = time.time()
        self.single_line_active = False
        self.tracked_line_side = None  # 'left' ili 'right'
        
        # Rastojanje od linije koje treba održavati - odvojeno za levu i desnu
        self.target_distance_right = 100   # pikseli za desnu liniju
        self.target_distance_left = 123   # pikseli za levu liniju
        
        # PID kontroler za praćenje jedne linije (kopirani parametri iz LaneFollow)
        self.pid = PIDController(kp=0.35, ki=0.01, kd=0, kaw=0, output_limits=(-25, 25))
        
        # Referentna tačka u slici
        self.center_x = self.width * 0.47
        self.measure_height = int(self.height * 0.8)
        
        # Parametri za raskrsnice - definisani na osnovu mape
        # Ovi čvorovi predstavljaju zone gde vozilo treba specijalno upravljanje
        self.intersection_nodes = {
            # Primer za malu mapu - prilagoditi stvarnoj mapi
            'intersection_main': ['190', '191', '192', '193', '194', '223'],  # Glavni čvorovi raskrsnica
            # Dodajte ostale raskrsnice na osnovu graphml fajla...
        }
        
        self.current_intersection = None
        self.intersection_direction = None  # 'Left', 'Right', 'Straight'
        self.intersection_angle_degrees = 15  # Ugao pod kojim se kreće kad ne vidi potrebnu liniju
        
        # Stanje modula
        self.last_frame_time = time.time()
        self.current_intersection = None  # Trenutna aktivna raskrsnica
        
    def is_in_intersection_zone(self, current_node: str):
        """Proverava da li je vozilo u zoni raskrsnice"""
        for intersection_name, nodes in self.intersection_nodes.items():
            if current_node in nodes:
                return intersection_name
        return None
        
    def determine_tracking_side(self, direction: str):
        """Određuje koju liniju treba pratiti na osnovu smera"""
        if direction == "Right":
            return "right"
        elif direction == "Left":
            return "left"
        else:  # Straight
            return None  # Za pravo, pokušava da prati obe linije
            
    def calculate_single_line_error(self, line_x: int, side: str):
        """Izračunava grešku za praćenje jedne linije"""
        if side == "left":
            # Za levu liniju, želimo da budemo na target_distance_left desno od nje
            target_x = line_x + self.target_distance_left
        else:  # right
            # Za desnu liniju, želimo da budemo na target_distance_right levo od nje
            target_x = line_x - self.target_distance_right
            
        error = self.center_x - target_x
        return error
        
 # Provjeriti funkciju naknadno
    def process_special_control(self, left_x: int | None, right_x: int | None, 
                              left_visible: bool, right_visible: bool, current_node: str, navigate_command_getter_method):
        """
        Glavna metoda za obradu specijalnih situacija
        
        Args:
            left_x: X koordinata leve linije
            right_x: X koordinata desne linije  
            left_visible: Da li je leva linija vidljiva
            right_visible: Da li je desna linija vidljiva
            current_node: Trenutni čvor na mapi
            navigate_command: Lista komandi za navigaciju (potrebno za preuzimanje direction)
            
        Returns:
            tuple: (angle, speed) ili (None, None) ako modul nije aktivan
        """
        current_time = time.time()
        dt = current_time - self.last_frame_time
        self.last_frame_time = current_time
        
        # Proverava da li je vozilo u zoni raskrsnice
        intersection_zone = self.is_in_intersection_zone(current_node)
        
        # Ažurira stanje raskrsnice
        if intersection_zone and not self.current_intersection:
            # Vozilo prvi put ulazi u raskrsnicu
            self.current_intersection = intersection_zone
            if navigate_command_getter_method:
                direction = navigate_command_getter_method()
                self.intersection_direction = direction
                self.pid.reset()
                if self.debugging:
                    print(f"Entering special intersection {intersection_zone}, direction: {direction}")
        elif not intersection_zone and self.current_intersection:
            # Vozilo izlazi iz raskrsnice
            if self.debugging:
                print(f"Exiting special intersection {self.current_intersection}")
            self.current_intersection = None
            self.intersection_direction = None
            self.pid.reset()
        
        if self.current_intersection and self.intersection_direction:
            return self._handle_intersection_control(left_x, right_x, left_visible, right_visible, dt)
        else:
            return self._handle_single_line_control(left_x, right_x, left_visible, right_visible, dt)
    
    def _handle_intersection_control(self, left_x, right_x, left_visible, right_visible, dt):
        """Upravljanje na raskrsnici"""
        tracking_side = self.determine_tracking_side(self.intersection_direction)
        
        if tracking_side == "left" and left_visible:
            error = self.calculate_single_line_error(left_x, "left")
            angle = self.pid.compute(error, dt=dt)
            
        elif tracking_side == "right" and right_visible:
            error = self.calculate_single_line_error(right_x, "right")
            angle = self.pid.compute(error, dt=dt)
            
        elif tracking_side == "left" and not left_visible:
            # Ide levo ali ne vidi levu liniju - skreće levo pod zadatim uglom
            angle = self.intersection_angle_degrees
            
        elif tracking_side == "right" and not right_visible:
            # Ide desno ali ne vidi desnu liniju - skreće desno pod zadatim uglom
            angle = -self.intersection_angle_degrees
            
        else:
            # Za pravo ili neočekivano stanje
            if left_visible and right_visible:
                # Standardno praćenje obe linije
                lane_center = (left_x + right_x) // 2
                error = self.center_x - lane_center
                angle = self.pid.compute(error, dt=dt)
            else:
                angle = 0
        
        speed = 250  # Smanjena brzina na raskrsnici
        
        if self.debugging:
            print(f"Intersection control: Direction={self.intersection_direction}, "
                  f"Tracking={tracking_side}, Angle={angle:.1f}°")
        
        return -int(angle * 10), speed
    
    def _handle_single_line_control(self, left_x, right_x, left_visible, right_visible, dt):
        """Upravljanje kada je vidljiva samo jedna linija"""
        current_time_ms = time.time() * 1000
        
        # Ažurira stanje vidljivosti linija
        if left_visible and right_visible:
            self.last_both_lines_time = time.time()
            self.single_line_active = False
            self.tracked_line_side = None
            return None, None  # Vraća kontrolu glavnom lane follow kontroleru
            
        # Proverava da li je dovoljno dugo vidljiva samo jedna linija
        time_since_both_lines = (time.time() - self.last_both_lines_time) * 1000
        
        if time_since_both_lines > self.single_line_threshold_ms:
            self.single_line_active = True
            
            if left_visible and not right_visible:
                self.tracked_line_side = "left"
                error = self.calculate_single_line_error(left_x, "left")
                
            elif right_visible and not left_visible:
                self.tracked_line_side = "right"
                error = self.calculate_single_line_error(right_x, "right")
                
            else:
                # Ni jedna linija nije vidljiva
                error = 0
                
            angle = self.pid.compute(error, dt=dt)
            speed = 220  # Smanjena brzina pri praćenju jedne linije
            
            if self.debugging:
                print(f"Single line control: Side={self.tracked_line_side}, "
                      f"Error={error:.1f}px, Angle={angle:.1f}°")
            
            return -int(angle * 10), speed
        
        return None, None
    
    def set_intersection_direction(self, direction: str):
        """Postavlja smer kretanja na raskrsnici"""
        if direction in ["Left", "Right", "Straight"]:
            self.intersection_direction = direction
            self.pid.reset()  # Reset PID-a pri promeni smera
            if self.debugging:
                print(f"Intersection direction set to: {direction}")
        else:
            raise ValueError(f"Invalid direction: {direction}")
    
    def clear_intersection_direction(self):
        """Briše smer raskrsnice kada vozilo napusti zonu"""
        self.intersection_direction = None
        self.current_intersection = None
        self.pid.reset()
        if self.debugging:
            print("Intersection direction cleared")
        
    def reset(self):
        """Reset modula u početno stanje"""
        self.single_line_active = False
        self.tracked_line_side = None
        self.last_both_lines_time = time.time()
        self.clear_intersection_direction()
        self.pid.reset()
        
    def is_active(self):
        """Proverava da li je modul trenutno aktivan"""
        return self.single_line_active or (self.current_intersection is not None and self.intersection_direction is not None)