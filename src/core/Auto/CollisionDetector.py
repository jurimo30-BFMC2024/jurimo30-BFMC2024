import numpy as np
import cv2

class CollisionDetector:
    def __init__(self, screen_width, screen_height, fixed_box_width, fixed_box_height, logging, debugging=True):
        self.target_width = screen_width
        self.target_height = screen_height
        self.logging = logging
        self.debuging = debugging
       
        center_x = self.target_width / 2
        center_y = self.target_height/ 2

        draw_x1 = int(center_x - fixed_box_width)
        draw_y1 = int(center_y - fixed_box_height)
        draw_x2 = int(center_x + fixed_box_width)
        draw_y2 = int(center_y + fixed_box_height)

        # Create trapezoid points (wider at bottom, narrower at top)
        center_x = (draw_x1 + draw_x2) // 2
        width = draw_x2 - draw_x1
        top_width = int(width * 0.6)  # Top is 60% of bottom width
        
        self.trapezoid_points = np.array([
            [center_x - top_width//2 - 10, draw_y1 - 10],      # Top left
            [center_x + top_width//2 + 10, draw_y1 - 10],      # Top right
            [draw_x2 + 30, draw_y2 + 15],                      # Bottom right
            [draw_x1 - 30, draw_y2 + 15]                       # Bottom left
        ], np.int32)
    
    def check_collision(self, object_bbox, object_name):
        x1, y1, x2, y2 = object_bbox

        # Površina detektovanog objekta
        object_area = max(0, x2 - x1) * max(0, y2 - y1)

        # Create object rectangle points
        object_points = np.array([
            [x1, y1],  # Top left
            [x2, y1],  # Top right
            [x2, y2],  # Bottom right
            [x1, y2]   # Bottom left
        ], np.int32)

        # Calculate intersection using OpenCV
        intersection_area = self._calculate_polygon_intersection(self.trapezoid_points, object_points)

        # Određivanje praga na osnovu tipa objekta
        if object_name == "car":
            threshold = 0.2  # 50% za car
        elif object_name == "stefanija":
            threshold = 0.001  # 5% za stefaniju
        else:
            threshold = 0.5  # default 50%

        if self.debuging:
            print(f"Detektovan objekat {object_name} unutar regiona: {intersection_area}, prag: {threshold * object_area}")

        # Provera da li je dovoljno objekta unutar fiksne oblasti
        if intersection_area >= threshold * object_area:
            print(f"Detektovan objekat {object_name} unutar regiona")
            return True
        else:
            print(f"Bez detekcije za {object_name}")
            return False

    def _calculate_polygon_intersection(self, poly1, poly2):
        """Calculate intersection area between two polygons using OpenCV."""
        # Create masks for both polygons
        mask1 = np.zeros((self.target_height, self.target_width), dtype=np.uint8)
        mask2 = np.zeros((self.target_height, self.target_width), dtype=np.uint8)
        
        # Fill polygons
        cv2.fillPoly(mask1, [poly1], 255)
        cv2.fillPoly(mask2, [poly2], 255)
        
        # Calculate intersection
        intersection = cv2.bitwise_and(mask1, mask2)
        intersection_area = cv2.countNonZero(intersection)
        
        return intersection_area