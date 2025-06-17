import cv2

class CollisionDetector:
    def __init__(self, screen_width, screen_height, fixed_box_width, fixed_box_height, logging, debugging=True):
        center_x = screen_width / 2
        center_y = screen_height / 2

        self.logging = logging
        self.debuging = debugging
        self.processing_width = 256
        self.processing_height = 256

        self.fixed_x_min = center_x - fixed_box_width / 2  # donji levi
        self.fixed_y_min = center_y - fixed_box_height / 2 # donji
        self.fixed_x_max = center_x + fixed_box_width / 2  # donji desni
        self.fixed_y_max = center_y + fixed_box_height / 2 # gornji
        # Scale factors to convert from target dimensions to processing dimensions
        # (dimensions of frame_to_draw_on)
        scale_x_target_to_processing = self.processing_width / screen_width
        scale_y_target_to_processing = self.processing_height / screen_height
        # Target coordinates
        tx1 = int(self.fixed_x_min)
        ty1 = int(self.fixed_y_min)
        tx2 = int(self.fixed_x_max)
        ty2 = int(self.fixed_y_max)
        # Scale the coordinates to the processing frame dimensions
        self.draw_x1 = int(tx1 * scale_x_target_to_processing)
        self.draw_y1 = int(ty1 * scale_y_target_to_processing)
        self.draw_x2 = int(tx2 * scale_x_target_to_processing)
        self.draw_y2 = int(ty2 * scale_y_target_to_processing)
        
    def check_collision(self, object_bbox):
        x1, y1, x2, y2 = object_bbox

        # Površina detektovanog objekta
        object_area = max(0, x2 - x1) * max(0, y2 - y1)

        # Koordinate preseka između objekta i fiksne oblasti
        inter_x1 = max(x1, self.draw_x1)
        inter_y1 = max(y1, self.draw_y1)
        inter_x2 = min(x2, self.draw_x2)
        inter_y2 = min(y2, self.draw_y2)

        # Površina preseka
        inter_width = max(0, inter_x2 - inter_x1)
        inter_height = max(0, inter_y2 - inter_y1)
        intersection_area = inter_width * inter_height
        if self.debuging:
            print(f"Detektovan objekat unutar regiona{intersection_area}")

        # Provera da li je barem polovina objekta unutar fiksne oblasti
        if intersection_area >= 0.5 * object_area:
            print(f"Detektovan objekat unutar regiona")
            return True
        else:
            print(f"Bez detekcije")
            return False