class CollisionDetector:
    def __init__(self, screen_width, screen_height, fixed_box_width, fixed_box_height, logging, debugging=True):
        self.target_width = screen_width
        self.target_height = screen_height
        self.logging = logging
        self.debuging = debugging
       
        center_x = self.target_width / 2
        center_y = self.target_height/ 2

        self.draw_x1 = int(center_x - fixed_box_width)
        self.draw_y1 = int(center_y - fixed_box_height)
        self.draw_x2 = int(center_x + fixed_box_width)
        self.draw_y2 = int(center_y + fixed_box_height)
        
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