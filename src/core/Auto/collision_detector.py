import cv2

class CollisionDetector:
    def __init__(self, screen_width, screen_height, fixed_box_width, fixed_box_height):
        center_x = screen_width / 2
        center_y = screen_height / 2

        self.fixed_x_min = center_x - fixed_box_width / 2  # donji levi
        self.fixed_y_min = center_y - fixed_box_height / 2 # donji
        self.fixed_x_max = center_x + fixed_box_width / 2  # donji desni
        self.fixed_y_max = center_y + fixed_box_height / 2 # gornji

    def check_collision(self, object_bbox):
        x1, y1, x2, y2 = object_bbox

        # Površina detektovanog objekta
        object_area = max(0, x2 - x1) * max(0, y2 - y1)

        # Koordinate preseka između objekta i fiksne oblasti
        inter_x1 = max(x1, self.fixed_x_min)
        inter_y1 = max(y1, self.fixed_y_min)
        inter_x2 = min(x2, self.fixed_x_max)
        inter_y2 = min(y2, self.fixed_y_max)

        # Površina preseka
        inter_width = max(0, inter_x2 - inter_x1)
        inter_height = max(0, inter_y2 - inter_y1)
        intersection_area = inter_width * inter_height

        # Provera da li je barem polovina objekta unutar fiksne oblasti
        if intersection_area >= 0.5 * object_area:
            return True
        else:
            return False

    def draw_fixed_box(self, frame):

        x1 = int(self.fixed_x_min)
        y1 = int(self.fixed_y_min)
        x2 = int(self.fixed_x_max)
        y2 = int(self.fixed_y_max)
        
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        return frame