import cv2
import numpy as np

class ImagePreProcessing:
    def __init__(self, width: int, height: int, logging, debugging=False, pc=False, camera_fov_degrees: float = 79.3):
        self.debugging = debugging
        self.logging = logging
        self.camera_fov_degrees = camera_fov_degrees    
        self.width = width
        self.height = height
        self.pc = pc

        self.roadReg = np.array([[
                (int(self.width * 0.01), self.height - int(self.height * 0.05)),
                (int(self.width * 0.25), self.height - int(self.height * 0.05)),
                (int(self.width * 0.3), self.height - int(self.height * 0.2)),
                (int(self.width * 0.7), self.height - int(self.height * 0.2)),
                (int(self.width * 0.75), self.height - int(self.height * 0.05)),
                (int(self.width * 0.99), self.height - int(self.height * 0.05)),
                (int(self.width * 0.99), self.height * 0.50),
                (int(self.width * 0.01), self.height * 0.50)
            ]], np.int32)

        self.gamma_lut = self._create_gamma_lut(13)  # Precompute LUT for gamma correction
        self.mask = self._create_roi_mask()  # Precompute ROI mask

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def _create_gamma_lut(self, gamma: float):
        """Create a Look-Up Table for gamma correction."""
        lut = np.array([((i / 255.0) ** gamma) * 255 for i in range(256)], dtype=np.uint8)
        return lut

    def _create_roi_mask(self):
        """Precompute the region of interest mask."""
        mask = np.zeros((self.height, self.width), dtype=np.uint8)
        cv2.fillPoly(mask, self.roadReg, 255)
        return mask


    def process_frame(self, frame: np.ndarray):
        processed = np.empty((self.height, self.width), dtype=np.uint8)
        cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY, dst=processed)
        cv2.LUT(processed, self.gamma_lut, dst=processed)

		# (simpler; not minimal) bounding rect around roi 
        x, y, w, h = cv2.boundingRect(self.mask)

		# crop both image and mask (mask is cropped to be applied to the cropped image)
        crop_img = processed[y:y+h, x:x+w]
        crop_mask = self.mask[y:y+h, x:x+w]

		# CLAHE on cropped image (without mask applied)
        clahe_applied = self.clahe.apply(crop_img)

		# apply mask AFTER CLAHE to avoid confusing clahe
        clahe_applied[crop_mask == 0] = 0

		# put everything back into original frame size
        full_processed = np.zeros_like(processed)
        full_processed[y:y+h, x:x+w] = clahe_applied

        cv2.medianBlur(full_processed, 5, dst=full_processed)
        _, full_processed = cv2.threshold(full_processed, 230, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        edges = cv2.ximgproc.thinning(full_processed, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        return edges