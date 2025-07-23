import cv2
import numpy as np
import base64

def encode_frame(yuv_frame: np.ndarray) -> bytes:
    """
    Convert YUV420 frame to BGR and encode it as JPEG.
    Returns JPEG bytes.
    """
    # Convert YUV420 (I420) to BGR for JPEG encoding
    bgr_frame = cv2.cvtColor(yuv_frame, cv2.COLOR_YUV2BGR_I420)

    # JPEG encode the BGR frame
    success, encoded = cv2.imencode(".jpg", bgr_frame)
    # success, encoded = cv2.imencode(".jpg", bgr_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not success:
        raise RuntimeError("Failed to encode image")

    return encoded.tobytes()


def decode_frame(encoded_bytes: bytes) -> np.ndarray:
    """
    Decode JPEG bytes back to a BGR image.
    """
    np_array = np.frombuffer(encoded_bytes, dtype=np.uint8)
    return cv2.imdecode(np_array, cv2.IMREAD_COLOR)


def decode_frame_to_base64(encoded_bytes: bytes, quality: int = 60) -> str:
    """
    Decode JPEG bytes and return a re-encoded base64 JPEG string with compression.
    `quality` controls JPEG quality (1-100), lower means more compression.
    """
    frame = decode_frame(encoded_bytes)
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
    success, reencoded = cv2.imencode(".jpg", frame, encode_params)
    if not success:
        raise RuntimeError("Failed to re-encode image")
    return base64.b64encode(reencoded).decode("utf-8")