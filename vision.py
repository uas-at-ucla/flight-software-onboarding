import cv2
import numpy as np


def find_red_object(
    image: np.ndarray,
) -> tuple[int, int] | None:
    """Return the center of the largest red object, or None if none is found."""

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Red wraps around both ends of OpenCV's hue range.
    lower_red1 = np.array([0, 100, 70], dtype=np.uint8)
    upper_red1 = np.array([10, 255, 255], dtype=np.uint8)

    lower_red2 = np.array([170, 100, 70], dtype=np.uint8)
    upper_red2 = np.array([179, 255, 255], dtype=np.uint8)

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Remove isolated pixels and fill small holes.
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)

    # Reject tiny red patches caused by noise.
    if cv2.contourArea(largest) < 100:
        return None

    moments = cv2.moments(largest)

    if moments["m00"] == 0:
        return None

    center_x = int(moments["m10"] / moments["m00"])
    center_y = int(moments["m01"] / moments["m00"])

    return center_x, center_y