from typing import List, Tuple
import math

def median_2d_point(points: List[Tuple[float, float]]) -> Tuple[float, float]:
    # Validate input
    if not isinstance(points, list) or len(points) == 0:
        raise ValueError("Input must be a non-empty list of (x, y) tuples.")
    if len(points) % 2 == 0:
        raise ValueError("Input list must have an odd number of elements.")
    for p in points:
        if not (isinstance(p, tuple) or isinstance(p, list)) or len(p) != 2:
            raise ValueError("Each element must be a tuple or list of two numbers (x, y).")
        if not (isinstance(p[0], (int, float)) and isinstance(p[1], (int, float))):
            raise ValueError("Each coordinate must be a number.")

    # Compute average point
    avg_x = sum(p[0] for p in points) / len(points)
    avg_y = sum(p[1] for p in points) / len(points)
    # avg_point = (avg_x, avg_y)

    # Sort by distance to average point
    sorted_points = sorted(points, key=lambda p: math.hypot(p[0] - avg_x, p[1] - avg_y))
    
    print(avg_x, avg_y)
    print(sorted_points)

    # Return the middle element
    mid = len(sorted_points) // 2
    return sorted_points[mid]

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # Test data (odd number of points)
    points = [
        (1, 1),
        (2, 3),
        (3, 2),
        (4, 4),
        (5, 0),
        (0, 5),
        (3, 3),
        (10, 10),
        (5, 1),
    ]

    # Compute the median point using your function
    median_point = median_2d_point(points)

    # Compute average point (to plot it)
    avg_x = sum(p[0] for p in points) / len(points)
    avg_y = sum(p[1] for p in points) / len(points)
    avg_point = (avg_x, avg_y)

    # Plot
    x_vals, y_vals = zip(*points)
    plt.scatter(x_vals, y_vals, label="Input Points", color="blue")
    plt.scatter(*avg_point, label="Average Point", color="orange", marker="x", s=100)
    plt.scatter(*median_point, label="Median Point", color="red", marker="D", s=80)

    for p in points:
        plt.plot([avg_point[0], p[0]], [avg_point[1], p[1]], 'k--', linewidth=0.5)

    plt.legend()
    plt.axis("equal")
    plt.title("2D Points and Median Point by Distance to Average")
    plt.grid(True)
    plt.show()
