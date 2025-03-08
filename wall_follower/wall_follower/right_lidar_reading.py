import numpy as np
from sensor_msgs.msg import LaserScan

def compute_least_squares_line(
    scan_msg: LaserScan,
    start_angle: float,
    end_angle: float,
    cutoff
):
    weight = 2
    weighted_area = 0.2

    angle_min = scan_msg.angle_min
    angle_max = scan_msg.angle_max
    angle_inc = scan_msg.angle_increment
    ranges = scan_msg.ranges

    if start_angle > end_angle:
        start_angle, end_angle = end_angle, start_angle

    start_angle = max(angle_min, start_angle)
    end_angle   = min(angle_max, end_angle)

    start_idx = int(round((start_angle - angle_min) / angle_inc))
    end_idx   = int(round((end_angle   - angle_min) / angle_inc))
    start_idx = max(0, start_idx)
    end_idx   = min(end_idx, len(ranges) - 1)

    x_points = []
    y_points = []


    for i in range(start_idx, end_idx):
        r = ranges[i]

        if not np.isfinite(r) or r <= 0.0 or r >= cutoff:
            continue

        # Compute the actual angle for this index
        theta_i = angle_min + i * angle_inc

        # Convert to Cartesian in the LIDAR/car frame
        x = r * np.cos(theta_i)
        y = r * np.sin(theta_i)

        if i > weighted_area * (start_idx + end_idx):
            for j in range(weight):
                x_points.append(x)
                y_points.append(y)
        else:
            x_points.append(x)
            y_points.append(y)

    
    x_arr = np.array(x_points)
    y_arr = np.array(y_points)

    if len(x_arr) == 0:
        m, b = 0, 0
    else:
        m, b = np.polyfit(x_arr, y_arr, 1)

    return x_arr, y_arr, m, b
