import numpy as np
from sensor_msgs.msg import LaserScan

def compute_least_squares_line(
    scan_msg: LaserScan,
    start_angle: float,
    end_angle: float,
    cutoff
):
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
    # angles = np.array([angle_min + angle_inc * i for i in range(0, int((angle_max - angle_min)/angle_inc))])
    # np.append(angles, angle_max)
    # front_distances = []
    # front_angles = []
    # front_x = []
    # front_y = []
    # for i in range(len(angles)):
    #     if angles[i] < 0.03 and angles[i] > -0.03 and ranges[i] <= 2:
    #         front_distances.append(ranges[i])
    #         front_angles.append(angles[i])

    # for i in range(len(front_angles)):
    #     front_x.append(front_angles[i] * np.cos(front_angles[i]))
    #     front_y.append(front_angles[i] * np.sin(front_angles[i]))

    for i in range(start_idx, end_idx):
        r = ranges[i]
        # 0.0 -> 0.2
        if not np.isfinite(r) or r <= 0 or r >= cutoff:
            continue

        # Compute the actual angle for this index
        theta_i = angle_min + i * angle_inc

        # Convert to Cartesian in the LIDAR/car frame
        x = r * np.cos(theta_i)
        y = r * np.sin(theta_i)

        x_points.append(x)
        y_points.append(y)

    # x_points.extend(front_x)
    # y_points.extend(front_y)

    x_arr = np.array(x_points)
    y_arr = np.array(y_points)

    if len(x_arr) == 0:
        m, b = 0, 0
    else:
        m, b = np.polyfit(x_arr, y_arr, 1)

    return x_arr, y_arr, m, b

