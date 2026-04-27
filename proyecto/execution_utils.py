"""
Utility functions for open-loop path execution.
Handles path saving, final measurements, and result logging.
"""

import math
import os


def save_path_with_results(configurations, filepath, qf_est=None, qact=None):
    """
    Save the planned path to file with optional final estimates.
    
    Format:
    x,y,theta
    ...
    # qf-est: x,y,theta
    # qact: x,y,theta
    """
    with open(filepath, 'w') as f:
        f.write("# Trayectoria Geométrica\n")
        f.write("# Formato: x[m], y[m], theta[deg]\n")
        f.write("#\n")
        
        for x, y, theta in configurations:
            f.write(f"{x:.4f},{y:.4f},{theta:.4f}\n")
        
        if qf_est is not None:
            x_est, y_est, theta_est = qf_est
            f.write(f"\n# qf-est: {x_est:.4f},{y_est:.4f},{theta_est:.4f}\n")
        
        if qact is not None:
            x_act, y_act, theta_act = qact
            f.write(f"# qact: {x_act:.4f},{y_act:.4f},{theta_act:.4f}\n")


def estimate_position_from_odometry(x0, y0, theta0, configurations):
    """
    Estimate final position after executing all configurations.
    Uses dead reckoning: integrate rotations and translations.
    
    Returns: (x_est, y_est, theta_est)
    """
    x = x0
    y = y0
    theta = theta0
    
    i = 0
    while i < len(configurations):
        conf_x, conf_y, conf_theta = configurations[i]
        
        # Check if this is a rotation (same position, different angle)
        if i > 0:
            prev_x, prev_y, _ = configurations[i - 1]
            if abs(conf_x - prev_x) < 1e-6 and abs(conf_y - prev_y) < 1e-6:
                # Rotation at same location
                theta = math.radians(conf_theta)
                i += 1
                continue
        
        # Translation: move from current position to config position
        # Assume we're aligned with the config heading
        delta_x = conf_x - x
        delta_y = conf_y - y
        
        x = conf_x
        y = conf_y
        theta = math.radians(conf_theta)
        
        i += 1
    
    return (x, y, math.degrees(theta))


def compute_qact_from_lidar(x_est, y_est, theta_est_rad, d_front, d_right, 
                            d_front_expected=0.5, d_right_expected=0.5):
    """
    Compute relocated configuration using LiDAR measurements.
    
    Simple geometric correction:
    - If d_front deviates from expected, adjust y-position
    - If d_right deviates from expected, adjust x-position
    
    Args:
        x_est, y_est, theta_est_rad: estimated pose (theta in radians)
        d_front, d_right: measured distances from LiDAR
        d_front_expected, d_right_expected: expected distances to walls
    
    Returns:
        (x_act, y_act, theta_act_deg)
    """
    # Simple correction: assume robot is in corridor
    # Front distance measures distance to wall ahead
    # Right distance measures distance to right wall
    
    # Adjust position based on deviations
    x_act = x_est
    y_act = y_est
    theta_act_rad = theta_est_rad
    
    # If front distance differs, adjust forward-back position
    if d_front is not None and d_front < float('inf'):
        error_front = d_front - d_front_expected
        y_act += error_front  # adjust forward
    
    # If right distance differs, adjust left-right position
    if d_right is not None and d_right < float('inf'):
        error_right = d_right - d_right_expected
        x_act -= error_right  # adjust to correct side
    
    return (x_act, y_act, math.degrees(theta_act_rad))


def normalize_angle_deg(deg):
    """Normalize angle in degrees to [-180, 180)."""
    rad = math.radians(deg)
    normalized_rad = math.atan2(math.sin(rad), math.cos(rad))
    return math.degrees(normalized_rad)
