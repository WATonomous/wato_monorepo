import math

def euler_from_quaternion(quaternion):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).
    
    Args:
    quaternion (list): A list containing the quaternion components [x, y, z, w].
    
    Returns:
    tuple: (roll, pitch, yaw) in radians.
    """
    if len(quaternion) != 4:
        raise ValueError("Quaternion must be a list of 4 elements [x, y, z, w]")
    
    x, y, z, w = quaternion
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)