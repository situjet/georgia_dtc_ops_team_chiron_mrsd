import math

# Example caps (use real specs for your gimbal!)
PITCH_MIN = -120.0  # down
PITCH_MAX = 120.0  # up
YAW_MIN = -170.0 # left
YAW_MAX = 170.0 # right

###needs to be set as an model parameter
# Center reference pitch angle (when target is at image center)
#CENTER_PITCH = -50.0

# Dampening factors to prevent overadjustment (can be tuned)
PITCH_GAIN = 0.5  # Reduce pitch movement by half
YAW_GAIN = 0.5    # Reduce yaw movement by half

def compute_gimbal_command(u, v, cx, cy, fx, fy, curr_pitch_deg, curr_yaw_deg):
    """
    Given a target pixel (u, v) and camera intrinsics, compute the new
    gimbal angles (pitch, yaw) needed to center the target.

    Parameters:
    - u, v: target pixel coordinates (e.g., center of bounding box)
    - cx, cy: principal point of the camera (usually image center)
    - fx, fy: focal lengths in pixels
    - curr_pitch_deg: current gimbal pitch angle in degrees
    - curr_yaw_deg: current gimbal yaw angle in degrees

    Returns:
    - new_pitch_deg, new_yaw_deg: angles to command to center target
    """
    print(f"[TRACK] ---- compute_gimbal_command START ----")
    print(f"[TRACK] Input: u={u}, v={v}, cx={cx}, cy={cy}, fx={fx}, fy={fy}")
    print(f"[TRACK] Current gimbal angles: pitch={curr_pitch_deg}°, yaw={curr_yaw_deg}°")

    # Check for invalid inputs
    if fx == 0 or fy == 0:
        print(f"[TRACK] ERROR: Invalid focal lengths fx={fx}, fy={fy}")
        raise ValueError("Focal lengths cannot be zero")

    # Offset from image center
    dx = u - cx
    dy = v - cy
    print(f"[TRACK] Pixel offset from center: dx={dx}, dy={dy}")

    # Convert pixel offset to angular offset (radians)
    try:
        yaw_offset_rad = math.atan2(dx, fx)
        # Negative dy to correct the direction (when target is above center, we want to move up)
        pitch_offset_rad = math.atan2(-dy, fy)
        print(f"[TRACK] Angular offset (rad): pitch={pitch_offset_rad}, yaw={yaw_offset_rad}")
    except Exception as e:
        print(f"[TRACK] ERROR in angular calculations: {str(e)}")
        raise

    # Convert to degrees
    yaw_offset_deg = math.degrees(yaw_offset_rad)
    pitch_offset_deg = math.degrees(pitch_offset_rad)
    print(f"[TRACK] Angular offset (deg): pitch={pitch_offset_deg}, yaw={yaw_offset_deg}")

    # Apply dampening factors to prevent overadjustment
    pitch_offset_deg *= PITCH_GAIN
    yaw_offset_deg *= YAW_GAIN
    print(f"[TRACK] Dampened offset (deg): pitch={pitch_offset_deg}, yaw={yaw_offset_deg}")
    
    # New target angles
    new_yaw_deg = curr_yaw_deg + yaw_offset_deg
    # For pitch
    new_pitch_deg = curr_pitch_deg + pitch_offset_deg
    print(f"[TRACK] New target angles (before clamping): pitch={new_pitch_deg}°, yaw={new_yaw_deg}°")

    # After computing new_pitch and new_yaw
    new_pitch_clamped = clamp(new_pitch_deg, PITCH_MIN, PITCH_MAX)
    new_yaw_clamped = clamp(new_yaw_deg, YAW_MIN, YAW_MAX)
    print(f"[TRACK] Clamped target angles: pitch={new_pitch_clamped}° [{PITCH_MIN},{PITCH_MAX}], yaw={new_yaw_clamped}° [{YAW_MIN},{YAW_MAX}]")
    
    # Check if angle change is significant
    pitch_change = abs(new_pitch_clamped - curr_pitch_deg)
    yaw_change = abs(new_yaw_clamped - curr_yaw_deg)
    print(f"[TRACK] Angle changes: pitch={pitch_change}°, yaw={yaw_change}°")
    
    print(f"[TRACK] ---- compute_gimbal_command END ----")
    return new_pitch_clamped, new_yaw_clamped

def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))



