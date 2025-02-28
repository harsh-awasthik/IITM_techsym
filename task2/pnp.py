import numpy as np
import cv2

CAMERA_MATRIX = np.array([
    [675.537322, 0.0,       311.191300],
    [0.0,        677.852071,221.610964],
    [0.0,        0.0,       1.0]
], dtype=np.float32)

DIST_COEFFS = np.zeros((5, 1), dtype=np.float32)

GATE_WIDTH = 75 #CM
GATE_HEIGHT = 75 #CM

# Points of the gate corners wrt gate
GATE_POINTS  = np.array([
    [-GATE_WIDTH/2,  GATE_HEIGHT/2,  0],  # top-left
    [ GATE_WIDTH/2,  GATE_HEIGHT/2,  0],  # top-right
    [ GATE_WIDTH/2, -GATE_HEIGHT/2,  0],  # bottom-right
    [-GATE_WIDTH/2, -GATE_HEIGHT/2,  0],  # bottom-left
], dtype=np.float32)


def apply_pnp(detected_points):
    success, rvec, tvec = cv2.solvePnP(
        GATE_POINTS,        # shape => (4,3)
        detected_points,    # shape => (4,2)
        CAMERA_MATRIX,
        DIST_COEFFS,
        flags=cv2.SOLVEPNP_IPPE
    )

    R, _ = cv2.Rodrigues(rvec)
    yaw, pitch, roll = rotational_to_euler(R)
    yaw_deg   = np.degrees(yaw)
    pitch_deg = np.degrees(pitch)
    roll_deg  = np.degrees(roll)

    tvec = tvec.flatten()

    line1 = f"Yaw={yaw_deg:5.1f}°, Pitch={pitch_deg:5.1f}°, Roll={roll_deg:5.1f}°"
    line2 = f"Tx={tvec[0]:.2f}, Ty={tvec[1]:.2f}, Tz={tvec[2]:.2f} (cm)"
    print(line1, line2, sep="\n")

    toreturn = [roll, pitch, yaw, tvec[0], tvec[1], tvec[2]]
    return toreturn

def rotational_to_euler(R):
    # Handle gimbal lock scenarios
    if np.isclose(R[2, 0], -1.0):
        # pitch = +90 deg 
        pitch = np.pi / 2
        yaw = np.arctan2(R[0, 1], R[0, 2])
        roll = 0.0
    elif np.isclose(R[2, 0], 1.0):
        # pitch = -90 deg
        pitch = -np.pi / 2
        yaw = np.arctan2(-R[0, 1], -R[0, 2])
        roll = 0.0
    else:
        # General case
        pitch = -np.arcsin(R[2, 0])  # around Y
        roll  =  np.arctan2(R[2, 1]/np.cos(pitch), R[2, 2]/np.cos(pitch))  # around X
        yaw   =  np.arctan2(R[1, 0]/np.cos(pitch), R[0, 0]/np.cos(pitch))  # around Z
    return yaw, pitch, roll



    
