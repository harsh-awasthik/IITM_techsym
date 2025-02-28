import numpy as np
import cv2, numpy as np, os, time
from scipy.spatial import cKDTree
import pyrealsense2 as rs

GATE_COLOR =  "G"
U_TH = 95   # For Red
V_TH = 11 # For Green

# === (A) Set Up RealSense Pipeline ===
print("....Initialising Realsence....")
pipe = rs.pipeline()
cfg  = rs.config() 
cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
profile = pipe.start(cfg)
print("....Realsence Initialised....")

# TODO: Maybe this will be in the main task2 funtion
def send_gate_points():
    while True:
        
        frame       = pipe.wait_for_frames()
        color_frame = frame.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())

        cv2.imshow("original", frame)

        masked = luv_mask(frame, GATE_COLOR)
        if GATE_COLOR == "R":
            mask_noise_r = remove_noise_from_mask(masked, min_area=1000)
            mask_noise_r = cv2.bitwise_and(frame, frame, mask=mask_noise_r)
        else:
            mask_noise_r = remove_noise_from_mask(masked, min_area=30)
            # mask_noise_r = cv2.bitwise_and(frame, frame, mask=mask_noise_r)
        
        corners, masked = get_gate_corners(mask_noise_r)


        corners = np.asanyarray(corners, dtype=np.float32)
        print(corners)
        array = apply_pnp(corners)
        print(array[3:])

        cv2.imshow("masked", masked)

        key = cv2.waitKey(1) & 0xFF

        if key == "q":
            break



def luv_mask(img_bgr, gate):
    # Convert BGR -> RGB -> LUV

    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    img_luv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2LUV)
    
    if gate == "R":
        mask = cv2.threshold(img_luv[:, :, 1], U_TH, 255, cv2.THRESH_BINARY)[1]
    else:
        mask = cv2.threshold(img_luv[:, :, 2], V_TH, 255, cv2.THRESH_BINARY)[1]

    return mask


def remove_noise_from_mask(mask, min_area=100):
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Create a new mask to draw only large blobs
    mask_filtered = np.zeros_like(mask)
    
    # Filter contours based on area
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > min_area:  # Keep only large contours
            cv2.drawContours(mask_filtered, [cnt], -1, 255, thickness=cv2.FILLED)
    
    return mask_filtered

def get_gate_corners(image):
    # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ys, xs = np.where(image)

    # print(list(ys), xs)
    points = np.column_stack((ys, xs))  # shape: (N, 2)
    tree = cKDTree(points)

    image_corners = [(0, 0), (image.shape[0]-1, 0), (image.shape[0]-1, image.shape[1]-1), (0, image.shape[1]-1)]
    
    nearest_points = []
    if not points.any():
        return [(-1, -1), (-1, -1), (-1, -1), (-1, -1)], image
    for corner in image_corners:
        cy, cx = corner
        dist, idx = tree.query([cy, cx])  # cKDTree.query() returns (distance, index)
        nearest_y, nearest_x = map(int, points[idx])  # Convert np.int64 to int
        nearest_points.append((nearest_x, nearest_y))
        print(f"Corner {corner} -> Nearest unmasked pixel: (row={nearest_y}, col={nearest_x})")
    
    toreturn = [nearest_points[0], nearest_points[3], nearest_points[2], nearest_points[1]]
    if toreturn[0][0] <= 2 or toreturn[0][1] <= 2:
        toreturn[0] = [-1, -1]
    if toreturn[1][0] >= image.shape[1]-2 or toreturn[1][1] <= 2:
        toreturn[1] = [-1, -1]
    if toreturn[2][0] >= image.shape[1]-2 or toreturn[2][1] >= image.shape[0]-2:
        toreturn[2] = [-1, -1]
    if toreturn[3][0] <= 2 or image.shape[0]-2 <= toreturn[3][1]:
        toreturn[3] = [-1, -1]
    
    dist = []
    print(toreturn)

    for i in range(4):
        if i == 3:
            j = 0
        else:
            j = i+1
        dist.append((toreturn[i][0]-toreturn[j][0])**2 + (toreturn[i][1] - toreturn[j][1])**2)

    print(dist)
    if dist[0] != 0 and dist[1] != 0 and (dist[0]/dist[1]) > 10:
        toreturn[2] = [-1, -1]
    if dist[2] != 0 and dist[3] != 0 and (dist[2]/dist[3]) > 10:
        toreturn[3] = [-1, -1]
        
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    for point in toreturn:
        if not np.array_equal(point, [-1, -1]):
            cv2.circle(image, (int(point[0]), int(point[1])), 5, (255, 0, 255), -1)
            
    return toreturn, image

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
    R = np.zeros((3, 3), dtype=np.float32)
    if rvec is not None:
        R, _ = cv2.Rodrigues(rvec)
    yaw, pitch, roll = rotational_to_euler(R)
    yaw_deg   = np.degrees(yaw)
    pitch_deg = np.degrees(pitch)
    roll_deg  = np.degrees(roll)

    if tvec is not None:
        tvec = tvec.flatten()
    else:
        tvec = np.zeros(3, dtype=np.float32)

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


while True:
    gate_points = send_gate_points()

    ls = apply_pnp(gate_points)

    print(ls[3], ls[4], ls[5], sep = "\n")

    key = cv2.waitKey(1) & 0xFF
    if key == "q":
        break
