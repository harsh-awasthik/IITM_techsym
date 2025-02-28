import cv2, numpy as np, os, time
from scipy.spatial import cKDTree
import pyrealsense2 as rs

GATE_COLOR = "R" #OR "G"
U_TH = 95   # For Red
V_TH = 151  # For Green

# === (A) Set Up RealSense Pipeline ===
print("....Initialising Realsence....")
pipe = rs.pipeline()
cfg  = rs.config()
cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
profile = pipe.start(cfg)
print("....Realsence Initialised....")

# TODO: Maybe this will be in the main task2 funtion
def send_gate_points():
    color_frame = None
    while not color_frame:
        frame       = pipe.wait_for_frames()
        color_frame = frame.get_color_frame()

    cv2.imshow("original", color_frame)

    masked = luv_mask(frame)
    if GATE_COLOR == "R":
        mask_noise_r = remove_noise_from_mask(masked, min_area=1000)
        mask_noise_r = cv2.bitwise_and(frame, frame, mask=mask_noise_r)
    else:
        mask_noise_r = remove_noise_from_mask(masked, min_area=30)
        mask_noise_r = cv2.bitwise_and(frame, frame, mask=mask_noise_r)

    corners, masked = get_gate_corners(mask_noise_r)

    cv2.imshow("masked", masked)

    return corners


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
        return [(-1, -1), (-1, -1), (-1, -1), (-1, -1)]
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