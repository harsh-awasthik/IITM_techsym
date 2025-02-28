from pnp import *
from gate_detect import *

while True:
    gate_points = send_gate_points()

    ls = apply_pnp(gate_points)

    print(ls[3], ls[4], ls[5], sep = "\n")

    key = cv2.waitKey(1) & 0xFF
    if key == "q":
        break
