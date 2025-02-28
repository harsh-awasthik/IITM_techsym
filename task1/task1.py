import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
from threading import Thread
from pid import *

TARGET_DEPTH = 40.0
SEC_TO_MOVE_FORWARD = 10

dist_coeffs = np.zeros((5, 1), dtype=np.float32) 

pid_surge = PIDController(kp=0.5, ki=0.0, kd=0.1)
pid_yaw   = PIDController(kp=1.0, ki=0.0, kd=0.1)
pid_heave = PIDController(kp=4.0, ki=0.3, kd=3.0)
pid_pitch = PIDController(kp=1.0, ki=0.0, kd=0.2)


def initialise_imu():
    start = time.time()
    print("----Calibrating Yaw----")

    while time.time() < (start + 2):
        euler_angles = imu.get_euler_angles()  # (roll, pitch, yaw) in degrees

    target_yaw = euler_angles[2]
    if target_yaw >= 180:
        target_yaw -= 360
    return target_yaw
TARGET_YAW = initialise_imu()


def move_forward(): 
    while get_yaw_error(TARGET_YAW) != 0:
        print("----Correcting Yaw----")
        current_time = time.time()
        e_yaw = get_yaw_error()
        e_heave = get_heave_error()
        e_pitch = get_pitch_error()

        # Update each PID and obtain control outputs
        #u_surge = pid_surge.update(e_surge, current_time)
        u_surge = 0
        u_yaw   = pid_yaw.update(e_yaw, current_time)
        u_heave = pid_heave.update(e_heave, current_time)
        u_pitch = pid_pitch.update(e_pitch, current_time)

        # Mix into thruster PWMs and send commands
        thruster_pwms = mix_outputs(u_surge, u_yaw, u_heave, u_pitch)
        print(thruster_pwms)
        send_pwm(thruster_pwms)
        time.sleep(0.05)

    print("----Yaw is Oriented---")                

    LT1 = time.time()

    while time.time() < (LT1 + SEC_TO_MOVE_FORWARD):
        print("----Moving Forward----")
        e_heave = get_heave_error()
        e_pitch = get_pitch_error()
        e_yaw = get_yaw_error(TARGET_YAW)

        # Update each PID and obtain control outputs
        #u_surge = pid_surge.update(e_surge, current_time)
        u_surge = 120
        u_yaw   = pid_yaw.update(e_yaw, current_time)
        u_heave = pid_heave.update(e_heave, current_time)
        u_pitch = pid_pitch.update(e_pitch, current_time)


        # Mix into thruster PWMs and send commands
        thruster_pwms = mix_outputs(u_surge, u_yaw, u_heave, u_pitch)

        print(thruster_pwms)
        send_pwm(thruster_pwms)
        time.sleep(0.05)                                                                                                                                                       


# =======================
# Sensor Error Functions (Stubs / Example)
# =======================
def get_surge_error():
    return 0.0

def get_yaw_error(target_yaw):
    euler_angles = imu.get_euler_angles()  # (roll, pitch, yaw) in degrees
    current_yaw = euler_angles[2]
    print(current_yaw)
    if current_yaw >= 180:
        current_yaw -= 360

    error_yaw = target_yaw - current_yaw
    if abs(error_yaw) < 2:
        error_yaw = 0

    print(f"current yaw {current_yaw}")
    print(f"error yaw {error_yaw}")
    return error_yaw

def get_heave_error():
    depth = depth_sensor.get_depth()
    print(f"current Depth {depth}")
    #depth = float(10)  # example value
    error = TARGET_DEPTH - depth
    print(f"error Depth {error}")
 
    return error

def get_pitch_error():    
    return 0

# =======================
# Main Control Loop
# =======================
def main_loop():
    """
    Main control loop that continuously reads sensor errors, updates PIDs,
    mixes outputs, and sends PWM signals.
    Loop stops as soon as stop_event is set.
    """
    while not stop_event.is_set():
        print("-" * 30)
        
        move_forward()

def run_main_loop():
    try:
        initialize_thrusters()
        main_loop()
    except KeyboardInterrupt:
        print("Shutting down control loop.")
    # Note: Do not send final PWM here;
    # we'll handle it after ensuring the thread has stopped.


# =======================
# Global Lists for Plotting PID Components
# =======================
time_values = []

# Yaw PID components and target (target is 0.0)
yaw_p_values = []
yaw_i_values = []
yaw_d_values = []
yaw_target_values = []

# Pitch PID components and target (target is 0.0)
pitch_p_values = []
pitch_i_values = []
pitch_d_values = []
pitch_target_values = []

# Heave PID components and target (target is 50.0)
heave_p_values = []
heave_i_values = []
heave_d_values = []
heave_target_values = []

# =======================
# Plot Setup: Single Figure with 3 Subplots
# =======================
fig, (ax_yaw, ax_pitch, ax_heave) = plt.subplots(3, 1, figsize=(10, 8))

# --- Yaw subplot ---
line_yaw_p, = ax_yaw.plot([], [], 'r-', label="Yaw P")
line_yaw_i, = ax_yaw.plot([], [], 'g-', label="Yaw I")
line_yaw_d, = ax_yaw.plot([], [], 'b-', label="Yaw D")
line_yaw_target, = ax_yaw.plot([], [], 'k--', label="Yaw Target")
ax_yaw.set_title("Yaw PID Components and Target")
ax_yaw.set_xlabel("Time (s)")
ax_yaw.set_ylabel("Value")
ax_yaw.legend()
ax_yaw.grid(True)

# --- Pitch subplot ---
line_pitch_p, = ax_pitch.plot([], [], 'r-', label="Pitch P")
line_pitch_i, = ax_pitch.plot([], [], 'g-', label="Pitch I")
line_pitch_d, = ax_pitch.plot([], [], 'b-', label="Pitch D")
line_pitch_target, = ax_pitch.plot([], [], 'k--', label="Pitch Target")
ax_pitch.set_title("Pitch PID Components and Target")
ax_pitch.set_xlabel("Time (s)")
ax_pitch.set_ylabel("Value")
ax_pitch.legend()
ax_pitch.grid(True)

# --- Heave subplot ---
line_heave_p, = ax_heave.plot([], [], 'r-', label="Heave P")
line_heave_i, = ax_heave.plot([], [], 'g-', label="Heave I")
line_heave_d, = ax_heave.plot([], [], 'b-', label="Heave D")
line_heave_target, = ax_heave.plot([], [], 'k--', label="Heave Target")
ax_heave.set_title("Heave PID Components and Target")
ax_heave.set_xlabel("Time (s)")
ax_heave.set_ylabel("Value")
ax_heave.legend()
ax_heave.grid(True)

start_time = time.time()

def update_plot(frame):
    """
    Animation update function that appends the latest PID components (from the
    controllers updated by the main control loop) and updates the subplots.
    """
    current_time = time.time() - start_time
    time_values.append(current_time)

    # --- Yaw ---
    yaw_p = getattr(pid_yaw, "last_p", 0.0)
    yaw_i = getattr(pid_yaw, "last_i", 0.0)
    yaw_d = getattr(pid_yaw, "last_d", 0.0)
    yaw_target = 0.0
    yaw_p_values.append(yaw_p)
    yaw_i_values.append(yaw_i)
    yaw_d_values.append(yaw_d)
    yaw_target_values.append(yaw_target)

    # --- Pitch ---
    pitch_p = getattr(pid_pitch, "last_p", 0.0)
    pitch_i = getattr(pid_pitch, "last_i", 0.0)
    pitch_d = getattr(pid_pitch, "last_d", 0.0)
    pitch_target = 0.0
    pitch_p_values.append(pitch_p)
    pitch_i_values.append(pitch_i)
    pitch_d_values.append(pitch_d)
    pitch_target_values.append(pitch_target)

    # --- Heave ---
    heave_p = getattr(pid_heave, "last_p", 0.0)
    heave_i = getattr(pid_heave, "last_i", 0.0)
    heave_d = getattr(pid_heave, "last_d", 0.0)
    heave_target = 50.0
    heave_p_values.append(heave_p)
    heave_i_values.append(heave_i)
    heave_d_values.append(heave_d)
    heave_target_values.append(heave_target)

    # Keep only the last 100 data points
    if len(time_values) > 100:
        time_values.pop(0)
        for lst in (yaw_p_values, yaw_i_values, yaw_d_values, yaw_target_values,
                    pitch_p_values, pitch_i_values, pitch_d_values, pitch_target_values,
                    heave_p_values, heave_i_values, heave_d_values, heave_target_values):
            lst.pop(0)

    # Update yaw subplot
    line_yaw_p.set_data(time_values, yaw_p_values)
    line_yaw_i.set_data(time_values, yaw_i_values)
    line_yaw_d.set_data(time_values, yaw_d_values)
    line_yaw_target.set_data(time_values, yaw_target_values)
    ax_yaw.relim()
    ax_yaw.autoscale_view()

    # Update pitch subplot
    line_pitch_p.set_data(time_values, pitch_p_values)
    line_pitch_i.set_data(time_values, pitch_i_values)
    line_pitch_d.set_data(time_values, pitch_d_values)
    line_pitch_target.set_data(time_values, pitch_target_values)
    ax_pitch.relim()
    ax_pitch.autoscale_view()

    # Update heave subplot
    line_heave_p.set_data(time_values, heave_p_values)
    line_heave_i.set_data(time_values, heave_i_values)
    line_heave_d.set_data(time_values, heave_d_values)
    line_heave_target.set_data(time_values, heave_target_values)
    ax_heave.relim()
    ax_heave.autoscale_view()

    return (line_yaw_p, line_yaw_i, line_yaw_d, line_yaw_target,
            line_pitch_p, line_pitch_i, line_pitch_d, line_pitch_target,
            line_heave_p, line_heave_i, line_heave_d, line_heave_target)

# Create animation (updates every 500 ms)
ani = animation.FuncAnimation(fig, update_plot, interval=500, blit=False, cache_frame_data=False)


# =======================
# Run Main Loop in a Separate Thread and Show Plots
# =======================
try:
    main_thread = Thread(target=run_main_loop)
    main_thread.start()

    plt.tight_layout()
    plt.show()  # When the plot window is closed, execution continues here.
except KeyboardInterrupt:
    print("KeyboardInterrupt detected in parent thread.")
finally:
    print("\n[INFO] Stopping AUV control loop...")
    stop_event.set()  # Signal the control loop to stop
    main_thread.join()

    # Reduce sleep delay to ensure quick termination
    for _ in range(10):  
        if not main_thread.is_alive():
            break  # Exit if the thread has already stopped
        time.sleep(0.1)  # Reduce sleep delay for faster exit

    if main_thread.is_alive():
        print("[WARNING] Force stopping the thread!")
        # Optional: Force stop the thread (if necessary)
        import ctypes
        ctypes.pythonapi.PyThreadState_SetAsyncExc(
            ctypes.c_long(main_thread.ident), ctypes.py_object(SystemExit)
        )

    print("[INFO] Sending 1500 to all thrusters (neutral)...")
    send_pwm(([1500] * 5),flag = False)  # Ensure motors stop
    print("[INFO] Executed finally block. System shut down.")