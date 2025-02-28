import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import serial
from threading import Thread, Event

from ..test.imu_reader import IMUReader
from ..test.pres import DepthSensor

# =======================
# Hardware and Sensor Setup
# =======================
depth_sensor = DepthSensor('/dev/ttyUSB_DEPTH')
imu = IMUReader(port="/dev/ttyUSB_IMU", baudrate=921600)
arduino_port_pwm = '/dev/ttyUSB_PWM'
baud_rate = 115200

def initialise_imu():
    start = time.time()
    print("----Calibrating Yaw----")

    while time.time() < (start + 2):
        euler_angles = imu.get_euler_angles()  # (roll, pitch, yaw) in degrees

    current_yaw = euler_angles[2]
    if current_yaw >= 180:
        current_yaw = 360-current_yaw
    else:
         current_yaw = -current_yaw

    return current_yaw

TARGET_YAW = initialise_imu()
TARGET_DEPTH = 75


# Start background reading for IMU
imu.start_reading()

# Set up serial communication to Arduino for thrusters
arduino = serial.Serial(arduino_port_pwm, baud_rate)
time.sleep(2)  

# -----------------------
# Global Stop Event
# -----------------------
stop_event = Event()

# =======================
# PID Controller Class (Modified)
# =======================
class PIDController:
    """
    A generic PID controller class for one DOF.
    Now also stores the individual P, I, and D components for plotting.
    """
    def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self._integral = 0.0
        self._prev_error = 0.0
        self._output_limits = output_limits
        self._last_time = None

        # For storing individual PID contributions
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_error = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None

    def update(self, measurement, current_time=None):
        """
        Update the PID controller with a new measurement.
        Returns the control output (u) and saves P, I, D for plotting.
        """
        if current_time is None:
            current_time = time.time()

        if self._last_time is None:
            dt = 0.0
        else:
            dt = current_time - self._last_time

        # Error: here measurement is assumed to be (target - current_value)
        error = measurement

        # Proportional term
        p = self.kp * error

        # Integral term
        self._integral += error * dt
        i = self.ki * self._integral

        # Derivative term
        d = 0.0
        if dt > 1e-6:
            d_error = (error - self._prev_error) / dt
            d = self.kd * d_error

        output = p + i + d

        # Apply output limits if any
        lower, upper = self._output_limits
        if lower is not None:
            output = max(output, lower)
        if upper is not None:
            output = min(output, upper)

        # Save state for next iteration
        self._prev_error = error
        self._last_time = current_time

        # Save individual components for plotting
        self.last_p = p
        self.last_i = i
        self.last_d = d
        self.last_error = error

        return output

# =======================
# Create PIDs for each DOF with example gains
# (Tune these as needed)
# =======================
pid_surge = PIDController(kp=0.5, ki=0.0, kd=0.1)
pid_yaw   = PIDController(kp=0.7, ki=0.05, kd=0.65)
pid_heave = PIDController(kp=4.0, ki=0.5, kd=3.0)
pid_pitch = PIDController(kp=1.0, ki=0.0, kd=0.2)

# =======================
# Mixing and PWM functions (unchanged)
# =======================
alpha_surge  = 1.0
alpha_yaw    = 1.0
beta_heave   = 1.0
gamma_pitch  = 1.0

PWM_NEUTRAL  = 1500
PWM_MIN      = 1200
PWM_MAX      = 1800

def map_pwm_value(value):
    """
    Maps PWM values:
    - 1100 to 1490 mapped linearly to 1100 to 1460 (reverse)
    - 1510 to 1900 mapped linearly to 1540 to 1900 (forward)
    - Values between 1490-1510 remain unchanged (neutral)
    """
    if 1100 <= value <= 1490:
        return int(1100 + (value - 1100) * (360 / 390))  # Scale to 1100-1460
    elif 1510 <= value <= 1900:
        return int(1540 + (value - 1510) * (360 / 390))  # Scale to 1540-1900
    else:
        return int(1500)  # Keep neutral values unchanged



def mix_outputs(u_surge, u_yaw, u_heave, u_pitch):
    """
    Convert DOF control outputs (PID results) to thruster PWMs.
    """
    forward_left_pwm = PWM_NEUTRAL + alpha_surge*u_surge - alpha_yaw*u_yaw
    forward_right_pwm = PWM_NEUTRAL + alpha_surge*u_surge + alpha_yaw*u_yaw
    depth_left_pwm = PWM_NEUTRAL + beta_heave*u_heave
    depth_right_pwm = PWM_NEUTRAL + beta_heave*u_heave
    pitch_pwm = PWM_NEUTRAL + gamma_pitch*u_pitch

    thruster_pwms = []
    for val in [depth_left_pwm, pitch_pwm, depth_right_pwm, forward_left_pwm, forward_right_pwm]:
        val = max(PWM_MIN, min(PWM_MAX, val))
        val = map_pwm_value(val)
        thruster_pwms.append(val)
    return thruster_pwms

previous_pwm_values = [1500] * 5  # Store last sent PWM values

def send_pwm(pwm_values, max_change=20, flag=True):
    """
    Send PWM values to the Arduino with smooth transitions.
    Limits changes to `max_change` per cycle to avoid sudden jumps.
    """
    global previous_pwm_values
    if flag:
        smooth_pwms = [
            max(min(prev + max_change, pwm), prev - max_change)  # Gradually change
            for prev, pwm in zip(previous_pwm_values, pwm_values)
        ]
        
        command = ','.join(map(str, smooth_pwms)) + '\n'
        arduino.write(command.encode())
        previous_pwm_values = smooth_pwms  # Store last values
        print(f"Sent to Arduino: {command.strip()}")
    else:
        command = ','.join(map(str, pwm_values)) + '\n'
        arduino.write(command.encode())
        print(f"Sent to Arduino: {command.strip()}")
        


def initialize_thrusters():
    """
    Sends a neutral (1500) PWM signal to all thrusters upon startup.
    """
    neutral_pwms = [1500] * 5
    send_pwm(neutral_pwms)
    print("Initialized thrusters to neutral (1500).")
    time.sleep(2)


# =======================
# Sensor Error Functions (Stubs / Example)
# =======================
def get_surge_error():
    return 0

def get_yaw_error():
    euler_angles = imu.get_euler_angles()  # (roll, pitch, yaw) in degrees
    current_yaw = euler_angles[2]
    if current_yaw >= 180:
        current_yaw = 360-current_yaw
    else:
         current_yaw = -current_yaw
       
    target_yaw = TARGET_YAW
    error_yaw = target_yaw - current_yaw
    error_yaw *= -1
    if abs(error_yaw) < 2:
        error_yaw = 0
    print(f"current yaw {current_yaw}")
    print(f"error yaw {error_yaw}")
    return error_yaw

def get_heave_error():
    offset = 5
    depth = depth_sensor.get_depth()
    depth_2 = depth - offset
    print(f"current Depth_2 {depth_2}")
    print(f"current Depth {depth}")
    #depth = float(10)  # example value
    target = TARGET_DEPTH
    error = target - depth_2
    print(f"error Depth {error}")
    #return error if depth else 0
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
        current_time = time.time()
        # Get error signals
        e_surge = get_surge_error()
        e_yaw   = get_yaw_error()
        e_heave = get_heave_error()
        e_pitch = get_pitch_error()

        # Update each PID and obtain control outputs
        u_surge = pid_surge.update(e_surge, current_time)
        u_yaw   = pid_yaw.update(e_yaw, current_time)
        u_heave = pid_heave.update(e_heave, current_time)
        u_pitch = pid_pitch.update(e_pitch, current_time)

        # Mix into thruster PWMs and send commands
        thruster_pwms = mix_outputs(u_surge, u_yaw, u_heave, u_pitch)
        print(thruster_pwms)
        send_pwm(thruster_pwms)
        
        # Run at ~20 Hz
        time.sleep(0.05)

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
    main_thread.join()  # Wait for thread to stop completely

    print("[INFO] Sending 1500 to all thrusters (neutral)...")
    send_pwm([1500] * 5, flag= False)  # Ensure motors stop
    print("[INFO] Executed finally block. System shut down.")
    

