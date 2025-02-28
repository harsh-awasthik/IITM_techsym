import time, serial
import numpy as np
from ..test.imu_reader import IMUReader
from threading import Event
from ..test.pres import DepthSensor

alpha_surge  = 1.0
alpha_yaw    = 1.0
beta_heave   = 1.0
gamma_pitch  = 1.0

PWM_NEUTRAL  = 1500
PWM_MIN      = 1100
PWM_MAX      = 1900

depth_sensor = DepthSensor('/dev/ttyUSB_DEPTH')
imu = IMUReader(port="/dev/ttyUSB_IMU", baudrate=921600)
arduino_port_pwm = '/dev/ttyUSB_PWM'
baud_rate = 115200

# Start background reading for IMU
imu.start_reading()

# Set up serial communication to Arduino for thrusters
arduino = serial.Serial(arduino_port_pwm, baud_rate)
time.sleep(2)  # Wait for Arduino to initialize


tuple_points = np.ones([4,2])

stop_event = Event()

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
        return 1500



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


def initialize_thrusters():
    """
    Sends a neutral (1500) PWM signal to all thrusters upon startup.
    """
    neutral_pwms = [1500] * 5
    send_pwm(neutral_pwms)
    print("Initialized thrusters to neutral (1500).")
    time.sleep(2)

previous_pwm_values = [1500] * 5  # Store last sent PWM values
def send_pwm(pwm_values, max_change=20, flag=True):
    """
    Send PWM values to the Arduino with smooth transitions.
    Limits changes to max_change per cycle to avoid sudden jumps.
    """
    if flag == True:
        global previous_pwm_values
        smooth_pwms = [
            max(min(prev + max_change, pwm), prev - max_change)  # Gradually change
            for prev, pwm in zip(previous_pwm_values, pwm_values)
        ]
    
        command = ','.join(map(str, smooth_pwms)) + '\n'
        arduino.write(command.encode())
        previous_pwm_values = smooth_pwms  # Store last values``
        print(f"Sent to Arduino: {command.strip()}")
    else:
        command = ','.join(map(str, smooth_pwms)) + '\n'
        arduino.write(command.encode())
        previous_pwm_values = smooth_pwms  # Store last values
        print(f"Sent to Arduino: {command.strip()}")

    
    