
import time
from threading import Thread
import numpy as np
from pid import *
from plot import *

TARGET_DEPTH = 40.0
SEC_TO_MOVE_FORWARD = 2


pid_surge = PIDController(kp=0.5, ki=0.0, kd=0.1)
pid_yaw   = PIDController(kp=1.0, ki=0.0, kd=0.1)
pid_heave = PIDController(kp=4.0, ki=0.3, kd=3.0)
pid_pitch = PIDController(kp=1.0, ki=0.0, kd=0.2)


def move_forward():
    
    while get_yaw_error(0) != 0:
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
                
    
    LT1 = time.time()

    while time.time() < (LT1 + SEC_TO_MOVE_FORWARD):
        e_heave = get_heave_error()
        e_pitch = get_pitch_error()
        e_yaw = get_yaw_error()

        # Update each PID and obtain control outputs
        #u_surge = pid_surge.update(e_surge, current_time)
        u_surge = 50 
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

def get_yaw_error(target_yaw = 0.0):
    euler_angles = imu.get_euler_angles()  # (roll, pitch, yaw) in degrees
    current_yaw = euler_angles[2]
    if current_yaw >= 180:
        current_yaw -= 360
        
    # target_yaw = 0.0
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
        current_time = time.time()
        print("-" * 30)
        
        move_forward()

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