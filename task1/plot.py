import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
from pid import *
import common

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

(pid_surge, pid_yaw, pid_pitch, pid_heave) = common.get_pid()

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
