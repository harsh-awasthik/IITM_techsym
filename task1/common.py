import task1

def get_pid():
    pid_surge = task1.pid_surge
    pid_yaw   = task1.pid_yaw
    pid_heave = task1.pid_heave
    pid_pitch = task1.pid_pitch

    return (pid_surge, pid_yaw, pid_pitch, pid_heave)