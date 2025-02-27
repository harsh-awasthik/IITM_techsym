import serial
import time

class DepthSensor:
    def __init__(self, port='/dev/ttyUSB_DEPTH', baudrate=9600, timeout=1):
        try:
            self.arduino = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(3)  # Allow time for Arduino to initialize
            self.arduino.flushInput()  # Clear buffer
            print(f"Connected to Arduino on {port}")
        except Exception as e:
            print(f"Failed to connect to Arduino: {e}")
            self.arduino = None

    def get_depth(self):
        """Reads depth data from the Arduino and returns it as a float."""
        if not self.arduino:
            print("Error: Serial connection not initialized.")
            return None

        while True:
            try:
                if self.arduino.in_waiting > 0:  # Check if data is available
                    depth_data = self.arduino.readline().decode('utf-8').strip()
                    if depth_data:  # Ensure data is not empty
                        try:
                            depth_value = float(depth_data)  # Convert to float for PID
                            print(f"Depth = {depth_value} cm")
                            return depth_value
                        except ValueError:
                            print(f"Invalid depth data: {depth_data}")
            except Exception as e:
                print(f"Error reading depth: {e}")
                return None

            time.sleep(0.1)  # Prevent CPU overload

# Run standalone if executed directly
if __name__ == "__main__":
    depth_sensor = DepthSensor()
    while True:
        depth = depth_sensor.get_depth()
        
        time.sleep(1)

