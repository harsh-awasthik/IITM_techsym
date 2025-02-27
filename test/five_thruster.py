import serial
import time

# Set up serial communication
arduino_port = '/dev/ttyUSB_PWM'  # Update with the correct port for your Arduino
baud_rate = 115200
arduino = serial.Serial(arduino_port, baud_rate)
time.sleep(2)  # Wait for the Arduino to initialize

def send_pwm_values(pwm_values):
    """
    Send PWM values for 5 thrusters to the Arduino.
    :param pwm_values: A list of 5 integers representing PWM values.
    """
    if len(pwm_values) != 5:
        print("Error: Exactly 5 PWM values are required.")
        return

    # Create the command string
    command = ','.join(map(str, pwm_values)) + '\n'

    # Send the command to Arduino
    arduino.write(command.encode())
    print(f"Sent to Arduino: {command.strip()}")

try:
    print("Enter PWM values for 5 thrusters separated by commas (e.g., 1500,1600,1550,1520,1580)")
    while True:
        user_input = input("Enter PWM values (or type 'exit' to quit): ").strip()
        if user_input.lower() == 'exit':
            print("Exiting.")
            break

        # Parse the user input into a list of integers
        try:
            pwm_values = list(map(int, user_input.split(',')))
            if len(pwm_values) != 5:
                print("Error: Please enter exactly 5 PWM values.")
                continue
        except ValueError:
            print("Error: Please enter valid integers separated by commas.")
            continue

        # Send the PWM values to Arduino
        send_pwm_values(pwm_values)

except KeyboardInterrupt:
    print("\nExiting program.")

finally:
    arduino.close()

