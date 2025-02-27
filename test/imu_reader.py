#!/usr/bin/env python
# -*- coding:utf-8 -*-
import serial
import struct
import platform
import threading
import time
import serial.tools.list_ports
import math


class IMUReader:
    def __init__(self, port="/dev/ttyUSB_IMU", baudrate=921600, timeout=0.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.angular_velocity = [0, 0, 0]  # [X, Y, Z] rad/s
        self.euler_angles = [0, 0, 0]  # [X, Y, Z] degrees
        self.running = False  # Flag for controlling the reading loop
        self.buffer = {}
        self.key = 0
        self.pub_flag = [True, True]
        self.python_version = platform.python_version()[0]

        self._connect_serial()

    def _connect_serial(self):
        """Establishes a serial connection with the IMU sensor."""
        try:
            self.serial_conn = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            if self.serial_conn.isOpen():
                print("\033[32mSerial port opened successfully...\033[0m")
            else:
                self.serial_conn.open()
                print("\033[32mSerial port opened successfully...\033[0m")
        except Exception as e:
            print(f"\033[31mFailed to open serial port: {e}\033[0m")
            exit(0)

    def _checksum(self, list_data, check_data):
        """CRC checksum verification."""
        data = bytearray(list_data)
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])

    def _hex_to_ieee(self, raw_data):
        """Converts hex data to IEEE floating point numbers."""
        ieee_data = []
        raw_data.reverse()
        for i in range(0, len(raw_data), 4):
            data2str = (
                hex(raw_data[i] | 0xff00)[4:6]
                + hex(raw_data[i + 1] | 0xff00)[4:6]
                + hex(raw_data[i + 2] | 0xff00)[4:6]
                + hex(raw_data[i + 3] | 0xff00)[4:6]
            )
            if self.python_version == '2':
                ieee_data.append(struct.unpack('>f', data2str.decode('hex'))[0])
            else:
                ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
        ieee_data.reverse()
        return ieee_data

    def _handle_serial_data(self, raw_data):
        """Processes serial data from the IMU sensor."""
        if self.python_version == '2':
            self.buffer[self.key] = ord(raw_data)
        else:
            self.buffer[self.key] = raw_data

        self.key += 1
        if self.buffer[0] != 0xaa:
            self.key = 0
            return
        if self.key < 3:
            return
        if self.buffer[1] != 0x55:
            self.key = 0
            return
        if self.key < self.buffer[2] + 5:
            return

        else:
            data_buff = list(self.buffer.values())

            if self.buffer[2] == 0x2c and self.pub_flag[0]:
                if self._checksum(data_buff[2:47], data_buff[47:49]):
                    data = self._hex_to_ieee(data_buff[7:47])
                    self.angular_velocity = data[1:4]
                else:
                    print('Checksum failed')
                self.pub_flag[0] = False

            elif self.buffer[2] == 0x14 and self.pub_flag[1]:
                if self._checksum(data_buff[2:23], data_buff[23:25]):
                    data = self._hex_to_ieee(data_buff[7:23])
                    self.euler_angles = data[1:4]
                else:
                    print('Checksum failed')
                self.pub_flag[1] = False
            else:
                print(f"No parser available for {str(self.buffer[2])} or data is incorrect")
                self.buffer = {}
                self.key = 0

            self.buffer = {}
            self.key = 0
            if self.pub_flag[0] or self.pub_flag[1]:
                return
            self.pub_flag[0] = self.pub_flag[1] = True

    def _read_loop(self):
        """Continuously reads IMU data in a separate thread."""
        while self.running:
            try:
                buffer_count = self.serial_conn.inWaiting()
                if buffer_count > 0:
                    buffer_data = self.serial_conn.read(buffer_count)
                    for i in range(buffer_count):
                        self._handle_serial_data(buffer_data[i])
            except Exception as e:
                print(f"Exception: {e}")
                print("IMU connection lost, loose connection, or disconnected")
                self.running = False  # Stop the loop
                break
            time.sleep(0.05)  # Adjust sampling rate

    def start_reading(self):
        """Starts a background thread to read IMU data continuously."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print("IMU reading started.")

    def stop_reading(self):
        """Stops the background thread reading IMU data."""
        self.running = False
        if self.thread:
            self.thread.join()
        print("IMU reading stopped.")

    def get_angular_velocity(self):
        """Returns the latest angular velocity (rad/s) as a tuple (x, y, z)."""
        return tuple(self.angular_velocity)

    def get_euler_angles(self):
        """Returns the latest Euler angles (degrees) as a tuple (x, y, z)."""
        return tuple(self.euler_angles)


if __name__ == "__main__":
    imu = IMUReader()
    imu.start_reading()  # Starts the background loop
    try:
        while True:
            print(f"Angular Velocity: {imu.get_angular_velocity()} rad/s")
            print(f"Euler Angles: {imu.get_euler_angles()}°")
            time.sleep(0.25)
    except KeyboardInterrupt:
        imu.stop_reading()

