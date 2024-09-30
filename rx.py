#################################################################################################################
# coding:utf-8
"""
Device Tracker Script
- Reads BLE MAC addresses and RSSI values from a USB serial port.
- Uses Kalman filter to smooth RSSI readings.
- Estimates distance from filtered RSSI.
- Tracks devices based on signal strength.
"""
#################################################################################################################
import serial
import threading
import time
import logging
import math
from collections import deque

# Initialize logging
logging.basicConfig(
    filename='detections.log',
    level=logging.INFO,
    format='%(asctime)s - %(message)s'
)

# Serial port configuration
SERIAL_PORT = '/dev/ttyACM0'  # Update with your serial port
BAUD_RATE = 115200            # Baud rate for USB CDC ACM
READ_TIMEOUT = 1              # Timeout for serial read

# RSSI to distance parameters
TX_POWER = -59                # Tx Power in dBm at 1 meter (adjust as needed)
PATH_LOSS_EXPONENT = 2.0      # Environmental factor (adjust based on environment)

# Alert thresholds in meters
ALERT_THRESHOLDS = {
    'Far': 1000.0,
    'Medium': 150.0,
    'Close': 25.0
}

# Device tracking data
devices = {}

# Maximum history length for RSSI values
MAX_HISTORY_LEN = 25

class KalmanFilter:
    def __init__(self, process_variance=1e-3, estimated_measurement_variance=1e-1):
        self.process_variance = process_variance  # Q
        self.estimated_measurement_variance = estimated_measurement_variance  # R
        self.posteri_estimate = None  # Initial estimate
        self.posteri_error_estimate = 1.0  # Initial error estimate

    def input_latest_noisy_measurement(self, measurement):
        if self.posteri_estimate is None:
            # First measurement
            self.posteri_estimate = measurement
            return self.posteri_estimate

        # Prediction step
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        # Update step
        blending_factor = priori_error_estimate / (
            priori_error_estimate + self.estimated_measurement_variance
        )
        self.posteri_estimate = priori_estimate + blending_factor * (
            measurement - priori_estimate
        )
        self.posteri_error_estimate = (
            1 - blending_factor
        ) * priori_error_estimate

        return self.posteri_estimate

def rssi_to_distance(rssi, tx_power=TX_POWER, n=PATH_LOSS_EXPONENT):
    """
    Convert RSSI to distance using the Log-distance Path Loss Model.
    """
    distance = 10 ** ((tx_power - rssi) / (10 * n))
    return round(distance, 2)

def get_alert_level(distance):
    """
    Determine the alert level based on distance.
    """
    if distance <= ALERT_THRESHOLDS['Close']:
        return 'Close'
    elif distance <= ALERT_THRESHOLDS['Medium']:
        return 'Medium'
    elif distance <= ALERT_THRESHOLDS['Far']:
        return 'Far'
    else:
        return 'Distant'

def update_device_tracking(mac_address, rssi):
    """
    Update the tracking information for a device.
    """
    if mac_address not in devices:
        devices[mac_address] = {
            'kalman_filter': KalmanFilter(),
            'distance': None,
            'movement': None,
            'rssi_history': deque(maxlen=MAX_HISTORY_LEN)
        }
    device = devices[mac_address]
    # Apply Kalman filter to the RSSI measurement
    filtered_rssi = device['kalman_filter'].input_latest_noisy_measurement(rssi)
    device['rssi_history'].append(filtered_rssi)
    # Calculate average filtered RSSI
    avg_rssi = sum(device['rssi_history']) / len(device['rssi_history'])
    distance = rssi_to_distance(avg_rssi)
    device['distance'] = distance
    # Determine movement
    if len(device['rssi_history']) > 1:
        if device['rssi_history'][-1] > device['rssi_history'][-2]:
            device['movement'] = 'Approaching'
        elif device['rssi_history'][-1] < device['rssi_history'][-2]:
            device['movement'] = 'Moving Away'
        else:
            device['movement'] = 'Stationary'
    else:
        device['movement'] = 'Unknown'
    # Log device information
    alert_level = get_alert_level(distance)
    log_message = (
        f"MAC: {mac_address}, Filtered RSSI: {filtered_rssi:.2f} dBm, "
        f"Distance: {distance} m, Alert Level: {alert_level}, "
        f"Movement: {device['movement']}"
    )
    print(log_message)
    logging.info(log_message)

def read_serial_data():
    """
    Read data from the serial port and process it.
    """
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=READ_TIMEOUT) as ser:
            print(f"Listening on {SERIAL_PORT}...")
            while True:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    # Expecting data in the format: MAC_ADDRESS,RSSI
                    try:
                        mac_address, rssi = line.split(',')
                        rssi = int(rssi)
                        update_device_tracking(mac_address, rssi)
                    except ValueError:
                        logging.error(f"Invalid data format: {line}")
    except serial.SerialException as e:
        logging.error(f"Serial exception: {e}")
        print(f"Error: {e}")

def main():
    serial_thread = threading.Thread(target=read_serial_data)
    serial_thread.daemon = True
    serial_thread.start()

    # Keep the main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")
        logging.info("Script terminated by user.")

if __name__ == "__main__":
    main()
