#region Copyright (c) 2024, Jack Leighton
#     __________________________________________________________________________________________________________________
#
#                  __                   __              __________                                      __   
#                _/  |_  ____   _______/  |_  __________\______   \_______   ____   ______ ____   _____/  |_ 
#                \   __\/ __ \ /  ___/\   __\/ __ \_  __ \     ___/\_  __ \_/ __ \ /  ___// __ \ /    \   __\
#                 |  | \  ___/ \___ \  |  | \  ___/|  | \/    |     |  | \/\  ___/ \___ \\  ___/|   |  \  |  
#                 |__|  \___  >____  > |__|  \___  >__|  |____|     |__|    \___  >____  >\___  >___|  /__|  
#                           \/     \/            \/                             \/     \/     \/     \/      
#                                                          .__       .__  .__          __                    
#                               ____________   ____   ____ |__|____  |  | |__| _______/  |_                  
#                              /  ___/\____ \_/ __ \_/ ___\|  \__  \ |  | |  |/  ___/\   __\                 
#                              \___ \ |  |_> >  ___/\  \___|  |/ __ \|  |_|  |\___ \  |  |                   
#                             /____  >|   __/ \___  >\___  >__(____  /____/__/____  > |__|                   
#                                  \/ |__|        \/     \/        \/             \/                         
#                                  __                         __  .__                                        
#                   _____   __ ___/  |_  ____   _____   _____/  |_|__|__  __ ____                            
#                   \__  \ |  |  \   __\/  _ \ /     \ /  _ \   __\  \  \/ // __ \                           
#                    / __ \|  |  /|  | (  <_> )  Y Y  (  <_> )  | |  |\   /\  ___/                           
#                   (____  /____/ |__|  \____/|__|_|  /\____/|__| |__| \_/  \___  >                          
#                        \/                         \/                          \/                           
#                                                  .__          __  .__                                      
#                                       __________ |  |  __ ___/  |_|__| ____   ____   ______                
#                                      /  ___/  _ \|  | |  |  \   __\  |/  _ \ /    \ /  ___/                
#                                      \___ (  <_> )  |_|  |  /|  | |  (  <_> )   |  \\___ \                 
#                                     /____  >____/|____/____/ |__| |__|\____/|___|  /____  >                
#                                          \/                                      \/     \/                 
#                                   Tester Present Specialist Automotive Solutions
#     __________________________________________________________________________________________________________________
#      |--------------------------------------------------------------------------------------------------------------|
#      |       https://github.com/jakka351/| https://testerPresent.com.au | https://facebook.com/testerPresent        |
#      |--------------------------------------------------------------------------------------------------------------|
#      | Copyright (c) 2022/2023/2024 Benjamin Jack Leighton                                                          |          
#      | All rights reserved.                                                                                         |
#      |--------------------------------------------------------------------------------------------------------------|
#        Redistribution and use in source and binary forms, with or without modification, are permitted provided that
#        the following conditions are met:
#        1.    With the express written consent of the copyright holder.
#        2.    Redistributions of source code must retain the above copyright notice, this
#              list of conditions and the following disclaimer.
#        3.    Redistributions in binary form must reproduce the above copyright notice, this
#              list of conditions and the following disclaimer in the documentation and/or other
#              materials provided with the distribution.
#        4.    Neither the name of the organization nor the names of its contributors may be used to
#              endorse or promote products derived from this software without specific prior written permission.
#      _________________________________________________________________________________________________________________
#      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#      INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#      DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#      SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#      WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
#      USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#      _________________________________________________________________________________________________________________
#
#       This software can only be distributed with my written permission. It is for my own educational purposes and  
#       is potentially dangerous to ECU health and safety. Gracias a Gato Blancoford desde las alturas del mar de chelle.                                                        
#      _________________________________________________________________________________________________________________
#
#
#//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endregion License
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
