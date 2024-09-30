# BLUETOOTH LOW ENERGY PROXIMITY & DISTANCE DETECTION 

##

Project: Long Range Proximity Detection of Bluetooth Low Energy Devices. Distance Estimation using Signal Strength RSSI.  

Hardware: Fanstel BU840XE Unit

Basis: Fanstel BU840XE Unit is programmed with firmware that scans for bluetooth device MAC addresss and RSSI, forwards that information onto a USB-Serial Port. A  python script on the other end recieves the data and uses a filter and algorithim to deterine approximate distance from the RSSI. The software is able to determine whether a device is stationary, moving away or moving toward you.

