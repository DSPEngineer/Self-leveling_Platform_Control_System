# Self-leveling Platform Control System

- BEE 526 Advanced Embedded Systems Design
-   Spring 2025
- Designer: Jose Pagan
-  Started: 04/07/2025

## 05/05/2025
- Weekend Goals:


## 04/30/2025
- Weekend goals:
  - Learn Euler angles
  - Learn more about digital LPF
  - apply this to accelerometer and gyro readings
- Update code and initiate github


## 04/25/2025 
- Weekend goals:
  - Complete Robot arm
  - Understand Motor control
- Update code and initiate github


## 04/15/2025
- Consulted with Dr. Berger on I2C debug
- Tried all of the above, found the module was defective
  - it seems likely I connected to 5.0 V instead of 3.3V
  - No data on the bus, from the device
  - remove device, see data from Teensy is unchanged
  - replace unit, all is working


## 04/09/2025
- Consulted with Dr. Berger on I2C debug techniques
- Suggestions
  - Ckeck connections (SCA, SCD, etc)
  - Look at signals with logic probe AND oscilloscope
  - Use terminating resistors (need to lookup sizing)
  - try for device ID (whoami) message

## 04/01/2025
- Breadbaord Teents 4.1 + MPU6050
- Not working, cannot get I2C across the bus
