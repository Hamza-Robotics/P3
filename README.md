# Face-Controlled Robot for Tetraplegic Patients

## Overview

This project focuses on developing a face-controlled robot system to aid tetraplegic patients in picking and placing objects. The system utilizes a C++ library for controlling a robot with DYNAMIXEL motors using an Arduino. Additionally, data from facial EMG signals and an accelerometer is extracted. ¨

## Relevant Directory

### 1. `src/libs/Control_System`

- This directory contains the computed torque control system for the robot. The computed torque approach is utilized to control the movement of the robot joints based on the extracted data from facial EMG signals and accelerometer readings.
### 2. `src/libs/Dynamixel_Lib`

- This directory houses the C++ library responsible for controlling the DYNAMIXEL motors on the robot and interfacing with the facial EMG signals. It handles the communication with the Arduino and the implementation of protocols to control individual motors.

## License

This project is licensed under the [MIT License](LICENSE).
