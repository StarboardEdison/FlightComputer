# Model Rocket Active Drag Control System


This repository contains the Arduino flight computer code for a model rocket active drag control system. The system reads sensor data, filters it, predicts the apogee using numerical integration, and determines when to deploy or retract airbrakes to hit a target altitude.


## Hardware Components

* Arduino compatible microcontroller

* MPU6050 6-DoF IMU

* Adafruit BMP3XX Precision Barometric Altimeter

* Micro SD Card Module

* Servo Motor


## Pin Configuration

* Servo Pin: 10

* SD Card CS Pin: 4

* BMP3XX: I2C address 0x77

* MPU6050: Standard I2C


## Core Features


### Sensor Fusion and Filtering

The codebase uses a 1D Kalman filter to smooth the altitude readings from the BMP3XX sensor. It calculates velocity and acceleration based on the change in these smoothed altitude readings over time. The MPU6050 data is processed using a Mahony AHRS algorithm to determine the roll, pitch, and yaw of the rocket.


### Apogee Prediction

A numerical integration loop simulates the remainder of the flight path during every cycle. It uses the current velocity, altitude, air density, rocket mass, and aerodynamic drag coefficients to predict the final apogee at 100ms time steps. 


### Deployment Logic

The target apogee is set to 750 feet. If the rocket is above the minimum deployment altitude of 300 feet and the predicted apogee exceeds 750 feet for 5 consecutive cycles, the system triggers the deployment state. If the predicted apogee falls 5 feet below the target for 5 consecutive cycles, it triggers the retraction state.


### Flight 1 Testing Mode

The current codebase is configured for a passive test flight. The drag coefficients for both deployed and retracted states are set to 0.559. The servo actuation commands within the deployment and retraction logic are commented out, and the servo is locked to 0 degrees during setup. This allows the system to log the theoretical behavior of the control loop without physically altering the flight profile.


## Data Logging

The system records flight data to the SD card at 50ms intervals. 


The main data file records the following metrics in a comma-separated format: Time in milliseconds, Altitude in feet, Velocity in feet per second, Acceleration X, Acceleration Y, Acceleration Z, Roll, Pitch, Yaw, Predicted Apogee in feet.


A secondary apogee log file records the maximum detected altitude once the rocket reaches apogee, detects a negative velocity, and begins descending.
