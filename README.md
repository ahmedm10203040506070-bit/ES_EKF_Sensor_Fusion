# ES-EKF GNSS/IMU Sensor Fusion Workbench

A comprehensive MATLAB App Designer-based graphical user interface (GUI) designed for tuning, testing, and analyzing an Error-State Extended Kalman Filter (ES-EKF) for multi-sensor integration (GNSS and IMU).

## Project Files
- `ES.mlapp`: The main MATLAB App Designer graphical interface files.
- `error_plots1.m` (and `error_plots2.m`): Specialized scripts for generating 6-DOF error plots and covariance bounds.
- `es_ekf.m`: Core implementation of the Error-State Extended Kalman Filter algorithm.
- `rotation.m`: Helper functions for quaternion operations and spatial transformations.
- `pt1_data.mat` / `pt3_data.mat`: Sample datasets used for testing and validating filter performance.
- `README.md`: Project documentation file.

## Key Features
- Dynamic Parameter Tuning: Real-time adjustment of process noise (Q matrix) and measurement noise (R matrix) using interactive UI sliders.
- Sensor Robustness Testing: Individual enable/disable capabilities for GNSS, Accelerometer, and Gyroscope sensors to evaluate filter resilience.
- Live 3D Visualization: Instant trajectory plotting comparing estimated paths against ground truth data.
- Error State Analytics: Automatic generation of position and attitude error analytics alongside covariance bounds directly within the dashboard.

## How to Run
1. Clone or download all repository files into a single local directory.
2. Open MATLAB and set your Current Folder to that directory.
3. Open `ES.mlapp` in MATLAB App Designer.
4. Click Run, load your dataset, adjust your tuning parameters, and execute the analysis.

---
Developed as an advanced research and engineering workbench for navigation systems and sensor fusion.
